from __future__ import annotations

import math
import os
from dataclasses import dataclass

import pygame

from models import Level
from parser import ParseError, Parser
from scheduler import Simulation

INK = (31, 41, 59)
INK_LIGHT = (248, 250, 252)
LINE_COLOR = (130, 144, 160)
RESTRICTED_RING = (194, 65, 12)
PRIORITY_RING = (8, 145, 178)
DRONE_COLORS = [
    (239, 68, 68),
    (245, 158, 11),
    (34, 197, 94),
    (59, 130, 246),
    (139, 92, 246),
    (236, 72, 153),
    (20, 184, 166),
    (249, 115, 22),
    (16, 185, 129),
    (99, 102, 241),
]
MIN_FONT = 8
MAX_ZOOM = 8.0


@dataclass
class _Layout:
    """Where the map sits on screen before zoom and pan are applied."""

    scale: float
    min_x: int
    min_y: int
    left: float
    bottom: float
    radius: float


class SimulationVisualizer:
    """Pygame visualizer with a map selection start screen."""

    def __init__(self, initial_map_path: str | None = None) -> None:
        self.width = 1800
        self.height = 980
        self.background = (245, 239, 230)
        self.running = True
        self.mode = "menu"

        self.move_duration_ms = 1000
        self.start_delay_ms = 1200
        self.transition_progress = 0.0
        self.turn_index = 0
        self.last_tick = 0
        self.start_time = 0

        self.level: Level | None = None
        self.turns: list[dict[int, str]] = []
        self.map_name = ""
        self.error_message: str | None = None

        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.dragging = False
        self._layout: _Layout | None = None
        self._layout_size = (0, 0)
        self._fonts: dict[int, pygame.font.Font] = {}

        self.maps = self._discover_maps()
        self.selected_map_index = 0

        pygame.init()
        pygame.display.set_caption("Fly-In Drone Simulation")
        self.screen = pygame.display.set_mode(
            (self.width, self.height), pygame.RESIZABLE
        )
        self.clock = pygame.time.Clock()
        self.font = pygame.font.SysFont("arial", 28, True)
        self.small_font = pygame.font.SysFont("arial", 20, True)

        if initial_map_path is not None:
            self._start_simulation(initial_map_path)

    def _discover_maps(self) -> list[str]:
        """Return all map files found under the project maps directory."""
        base_dir = os.path.dirname(os.path.abspath(__file__))
        maps_dir = os.path.join(base_dir, "maps")
        map_files: list[str] = []
        for root, _, files in os.walk(maps_dir):
            for filename in files:
                if filename.endswith(".txt"):
                    map_files.append(os.path.join(root, filename))
        map_files.sort()
        return map_files

    def _start_simulation(self, map_path: str) -> None:
        """Load a map and switch to simulation mode."""
        try:
            level = Parser().parse(map_path)
            turns = Simulation(level).run_with_states()
        except (OSError, ParseError, ValueError, RuntimeError) as error:
            self.error_message = f"Error: {error}"
            self.mode = "menu"
            return

        self.level = level
        self.turns = turns
        self.map_name = os.path.basename(map_path)
        self.turn_index = 0
        self.transition_progress = 0.0
        self.last_tick = pygame.time.get_ticks()
        self.start_time = self.last_tick
        self.error_message = None
        self._reset_view()
        self.mode = "simulation"

    def _reset_view(self) -> None:
        """Forget zoom, pan and the cached layout."""
        self.zoom = 1.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self.dragging = False
        self._layout = None

    # ------------------------------------------------------------ layout

    def _min_spacing(self) -> float:
        """Smallest distance between two hubs in map coordinates."""
        level = self._require_level()
        points = [(hub.loc_x, hub.loc_y) for hub in level.hubs.values()]
        best = math.inf
        for index, first in enumerate(points):
            for second in points[index + 1:]:
                distance = math.dist(first, second)
                if 0 < distance < best:
                    best = distance
        return 1.0 if best == math.inf else best

    def _get_layout(self) -> _Layout:
        """Return the cached layout, recomputing it after a resize."""
        size = (self.width, self.height)
        if self._layout is None or self._layout_size != size:
            self._layout = self._compute_layout()
            self._layout_size = size
        return self._layout

    def _compute_layout(self) -> _Layout:
        """Fit the map uniformly into the window and size the nodes.

        The node radius follows the closest pair of hubs, so a crowded
        map gets smaller nodes instead of overlapping ones.
        """
        level = self._require_level()
        xs = [hub.loc_x for hub in level.hubs.values()]
        ys = [hub.loc_y for hub in level.hubs.values()]
        min_x, min_y = min(xs), min(ys)
        span_x = max(max(xs) - min_x, 1)
        span_y = max(max(ys) - min_y, 1)
        top, bottom_margin, side = 90.0, 70.0, 40.0
        avail_w = self.width - 2 * side
        avail_h = self.height - top - bottom_margin
        spacing = self._min_spacing()

        pad = 60.0
        scale, radius = 1.0, 12.0
        for _ in range(2):
            scale = max(
                min((avail_w - 2 * pad) / span_x,
                    (avail_h - 2 * pad) / span_y),
                1.0,
            )
            radius = min(max(spacing * scale * 0.4, 12.0), 60.0)
            pad = radius * 2.2 + 10
        used_w = (max(xs) - min_x) * scale
        used_h = (max(ys) - min_y) * scale
        return _Layout(
            scale=scale,
            min_x=min_x,
            min_y=min_y,
            left=(self.width - used_w) / 2,
            bottom=top + (avail_h + used_h) / 2,
            radius=radius,
        )

    def _to_screen(self, base_x: float, base_y: float) -> tuple[float, float]:
        """Apply zoom and pan around the window center."""
        center_x, center_y = self.width / 2, self.height / 2
        return (
            center_x + (base_x - center_x) * self.zoom + self.pan_x,
            center_y + (base_y - center_y) * self.zoom + self.pan_y,
        )

    def _hub_center(self, name: str) -> tuple[float, float]:
        """Screen position of a hub."""
        layout = self._get_layout()
        hub = self._require_level().hubs[name]
        return self._to_screen(
            layout.left + (hub.loc_x - layout.min_x) * layout.scale,
            layout.bottom - (hub.loc_y - layout.min_y) * layout.scale,
        )

    def _hub_radius(self, name: str) -> float:
        """Screen radius of a hub (start and end are drawn larger)."""
        level = self._require_level()
        radius = self._get_layout().radius * self.zoom
        return radius * 1.25 if name in (level.start, level.end) else radius

    # ------------------------------------------------------------ drawing

    def _font(self, size: int) -> pygame.font.Font:
        """Return a cached bold font of the given size."""
        font = self._fonts.get(size)
        if font is None:
            font = pygame.font.SysFont("arial", size, True)
            self._fonts[size] = font
        return font

    def _draw_text(
        self,
        text: str,
        color: tuple[int, int, int],
        center: tuple[float, float],
        size: int,
        max_width: float,
    ) -> None:
        """Draw centered text, shrinking or shortening it to fit."""
        if max_width < 10:
            return
        size = max(size, MIN_FONT)
        font = self._font(size)
        while size > MIN_FONT and font.size(text)[0] > max_width:
            size -= 1
            font = self._font(size)
        shown = text
        while len(shown) > 1 and font.size(shown)[0] > max_width:
            shown = shown.rstrip("…")[:-1] + "…"
        label = font.render(shown, True, color)
        rect = label.get_rect(center=(int(center[0]), int(center[1])))
        self.screen.blit(label, rect)

    def _hub_color(self, hub_name: str) -> tuple[int, int, int]:
        """Return a display color for a hub."""
        level = self._require_level()
        hub = level.hubs[hub_name]
        if hub.color is not None:
            palette = {
                "green": (34, 197, 94),
                "yellow": (250, 204, 21),
                "blue": (96, 165, 250),
                "red": (248, 113, 113),
                "gray": (148, 163, 184),
                "orange": (251, 146, 60),
                "cyan": (34, 211, 238),
                "purple": (167, 139, 250),
            }
            return palette.get(hub.color.lower(), (121, 134, 203))
        if hub.zone_type.value == "restricted":
            return (249, 115, 22)
        if hub.zone_type.value == "priority":
            return (56, 189, 248)
        if hub.zone_type.value == "blocked":
            return (100, 116, 139)
        return (125, 211, 252)

    def _text_color(self, fill: tuple[int, int, int]) -> tuple[int, int, int]:
        """Dark text on light fills, light text on dark fills."""
        brightness = 0.299 * fill[0] + 0.587 * fill[1] + 0.114 * fill[2]
        return INK if brightness > 140 else INK_LIGHT

    def _draw_connections(self) -> None:
        """Draw connection lines; thicker lines carry more drones."""
        level = self._require_level()
        base = max(2.0, self._get_layout().radius * self.zoom * 0.12)
        for connection in level.connections:
            start = self._hub_center(connection.point1)
            end = self._hub_center(connection.point2)
            factor = 1 + 0.6 * (connection.max_link_capacity - 1)
            width = int(min(base * factor, base * 4))
            pygame.draw.line(
                self.screen,
                LINE_COLOR,
                (int(start[0]), int(start[1])),
                (int(end[0]), int(end[1])),
                width=max(width, 2),
            )

    def _draw_hubs(self) -> None:
        """Draw each hub as a node with its name inside."""
        level = self._require_level()
        for name, hub in level.hubs.items():
            center_x, center_y = self._hub_center(name)
            radius = self._hub_radius(name)
            if (
                center_x + radius < 0
                or center_y + radius < 0
                or center_x - radius > self.width
                or center_y - radius > self.height
            ):
                continue
            center = (int(center_x), int(center_y))
            outline = max(2, int(radius * 0.08))
            ring = {
                "restricted": RESTRICTED_RING,
                "priority": PRIORITY_RING,
            }.get(hub.zone_type.value)
            if ring is not None:
                pygame.draw.circle(
                    self.screen,
                    ring,
                    center,
                    int(radius + outline * 2 + 2),
                    width=max(2, int(radius * 0.07)),
                )
            fill = self._hub_color(name)
            pygame.draw.circle(self.screen, fill, center, int(radius))
            pygame.draw.circle(
                self.screen, INK, center, int(radius), width=outline
            )
            if hub.zone_type.value == "blocked":
                offset = int(radius * 0.7)
                for sign in (1, -1):
                    pygame.draw.line(
                        self.screen,
                        INK,
                        (center[0] - offset, center[1] - sign * offset),
                        (center[0] + offset, center[1] + sign * offset),
                        width=outline,
                    )
            color = self._text_color(fill)
            show_capacity = (
                name not in (level.start, level.end)
                and hub.max_drones > 1
                and radius >= 28
            )
            shift = radius * 0.16 if show_capacity else 0.0
            self._draw_text(
                name,
                color,
                (center_x, center_y - shift),
                int(radius * 0.5),
                radius * 1.7,
            )
            if show_capacity:
                self._draw_text(
                    f"max {hub.max_drones}",
                    color,
                    (center_x, center_y + radius * 0.42),
                    int(radius * 0.3),
                    radius * 1.3,
                )

    def _draw_drones(self) -> None:
        """Draw drones stacked on their hub, gliding along connections."""
        level = self._require_level()
        last = len(self.turns) - 1
        before = min(self.turn_index, last)
        after = min(before + 1, last)
        progress = self.transition_progress if after != before else 0.0
        old_state, new_state = self.turns[before], self.turns[after]
        chip = min(max(self._get_layout().radius * self.zoom * 0.3, 7.0), 18.0)

        resting: list[tuple[int, float, float]] = []
        moving: list[tuple[int, float, float]] = []
        for drone_id in range(1, level.nb_drones + 1):
            old_hub, new_hub = old_state[drone_id], new_state[drone_id]
            start_x, start_y = self._hub_center(old_hub)
            end_x, end_y = self._hub_center(new_hub)
            point_x = start_x + (end_x - start_x) * progress
            point_y = start_y + (end_y - start_y) * progress
            target = moving if old_hub != new_hub else resting
            target.append((drone_id, point_x, point_y))

        for drone_id, point_x, point_y in resting + moving:
            center = (int(point_x), int(point_y))
            color = DRONE_COLORS[(drone_id - 1) % len(DRONE_COLORS)]
            pygame.draw.circle(self.screen, color, center, int(chip))
            pygame.draw.circle(
                self.screen, INK, center, int(chip), width=2
            )
            self._draw_text(
                str(drone_id),
                INK,
                (point_x, point_y),
                int(chip * 1.1),
                chip * 1.7,
            )

    def _draw_hud(self) -> None:
        """Draw the turn counter and the legend."""
        total = max(len(self.turns) - 1, 1)
        shown = min(self.turn_index + 1, total)
        label = self.font.render(f"Turn {shown}/{total}", True, INK)
        self.screen.blit(label, (24, 18))
        name = self.small_font.render(self.map_name, True, (71, 85, 105))
        self.screen.blit(name, (24, 56))

        y = self.height - 34
        x = 24
        for ring, text in (
            (RESTRICTED_RING, "restricted (2 turns)"),
            (PRIORITY_RING, "priority"),
        ):
            pygame.draw.circle(self.screen, ring, (x + 10, y + 10), 10, 3)
            caption = self.small_font.render(text, True, INK)
            self.screen.blit(caption, (x + 28, y))
            x += 28 + caption.get_width() + 28
        hint = "thick line = more link capacity   |   " \
            "wheel: zoom   drag: move   R: reset   Esc: quit"
        caption = self.small_font.render(hint, True, (71, 85, 105))
        self.screen.blit(caption, (x, y))

    def _render_simulation(self) -> None:
        """Render one simulation frame."""
        self.screen.fill(self.background)
        self._draw_connections()
        self._draw_hubs()
        self._draw_drones()
        self._draw_hud()

    def _render_menu(self) -> None:
        """Render the map selection start screen."""
        self.screen.fill(self.background)

        title = self.font.render("Fly-In Map Selector", True, (17, 24, 39))
        self.screen.blit(title, (36, 28))

        help_text = (
            "UP/DOWN: select map  ENTER: start  "
            "ESC/Q: quit  (after animation it returns here)"
        )
        controls = self.small_font.render(help_text, True, (71, 85, 105))
        self.screen.blit(controls, (36, 80))

        if self.error_message is not None:
            error = self.small_font.render(
                self.error_message,
                True,
                (185, 28, 28),
            )
            self.screen.blit(error, (36, 116))

        if not self.maps:
            empty = self.font.render(
                "No map files found in ./maps",
                True,
                (31, 41, 59),
            )
            self.screen.blit(empty, (36, 170))
            return

        top = 170
        row_height = 44
        max_visible = 16
        start = max(self.selected_map_index - max_visible // 2, 0)
        end = min(start + max_visible, len(self.maps))
        if end - start < max_visible:
            start = max(end - max_visible, 0)

        for visible_index, absolute_index in enumerate(range(start, end)):
            map_path = self.maps[absolute_index]
            rel_path = os.path.relpath(map_path, os.getcwd())
            y = top + visible_index * row_height
            is_selected = absolute_index == self.selected_map_index
            if is_selected:
                highlight = pygame.Rect(30, y - 6, self.width - 60, row_height)
                pygame.draw.rect(
                    self.screen,
                    (220, 229, 245),
                    highlight,
                    border_radius=8,
                )

            prefix = "▶ " if is_selected else "  "
            line = self.small_font.render(
                f"{prefix}{absolute_index + 1}. {rel_path}",
                True,
                (17, 24, 39),
            )
            self.screen.blit(line, (40, y))

    def _require_level(self) -> Level:
        """Return loaded level or fail if simulation is not ready."""
        if self.level is None:
            raise RuntimeError("No active simulation level.")
        return self.level

    # ------------------------------------------------------------ input

    def _handle_menu_key(self, key: int) -> None:
        """Handle key presses while in menu mode."""
        if key in (pygame.K_ESCAPE, pygame.K_q):
            self.running = False
            return
        if not self.maps:
            return
        if key == pygame.K_DOWN:
            self.selected_map_index = min(
                self.selected_map_index + 1,
                len(self.maps) - 1,
            )
            return
        if key == pygame.K_UP:
            self.selected_map_index = max(self.selected_map_index - 1, 0)
            return
        if key in (pygame.K_RETURN, pygame.K_KP_ENTER):
            self._start_simulation(self.maps[self.selected_map_index])

    def _handle_simulation_key(self, key: int) -> None:
        """Handle key presses while in simulation mode."""
        if key in (pygame.K_ESCAPE, pygame.K_q):
            self.running = False
        elif key == pygame.K_r:
            self._reset_view()

    def _zoom_at(self, mouse: tuple[int, int], steps: int) -> None:
        """Zoom the view while keeping the point under the mouse fixed."""
        new_zoom = min(max(self.zoom * 1.15 ** steps, 1.0), MAX_ZOOM)
        ratio = new_zoom / self.zoom
        offset_x = mouse[0] - self.width / 2
        offset_y = mouse[1] - self.height / 2
        self.pan_x = offset_x - (offset_x - self.pan_x) * ratio
        self.pan_y = offset_y - (offset_y - self.pan_y) * ratio
        self.zoom = new_zoom
        if self.zoom == 1.0:
            self.pan_x = self.pan_y = 0.0

    def _handle_mouse(self, event: pygame.event.Event) -> None:
        """Zoom with the wheel and move the map by dragging."""
        if event.type == pygame.MOUSEWHEEL:
            self._zoom_at(pygame.mouse.get_pos(), event.y)
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            self.dragging = True
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            self.pan_x += event.rel[0]
            self.pan_y += event.rel[1]

    def run(self) -> None:
        """Run the application until closed."""
        while self.running:
            self.width, self.height = self.screen.get_size()
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.VIDEORESIZE:
                    surface = pygame.display.get_surface()
                    if surface is not None:
                        self.screen = surface
                elif event.type == pygame.KEYDOWN:
                    if self.mode == "menu":
                        self._handle_menu_key(event.key)
                    else:
                        self._handle_simulation_key(event.key)
                elif self.mode == "simulation":
                    self._handle_mouse(event)

            now = pygame.time.get_ticks()
            delta = now - self.last_tick
            self.last_tick = now

            if self.mode == "simulation":
                if now - self.start_time >= self.start_delay_ms:
                    if self.turn_index < len(self.turns) - 1:
                        self.transition_progress += (
                            delta / self.move_duration_ms
                        )
                        if self.transition_progress >= 1.0:
                            self.transition_progress = 0.0
                            self.turn_index += 1
                    else:
                        self.mode = "menu"
                        self.level = None
                        self.turns = []
                        self._render_menu()
                        pygame.display.flip()
                        self.clock.tick(60)
                        continue
                self._render_simulation()
            else:
                self._render_menu()

            pygame.display.flip()
            self.clock.tick(60)

        pygame.quit()


def run_gui(map_path: str | None = None) -> None:
    """Open the visual simulator (menu if no map is preselected)."""
    app = SimulationVisualizer(map_path)
    app.run()
