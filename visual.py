from __future__ import annotations

import os

import pygame

from models import Level
from parser import ParseError, Parser
from scheduler import Simulation


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
        self.error_message: str | None = None

        self.maps = self._discover_maps()
        self.selected_map_index = 0

        pygame.init()
        pygame.display.set_caption("Fly-In Drone Simulation")
        self.screen = pygame.display.set_mode((self.width, self.height))
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
        self.turn_index = 0
        self.transition_progress = 0.0
        self.last_tick = pygame.time.get_ticks()
        self.start_time = self.last_tick
        self.error_message = None
        self.mode = "simulation"

    def _world_to_screen(self, x: float, y: float) -> tuple[int, int]:
        """Convert map coordinates into screen coordinates."""
        level = self._require_level()
        min_x = min(hub.loc_x for hub in level.hubs.values())
        max_x = max(hub.loc_x for hub in level.hubs.values())
        min_y = min(hub.loc_y for hub in level.hubs.values())
        max_y = max(hub.loc_y for hub in level.hubs.values())

        width = max(max_x - min_x, 1)
        height = max(max_y - min_y, 1)
        scale_x = 1280 / width
        scale_y = 710 / height
        px = 220 + (x - min_x) * scale_x
        py = self.height - 220 - (y - min_y) * scale_y
        return int(px), int(py)

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

    def _draw_connections(self) -> None:
        """Draw all connection lines between hubs."""
        level = self._require_level()
        for connection in level.connections:
            start = self._world_to_screen(
                level.hubs[connection.point1].loc_x,
                level.hubs[connection.point1].loc_y,
            )
            end = self._world_to_screen(
                level.hubs[connection.point2].loc_x,
                level.hubs[connection.point2].loc_y,
            )
            pygame.draw.line(self.screen, (91, 105, 117), start, end, width=10)

    def _draw_hubs(self) -> None:
        """Draw each hub as a node."""
        level = self._require_level()
        for name, hub in level.hubs.items():
            center = self._world_to_screen(hub.loc_x, hub.loc_y)
            radius = 82 if name in (level.start, level.end) else 52
            pygame.draw.circle(
                self.screen,
                self._hub_color(name),
                center,
                radius,
            )
            pygame.draw.circle(
                self.screen,
                (31, 41, 59),
                center,
                radius,
                width=5,
            )
            label = self.small_font.render(name, True, (31, 41, 59))
            label_rect = label.get_rect(center=(center[0], center[1] + 95))
            self.screen.blit(label, label_rect)

    def _position_for_drone(self, drone_id: int) -> tuple[int, int]:
        """Return the current interpolated center for a drone."""
        level = self._require_level()
        if self.turn_index >= len(self.turns) - 1:
            state = self.turns[-1]
            position = state.get(drone_id)
            if position is None:
                return (0, 0)
            hub = level.hubs[position]
            return self._world_to_screen(hub.loc_x, hub.loc_y)

        prev_state = self.turns[self.turn_index]
        next_state = self.turns[self.turn_index + 1]
        previous_position = prev_state.get(drone_id)
        next_position = next_state.get(drone_id)
        if previous_position is None and next_position is not None:
            previous_position = next_position
        if next_position is None and previous_position is not None:
            next_position = previous_position
        if previous_position is None or next_position is None:
            return (0, 0)

        if previous_position == next_position:
            hub = level.hubs[previous_position]
            return self._world_to_screen(hub.loc_x, hub.loc_y)

        prev_center = self._world_to_screen(
            level.hubs[previous_position].loc_x,
            level.hubs[previous_position].loc_y,
        )
        next_center = self._world_to_screen(
            level.hubs[next_position].loc_x,
            level.hubs[next_position].loc_y,
        )
        t = self.transition_progress
        x = int(prev_center[0] + (next_center[0] - prev_center[0]) * t)
        y = int(prev_center[1] + (next_center[1] - prev_center[1]) * t)
        return (x, y)

    def _draw_drones(self) -> None:
        """Draw drone markers from the current interpolated state."""
        level = self._require_level()
        palette = [
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
        for drone_id in range(1, level.nb_drones + 1):
            center = self._position_for_drone(drone_id)
            color = palette[(drone_id - 1) % len(palette)]
            pygame.draw.circle(self.screen, color, center, 28)
            pygame.draw.circle(self.screen, (17, 24, 39), center, 28, width=4)
            marker = self.small_font.render(f"{drone_id}", True, (17, 24, 39))
            label_rect = marker.get_rect(center=center)
            self.screen.blit(marker, label_rect)

    def _draw_turn_label(self) -> None:
        """Draw the current turn and status banner."""
        text = f"Turn {self.turn_index + 1}/{len(self.turns)}"
        label = self.font.render(text, True, (17, 24, 39))
        self.screen.blit(label, (24, 18))

    def _render_simulation(self) -> None:
        """Render one simulation frame."""
        self.screen.fill(self.background)
        self._draw_connections()
        self._draw_hubs()
        self._draw_drones()
        self._draw_turn_label()

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

    def run(self) -> None:
        """Run the application until closed."""
        while self.running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                elif event.type == pygame.KEYDOWN:
                    if self.mode == "menu":
                        self._handle_menu_key(event.key)
                    else:
                        self._handle_simulation_key(event.key)

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
