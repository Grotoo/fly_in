from __future__ import annotations

from collections import Counter
from dataclasses import dataclass

from models import Level, ZoneType
from pathfinding import Graph
from planner import (
    Move,
    Plan,
    PlanError,
    Planner,
    ReservationPlanner,
    TimeExpandedPlanner,
)


@dataclass
class Drone:
    """Single drone state during the simulation."""

    drone_id: int
    position: str
    moves: list[Move]
    next_move: int = 0
    in_transit: bool = False
    transit_target: str | None = None

    def due_move(self, turn: int) -> Move | None:
        """Return the move this drone has to make during `turn`."""
        if self.next_move >= len(self.moves):
            return None
        move = self.moves[self.next_move]
        return move if move.turn == turn else None


class Simulation:
    """Turn-based engine that replays a plan and enforces every rule.

    A planner proposes a schedule; the engine only accepts it when each
    turn respects zone capacity, link capacity and the two-turn rule for
    restricted zones. The first planner whose plan passes is used.
    """

    planners: tuple[type[Planner], ...] = (
        TimeExpandedPlanner,
        ReservationPlanner,
    )

    def __init__(self, level: Level) -> None:
        self.level = level
        self.graph = Graph(level)
        inner = [
            name for name in level.hubs if name not in (level.start, level.end)
        ]
        self.status_zone = inner[0] if inner else None
        self.status_connection = (
            level.connections[0] if level.connections else None
        )
        self.last_status = ""

    def _solve(
        self, show_usage: bool = False
    ) -> tuple[list[dict[int, str]], list[str]]:
        """Plan and execute; fall back to the next planner on rule errors."""
        error = PlanError("no planner available")
        for planner_class in self.planners:
            plan = planner_class(self.level, self.graph).plan()
            try:
                return self._execute(plan, show_usage)
            except PlanError as caught:
                error = caught
        raise error

    def _execute(
        self, plan: Plan, show_usage: bool
    ) -> tuple[list[dict[int, str]], list[str]]:
        """Replay a plan turn by turn; return the states and output lines."""
        if len(plan) != self.level.nb_drones:
            raise PlanError("plan does not cover every drone")
        drones = [
            Drone(drone_id=index, position=self.level.start, moves=moves)
            for index, moves in enumerate(plan, start=1)
        ]
        states = [self._snapshot(drones)]
        lines: list[str] = []
        last_turn = 1 + max(
            (move.turn for moves in plan for move in moves), default=0
        )
        for turn in range(1, last_turn + 1):
            events, usage = self._advance(drones, turn)
            lines.append(" ".join(events))
            if show_usage:
                status = self._status(drones, usage)
                if status:
                    lines.append(status)
            states.append(self._snapshot(drones))
            if self._all_delivered(drones):
                return states, lines
        raise PlanError("plan leaves drones undelivered")

    def _advance(
        self, drones: list[Drone], turn: int
    ) -> tuple[list[str], Counter[frozenset[str]]]:
        """Play one turn: arrivals, departures, then the capacity check."""
        events: list[str] = []
        usage: Counter[frozenset[str]] = Counter()
        for drone in drones:
            if drone.in_transit and drone.transit_target is not None:
                drone.position = drone.transit_target
                drone.in_transit = False
                drone.transit_target = None
                events.append(f"D{drone.drone_id}-{drone.position}")
                continue
            move = drone.due_move(turn)
            if move is not None:
                events.append(self._depart(drone, move, usage))
        self._check_zones(drones)
        return events, usage

    def _depart(
        self, drone: Drone, move: Move, usage: Counter[frozenset[str]]
    ) -> str:
        """Start a move and return its output token."""
        if drone.position != move.source:
            raise PlanError(
                f"D{drone.drone_id} is not in '{move.source}' "
                f"during turn {move.turn}"
            )
        connection = self.graph.connection_between(move.source, move.target)
        if connection is None:
            raise PlanError(
                f"no connection {move.source}-{move.target}"
            )
        target = self.level.hubs[move.target]
        if target.zone_type == ZoneType.BLOCKED:
            raise PlanError(f"zone '{move.target}' is blocked")
        key = frozenset((move.source, move.target))
        usage[key] += 1
        if usage[key] > connection.max_link_capacity:
            raise PlanError(
                f"connection {move.source}-{move.target} over capacity "
                f"in turn {move.turn}"
            )
        drone.next_move += 1
        if target.zone_type == ZoneType.RESTRICTED:
            drone.in_transit = True
            drone.transit_target = move.target
            return f"D{drone.drone_id}-{move.source}-{move.target}"
        drone.position = move.target
        return f"D{drone.drone_id}-{move.target}"

    def _check_zones(self, drones: list[Drone]) -> None:
        """Make sure no zone holds more drones than its capacity."""
        occupancy = Counter(
            drone.position
            for drone in drones
            if not drone.in_transit and drone.position != self.level.end
        )
        for name, count in occupancy.items():
            capacity = self.level.zone_capacity(name)
            if capacity is not None and count > capacity:
                raise PlanError(f"zone '{name}' is over capacity")

    def _all_delivered(self, drones: list[Drone]) -> bool:
        """Tell whether every drone has reached the end hub."""
        return all(drone.position == self.level.end for drone in drones)

    def _snapshot(self, drones: list[Drone]) -> dict[int, str]:
        """Zone of every drone (a drone in flight stays at its origin)."""
        return {drone.drone_id: drone.position for drone in drones}

    def _status(
        self, drones: list[Drone], usage: Counter[frozenset[str]]
    ) -> str:
        """Describe the usage of one sample zone and one sample link."""
        if self.status_zone is None or self.status_connection is None:
            return ""
        zone = self.level.hubs[self.status_zone]
        connection = self.status_connection
        key = frozenset((connection.point1, connection.point2))
        zone_used = sum(
            drone.position == self.status_zone and not drone.in_transit
            for drone in drones
        )
        self.last_status = (
            f"Zone {self.status_zone}: {zone_used}/{zone.max_drones} drones, "
            f"Connection {connection.point1}-{connection.point2}: "
            f"{usage[key]}/{connection.max_link_capacity} capacity used"
        )
        return self.last_status

    def run_with_states(self) -> list[dict[int, str]]:
        """Return drone positions after each turn for visualization."""
        return self._solve()[0]

    def run(self, show_usage: bool = False) -> list[str]:
        """Execute the simulation and return per-turn output lines."""
        return self._solve(show_usage)[1]
