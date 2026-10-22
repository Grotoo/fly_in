from __future__ import annotations

from collections import Counter
from dataclasses import dataclass

from models import Level, ZoneType
from pathfinding import Graph


@dataclass
class Drone:
    """Single drone state during the simulation."""

    drone_id: int
    position: str
    route: list[str]
    route_index: int = 0
    in_transit: bool = False
    transit_target: str | None = None
    transit_arrival_turn: int | None = None

    def next_hop(self) -> str | None:
        """Return the next zone on the drone route, if any."""
        if self.route_index + 1 >= len(self.route):
            return None
        return self.route[self.route_index + 1]


class Simulation:
    """Turn-based scheduler that respects capacities and movement rules."""

    def __init__(self, level: Level) -> None:
        self.level = level
        self.graph = Graph(level)

    def _route_for_drone(self) -> list[str]:
        """Compute the shortest valid route for a drone."""
        return self.graph.shortest_path(self.level.start, self.level.end)

    def _route_index(self, route: list[str], position: str) -> int:
        """Find the index of position inside route."""
        try:
            return route.index(position)
        except ValueError:
            return 0

    def _turn_events(self, drones: list[Drone], turn: int) -> list[str]:
        """Advance the current turn and return the movement events."""
        events: list[str] = []

        for drone in drones:
            if drone.in_transit and drone.transit_arrival_turn == turn:
                assert drone.transit_target is not None
                drone.position = drone.transit_target
                drone.in_transit = False
                drone.transit_target = None
                drone.transit_arrival_turn = None
                drone.route_index = self._route_index(
                    drone.route, drone.position
                )
                events.append(f"D{drone.drone_id}-{drone.position}")

        occupancy: Counter[str] = Counter()
        for drone in drones:
            if not drone.in_transit and drone.position != self.level.end:
                occupancy[drone.position] += 1

        source_departures: Counter[str] = Counter()
        target_arrivals: Counter[str] = Counter()
        connection_usage: Counter[str] = Counter()
        accepted: list[tuple[Drone, str, str]] = []

        for drone in sorted(drones, key=lambda item: item.drone_id):
            if drone.in_transit or drone.position == self.level.end:
                continue
            next_hub = drone.next_hop()
            if next_hub is None:
                continue
            connection = self.graph.connection_between(
                drone.position, next_hub
            )
            if connection is None:
                continue
            key = f"{drone.position}-{next_hub}"

            if connection_usage[key] >= connection.max_link_capacity:
                continue

            target_capacity = self.level.zone_capacity(next_hub)
            if target_capacity is not None:
                projected = (
                    occupancy[next_hub]
                    - source_departures[next_hub]
                    + target_arrivals[next_hub]
                    + 1
                )
                if projected > target_capacity:
                    continue

            if self.level.hubs[next_hub].zone_type == ZoneType.RESTRICTED:
                source_departures[drone.position] += 1
                target_arrivals[next_hub] += 1
                connection_usage[key] += 1
                drone.in_transit = True
                drone.transit_target = next_hub
                drone.transit_arrival_turn = turn + 1
                accepted.append((drone, next_hub, key))
                continue

            source_departures[drone.position] += 1
            target_arrivals[next_hub] += 1
            connection_usage[key] += 1
            accepted.append((drone, next_hub, key))

        for drone, next_hub, key in accepted:
            if drone.in_transit:
                events.append(f"D{drone.drone_id}-{key}")
                continue
            drone.position = next_hub
            drone.route_index = self._route_index(drone.route, next_hub)
            events.append(f"D{drone.drone_id}-{next_hub}")

        return events

    def run_with_states(self) -> list[dict[int, str]]:
        """Return drone positions after each turn for visualization."""
        route = self._route_for_drone()
        drones = [
            Drone(
                drone_id=index,
                position=self.level.start,
                route=route.copy(),
            )
            for index in range(1, self.level.nb_drones + 1)
        ]

        states: list[dict[int, str]] = []
        turn = 0
        while True:
            turn += 1
            self._turn_events(drones, turn)
            states.append(
                {drone.drone_id: drone.position for drone in drones}
            )
            if all(drone.position == self.level.end for drone in drones):
                break
            if turn > 10000:
                raise RuntimeError("Simulation did not converge.")
        return states

    def run(self) -> list[str]:
        """Execute the simulation and return per-turn output lines."""
        route = self._route_for_drone()
        drones = [
            Drone(
                drone_id=index,
                position=self.level.start,
                route=route.copy(),
            )
            for index in range(1, self.level.nb_drones + 1)
        ]

        turns: list[str] = []
        turn = 0
        while True:
            turn += 1
            events = self._turn_events(drones, turn)
            if events:
                turns.append(" ".join(events))
            if all(drone.position == self.level.end for drone in drones):
                break
            if turn > 10000:
                raise RuntimeError("Simulation did not converge.")
        return turns
