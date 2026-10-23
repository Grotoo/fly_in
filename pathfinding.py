from __future__ import annotations

from collections import defaultdict
from heapq import heappop, heappush

from models import Connection, Level, ZoneType


class Graph:
    """Static graph used for pathfinding and route lookup."""

    def __init__(self, level: Level) -> None:
        self.level = level
        self.adjacency: dict[str, list[Connection]] = defaultdict(list)
        for connection in level.connections:
            self.adjacency[connection.point1].append(connection)
            self.adjacency[connection.point2].append(connection)

    def other_end(self, current: str, connection: Connection) -> str:
        """Return the neighboring zone reached from current."""
        if connection.point1 == current:
            return connection.point2
        if connection.point2 == current:
            return connection.point1
        raise ValueError(
            f"Connection {connection.point1}-{connection.point2} is not "
            f"adjacent to {current}."
        )

    def connection_between(self, start: str, end: str) -> Connection | None:
        """Fetch the direct connection between two neighboring hubs."""
        for connection in self.adjacency.get(start, []):
            if self.other_end(start, connection) == end:
                return connection
        return None

    def edge_cost(self, zone_name: str) -> int:
        """Movement cost of entering the given zone."""
        zone = self.level.hubs[zone_name]
        if zone.zone_type == ZoneType.BLOCKED:
            return 10**9
        if zone.zone_type == ZoneType.RESTRICTED:
            return 2
        return 1

    def path_score(self, path: list[str]) -> tuple[int, int, int]:
        """Score a path by travel cost, priority preference, and length."""
        cost = sum(self.edge_cost(name) for name in path[1:])
        priority_zones = sum(
            self.level.hubs[name].zone_type == ZoneType.PRIORITY
            for name in path[1:]
        )
        return cost, -priority_zones, len(path)

    def all_paths(
        self, start: str, end: str, limit: int = 256
    ) -> list[list[str]]:
        """Return bounded simple paths ordered by movement cost."""
        paths: list[list[str]] = []

        def visit(current: str, path: list[str]) -> None:
            if len(paths) >= limit:
                return
            if current == end:
                paths.append(path.copy())
                return
            for connection in self.adjacency.get(current, []):
                neighbor = self.other_end(current, connection)
                if neighbor in path:
                    continue
                if self.level.hubs[neighbor].zone_type == ZoneType.BLOCKED:
                    continue
                visit(neighbor, path + [neighbor])

        visit(start, [start])
        paths.sort(key=self.path_score)
        return paths

    def shortest_path(self, start: str, end: str) -> list[str]:
        """Compute a shortest route from start to end."""
        if start == end:
            return [start]

        distances: dict[str, int] = {start: 0}
        previous: dict[str, str | None] = {start: None}
        queue: list[tuple[int, str]] = [(0, start)]

        while queue:
            cost, current = heappop(queue)
            if cost > distances.get(current, 10**9):
                continue
            if current == end:
                break
            for connection in self.adjacency.get(current, []):
                neighbor = self.other_end(current, connection)
                if self.level.hubs[neighbor].zone_type == ZoneType.BLOCKED:
                    continue
                candidate = cost + self.edge_cost(neighbor)
                if candidate < distances.get(neighbor, 10**9):
                    distances[neighbor] = candidate
                    previous[neighbor] = current
                    heappush(queue, (candidate, neighbor))

        if end not in distances:
            raise ValueError(
                f"No valid path connects '{start}' to '{end}' in this map."
            )

        path: list[str] = []
        cursor: str | None = end
        while cursor is not None:
            path.append(cursor)
            cursor = previous[cursor]
        path.reverse()
        return path
