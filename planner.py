"""Planners that turn a map into one schedule per drone.

A schedule is a list of moves. A move says "during turn N the drone
leaves zone A towards zone B"; waiting is simply the absence of a move.
The simulation engine (scheduler.py) replays and checks every schedule.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from collections import Counter, deque
from dataclasses import dataclass
from heapq import heappop, heappush

from models import Level, ZoneType
from pathfinding import Graph


class PlanError(ValueError):
    """Raised when a plan is impossible or breaks a movement rule."""


@dataclass(frozen=True, order=True)
class Move:
    """A drone leaves `source` during `turn` heading for `target`."""

    turn: int
    source: str
    target: str


Plan = list[list[Move]]


class Planner(ABC):
    """Common interface of all planners."""

    def __init__(self, level: Level, graph: Graph) -> None:
        self.level = level
        self.graph = graph

    def no_path_error(self) -> ValueError:
        """Error used when the end hub cannot be reached at all."""
        return ValueError(
            f"No valid path connects '{self.level.start}' "
            f"to '{self.level.end}' in this map."
        )

    @abstractmethod
    def plan(self) -> Plan:
        """Return one list of moves per drone."""


class FlowNetwork:
    """Min-cost max-flow over an edge list with residual twins."""

    INFINITY = 10**9

    def __init__(self) -> None:
        self.adj: list[list[int]] = []
        self.to: list[int] = []
        self.cap: list[int] = []
        self.cost: list[int] = []

    def add_node(self) -> int:
        """Create a node and return its id."""
        self.adj.append([])
        return len(self.adj) - 1

    def add_edge(
        self, source: int, target: int, cap: int, cost: int = 0
    ) -> int:
        """Add an edge plus its residual twin; return the edge id."""
        edge = len(self.to)
        self.adj[source].append(edge)
        self.to.append(target)
        self.cap.append(cap)
        self.cost.append(cost)
        self.adj[target].append(edge + 1)
        self.to.append(source)
        self.cap.append(0)
        self.cost.append(-cost)
        return edge

    def max_flow(self, source: int, sink: int, limit: int) -> int:
        """Push up to `limit` units from source to sink, cheapest first.

        Primal-dual scheme: SPFA gives the cheapest distance to every node,
        then a Dinic-style blocking flow saturates all cheapest paths at
        once. With all costs at zero this is plain Dinic.
        """
        total = 0
        while total < limit:
            dist = self._distances(source)
            if dist[sink] >= self.INFINITY:
                break
            while total < limit:
                level = self._levels(source, dist)
                if level[sink] < 0:
                    break
                total += self._blocking_flow(
                    source, sink, limit - total, dist, level
                )
        return total

    def _distances(self, source: int) -> list[int]:
        """Cheapest cost from source to every node of the residual graph."""
        dist = [self.INFINITY] * len(self.adj)
        queued = [False] * len(self.adj)
        dist[source] = 0
        queue = deque([source])
        while queue:
            node = queue.popleft()
            queued[node] = False
            for edge in self.adj[node]:
                target = self.to[edge]
                candidate = dist[node] + self.cost[edge]
                if self.cap[edge] > 0 and candidate < dist[target]:
                    dist[target] = candidate
                    if not queued[target]:
                        queued[target] = True
                        queue.append(target)
        return dist

    def _levels(self, source: int, dist: list[int]) -> list[int]:
        """BFS depth of every node using only cheapest-path edges."""
        level = [-1] * len(self.adj)
        level[source] = 0
        queue = deque([source])
        while queue:
            node = queue.popleft()
            for edge in self.adj[node]:
                target = self.to[edge]
                if (
                    self.cap[edge] > 0
                    and level[target] < 0
                    and dist[node] + self.cost[edge] == dist[target]
                ):
                    level[target] = level[node] + 1
                    queue.append(target)
        return level

    def _blocking_flow(
        self,
        source: int,
        sink: int,
        limit: int,
        dist: list[int],
        level: list[int],
    ) -> int:
        """Send flow along cheapest level-increasing paths until blocked."""
        pointer = [0] * len(self.adj)
        path: list[int] = []
        node = source
        pushed = 0
        while pushed < limit:
            if node == sink:
                amount = min(limit - pushed, min(self.cap[e] for e in path))
                for edge in path:
                    self.cap[edge] -= amount
                    self.cap[edge ^ 1] += amount
                pushed += amount
                path.clear()
                node = source
                continue
            edges = self.adj[node]
            while pointer[node] < len(edges):
                edge = edges[pointer[node]]
                target = self.to[edge]
                if (
                    self.cap[edge] > 0
                    and level[target] == level[node] + 1
                    and dist[node] + self.cost[edge] == dist[target]
                ):
                    path.append(edge)
                    node = target
                    break
                pointer[node] += 1
            else:
                if node == source:
                    break
                level[node] = -1
                edge = path.pop()
                node = self.to[edge ^ 1]
                pointer[node] += 1
        return pushed


@dataclass
class _TimeNetwork:
    """A flow network over (zone, turn) nodes plus decoding tables."""

    net: FlowNetwork
    source: int
    sink: int
    first: int
    moves: dict[int, Move]
    entries: dict[int, tuple[str, int]]
    exits: dict[int, str]


class TimeExpandedPlanner(Planner):
    """Optimal planner based on a flow over time.

    Every zone is copied once per turn. Waiting, moving and the zone and
    link capacities become edges of a flow network, so a feasible flow
    of nb_drones units is exactly a conflict-free schedule. The smallest
    horizon with such a flow is the minimum number of turns.
    """

    def __init__(self, level: Level, graph: Graph) -> None:
        super().__init__(level, graph)
        self.start_times = graph.travel_times(level.start)
        self.end_times = graph.travel_times(level.end, reverse=True)

    def plan(self) -> Plan:
        """Find the fewest turns and extract one route per drone."""
        graph, level = self.graph, self.level
        graph.shortest_path(level.start, level.end)
        drones = level.nb_drones
        lower = self.end_times[level.start]
        per_turn = self._throughput()
        # No schedule can beat this: the first drone needs `lower` turns
        # and afterwards at most `per_turn` drones can arrive per turn.
        impossible = lower + -(-drones // per_turn) - 2
        guaranteed = lower + drones - 1
        probe, step = impossible + 1, 1
        while probe < guaranteed and not self._feasible(probe):
            impossible, probe = probe, min(probe + step, guaranteed)
            step *= 2
        while probe - impossible > 1:
            middle = (impossible + probe) // 2
            if self._feasible(middle):
                probe = middle
            else:
                impossible = middle
        built = self._build(probe, weighted=True)
        if built.net.max_flow(built.source, built.sink, drones) < drones:
            raise PlanError("flow planner found no complete schedule")
        routes = [self._walk(built) for _ in range(drones)]
        routes.sort()
        return routes

    def _throughput(self) -> int:
        """Most drones that can reach the end hub in a single turn."""
        level = self.level
        net = FlowNetwork()
        source, sink = net.add_node(), net.add_node()
        inn = {zone: net.add_node() for zone in level.hubs}
        out = {zone: net.add_node() for zone in level.hubs}
        for zone in level.hubs:
            capacity = level.zone_capacity(zone) or level.nb_drones
            net.add_edge(inn[zone], out[zone], capacity)
        net.add_edge(source, inn[level.start], level.nb_drones)
        net.add_edge(out[level.end], sink, level.nb_drones)
        for connection in level.connections:
            a, b = connection.point1, connection.point2
            if self.graph.is_blocked(a) or self.graph.is_blocked(b):
                continue
            capacity = connection.max_link_capacity
            for source_zone, target_zone in ((a, b), (b, a)):
                if source_zone != level.end:
                    net.add_edge(out[source_zone], inn[target_zone], capacity)
        return max(1, net.max_flow(source, sink, level.nb_drones))

    def _feasible(self, horizon: int) -> bool:
        """Can all drones arrive within `horizon` turns?"""
        built = self._build(horizon, weighted=False)
        drones = self.level.nb_drones
        return built.net.max_flow(built.source, built.sink, drones) >= drones

    def _move_cost(self, zone: str) -> int:
        """Tie-break cost: prefer priority zones and fewer slow moves."""
        zone_type = self.level.hubs[zone].zone_type
        if zone_type == ZoneType.PRIORITY:
            return 1
        if zone_type == ZoneType.RESTRICTED:
            return 3
        return 2

    def _build(self, horizon: int, weighted: bool) -> _TimeNetwork:
        """Create the time-expanded network for a given horizon."""
        level = self.level
        drones = level.nb_drones
        net = FlowNetwork()
        built = _TimeNetwork(
            net, net.add_node(), net.add_node(), 0, {}, {}, {}
        )
        inn: dict[tuple[str, int], int] = {}
        out: dict[tuple[str, int], int] = {}

        for zone, hub in level.hubs.items():
            earliest = self.start_times.get(zone)
            to_end = self.end_times.get(zone)
            if (
                zone == level.end
                or hub.zone_type == ZoneType.BLOCKED
                or earliest is None
                or to_end is None
            ):
                continue
            capacity = drones if zone == level.start else hub.max_drones
            for turn in range(earliest, horizon - to_end + 1):
                node_in, node_out = net.add_node(), net.add_node()
                inn[zone, turn], out[zone, turn] = node_in, node_out
                net.add_edge(node_in, node_out, capacity)
        for (zone, turn), node_out in out.items():
            if (zone, turn + 1) in inn:
                net.add_edge(node_out, inn[zone, turn + 1], drones)

        built.first = inn[level.start, 0]
        net.add_edge(built.source, built.first, drones)
        for connection in level.connections:
            first, second = connection.point1, connection.point2
            ends = (first, second)
            if any(self.graph.is_blocked(zone) for zone in ends):
                continue
            shared = all(self.graph.edge_cost(zone) == 1 for zone in ends)
            for turn in range(1, horizon + 1):
                if shared:
                    self._add_shared(
                        built, inn, out, connection.max_link_capacity,
                        ends, turn, weighted,
                    )
                    continue
                for source, target in ((first, second), (second, first)):
                    self._add_directed(
                        built, inn, out, connection.max_link_capacity,
                        (source, target), turn, horizon, weighted,
                    )
        return built

    def _add_directed(
        self,
        built: _TimeNetwork,
        inn: dict[tuple[str, int], int],
        out: dict[tuple[str, int], int],
        link_capacity: int,
        way: tuple[str, str],
        turn: int,
        horizon: int,
        weighted: bool,
    ) -> None:
        """Add one move edge (used when travel times differ by direction)."""
        source, target = way
        if (source, turn - 1) not in out:
            return
        arrival = turn - 1 + self.graph.edge_cost(target)
        if target == self.level.end:
            if arrival > horizon:
                return
            node = built.sink
        elif (target, arrival) in inn:
            node = inn[target, arrival]
        else:
            return
        cost = self._move_cost(target) if weighted else 0
        edge = built.net.add_edge(
            out[source, turn - 1], node, link_capacity, cost
        )
        built.moves[edge] = Move(turn, source, target)

    def _add_shared(
        self,
        built: _TimeNetwork,
        inn: dict[tuple[str, int], int],
        out: dict[tuple[str, int], int],
        link_capacity: int,
        ends: tuple[str, str],
        turn: int,
        weighted: bool,
    ) -> None:
        """Add a link node so both directions share one capacity."""
        net = built.net
        entries = [z for z in ends if (z, turn - 1) in out]
        exits: list[tuple[str, int]] = []
        for zone in ends:
            if zone == self.level.end:
                exits.append((zone, built.sink))
            elif (zone, turn) in inn:
                exits.append((zone, inn[zone, turn]))
        if not any(a != b for a in entries for b, _ in exits):
            return
        link_in, link_out = net.add_node(), net.add_node()
        net.add_edge(link_in, link_out, link_capacity)
        for zone in entries:
            edge = net.add_edge(
                out[zone, turn - 1], link_in, self.level.nb_drones
            )
            built.entries[edge] = (zone, turn)
        for zone, node in exits:
            cost = self._move_cost(zone) if weighted else 0
            edge = net.add_edge(
                link_out, node, self.level.nb_drones, cost
            )
            built.exits[edge] = zone

    def _walk(self, built: _TimeNetwork) -> list[Move]:
        """Follow one unit of flow from the start to the sink."""
        net = built.net
        moves: list[Move] = []
        origin: tuple[str, int] | None = None
        node = built.first
        while node != built.sink:
            edge = self._next_edge(built, node, origin)
            if edge in built.moves:
                moves.append(built.moves[edge])
            elif edge in built.entries:
                origin = built.entries[edge]
            elif edge in built.exits and origin is not None:
                if built.exits[edge] != origin[0]:
                    moves.append(
                        Move(origin[1], origin[0], built.exits[edge])
                    )
                origin = None
            node = net.to[edge]
        return moves

    def _next_edge(
        self,
        built: _TimeNetwork,
        node: int,
        origin: tuple[str, int] | None,
    ) -> int:
        """Pick and use up one unit of flow leaving `node`."""
        net = built.net
        chosen = -1
        for edge in net.adj[node]:
            if edge % 2 or net.cap[edge ^ 1] <= 0:
                continue
            if origin is not None and built.exits.get(edge) == origin[0]:
                chosen = edge
                break
            if chosen < 0:
                chosen = edge
        if chosen < 0:
            raise PlanError("flow decomposition failed")
        net.cap[chosen ^ 1] -= 1
        return chosen


class ReservationPlanner(Planner):
    """Fallback planner: drones are routed one after another.

    Each drone takes the earliest free path through (zone, turn) slots
    that earlier drones have not reserved yet. It always yields a valid
    plan but is not guaranteed to use the fewest turns.
    """

    def __init__(self, level: Level, graph: Graph) -> None:
        super().__init__(level, graph)
        self.zone_load: Counter[tuple[str, int]] = Counter()
        self.link_load: Counter[tuple[frozenset[str], int]] = Counter()

    def plan(self) -> Plan:
        """Route every drone through the slots left by the previous ones."""
        level = self.level
        times = self.graph.travel_times(level.end, reverse=True)
        if level.start not in times:
            raise self.no_path_error()
        shortest = times[level.start]
        latest = 0
        routes: Plan = []
        for _ in range(level.nb_drones):
            moves = self._route_one(latest + shortest)
            latest = max(latest, self._reserve(moves))
            routes.append(moves)
        return routes

    def _route_one(self, limit: int) -> list[Move]:
        """Earliest-arrival search over (zone, turn) states."""
        level, graph = self.level, self.graph
        counter = 0
        queue: list[tuple[int, int, str]] = [(0, counter, level.start)]
        parent: dict[tuple[str, int], tuple[str, int] | None] = {
            (level.start, 0): None
        }
        done: set[tuple[str, int]] = set()
        while queue:
            time, _, zone = heappop(queue)
            if (zone, time) in done:
                continue
            done.add((zone, time))
            if zone == level.end:
                return self._rebuild(parent, (zone, time))
            options: list[tuple[str, int]] = []
            capacity = level.zone_capacity(zone)
            if time < limit and (
                capacity is None or self.zone_load[zone, time + 1] < capacity
            ):
                options.append((zone, time + 1))
            for connection in graph.adjacency.get(zone, []):
                target = graph.other_end(zone, connection)
                arrival = time + graph.edge_cost(target)
                key = frozenset((zone, target))
                target_cap = level.zone_capacity(target)
                if (
                    graph.is_blocked(target)
                    or arrival > limit
                    or self.link_load[key, time + 1]
                    >= connection.max_link_capacity
                    or (
                        target_cap is not None
                        and self.zone_load[target, arrival] >= target_cap
                    )
                ):
                    continue
                options.append((target, arrival))
            for target, arrival in options:
                if (target, arrival) in parent:
                    continue
                parent[target, arrival] = (zone, time)
                counter += 1
                heappush(queue, (arrival, counter, target))
        raise PlanError("reservation planner found no free route")

    def _rebuild(
        self,
        parent: dict[tuple[str, int], tuple[str, int] | None],
        state: tuple[str, int],
    ) -> list[Move]:
        """Turn the parent links of the search into a list of moves."""
        moves: list[Move] = []
        current: tuple[str, int] | None = state
        while current is not None:
            previous = parent[current]
            if previous is not None and previous[0] != current[0]:
                moves.append(Move(previous[1] + 1, previous[0], current[0]))
            current = previous
        moves.reverse()
        return moves

    def _reserve(self, moves: list[Move]) -> int:
        """Book the zones and links used by a route; return its end turn."""
        zone, since = self.level.start, 0
        for move in moves:
            for time in range(since, move.turn):
                self.zone_load[zone, time] += 1
            key = frozenset((move.source, move.target))
            self.link_load[key, move.turn] += 1
            zone = move.target
            since = move.turn - 1 + self.graph.edge_cost(zone)
        return since
