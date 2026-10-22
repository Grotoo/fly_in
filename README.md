*This project has been created as part of the 42 curriculum by Grotoo.*

# Fly-In: Drone Fleet Routing Simulator

## Description

Fly-In is a Python 3 project that parses drone network maps, computes valid routes,
and simulates the movement of multiple drones from a start hub to an end hub while
respecting zone capacities, connection capacities, and movement costs. The simulator
uses a fully object-oriented structure and prevents collisions, deadlocks, and
invalid transitions through explicit turn-based scheduling.

The project is designed around the rules from the assignment: blocked zones are
forbidden, restricted zones cost two turns, priority zones are preferred, and all
routing decisions are evaluated with capacity-aware movement rules.

## Instructions

### Dependencies

```bash
make install
```

### Run the simulator

```bash
make run MAP=maps/easy/01_linear_path.txt
```

Force terminal colors when your terminal integration disables TTY color auto-detect:

```bash
python3 main.py --color maps/medium/01_dead_end_trap.txt
```

### Run the GUI map selector

```bash
python3 main.py --gui
```

### Debug mode

```bash
make debug MAP=maps/easy/01_linear_path.txt
```

### Linting

```bash
make lint
```

### Strict linting (optional)

```bash
make lint-strict
```

### Clean temporary files

```bash
make clean
```

## Algorithm and implementation strategy

The simulator is structured in three layers:

1. Parsing: the map file is read line by line and transformed into typed objects.
2. Graph construction: zones and connections are represented as an object-oriented
   graph with no external graph library.
3. Simulation: drones follow a shortest valid route, then advance through the map
   turn by turn while tracking occupancy constraints.

The route engine uses a Dijkstra-style search over the zone graph. Each time a
zone is entered, its cost is computed from its type:

- normal: 1 turn
- priority: 1 turn
- restricted: 2 turns
- blocked: forbidden

The scheduler keeps a per-turn occupancy count for every hub and connection. A
move is accepted only if the target zone still has available capacity and the
connection itself is not saturated. This keeps the simulation valid even when many
drones share a narrow corridor or converge on a single choke point.

The visual output is available in two modes:

- terminal mode (default): each turn prints movement lines in the assignment
  format, with ANSI colors based on hub metadata/zone type.
- Pygame mode (`--gui`): animated simulation with a map-selection start screen.

## Visual representation

Terminal mode prints a step-by-step log of movements, one turn per line, and
applies colors directly to movement tokens so hub types/colors are visible while
keeping the required output structure. For example, a normal turn looks like:

```text
D1-waypoint1 D2-waypoint1
D1-waypoint2 D2-waypoint2
D1-goal D2-goal
```

The optional Pygame mode adds animated movement and map browsing. After a map
finishes, the app returns to the map-selection menu so multiple maps can be run
in one session.

## Example input and output

### Example map

```text
nb_drones: 2
start_hub: start 0 0 [color=green]
hub: waypoint1 1 0 [color=blue]
hub: waypoint2 2 0 [color=blue]
end_hub: goal 3 0 [color=red]

connection: start-waypoint1
connection: waypoint1-waypoint2
connection: waypoint2-goal
```

### Example output

```text
D1-waypoint1
D2-waypoint1
D1-waypoint2
D2-waypoint2
D1-goal
D2-goal
```

## Resources

- Python documentation: https://docs.python.org/3/
- Pydantic documentation: https://docs.pydantic.dev/
- Flake8: https://flake8.pycqa.org/
- MyPy: https://mypy.readthedocs.io/

AI assistance was used to structure the object-oriented architecture, validate the
parser logic, and verify the turn-based scheduling approach before final testing.

## Notes

This project is intentionally built without third-party graph libraries to stay in
line with the assignment constraints. All logic is implemented directly in Python
using object-oriented classes and explicit algorithms.
