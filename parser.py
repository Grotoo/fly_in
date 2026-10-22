import sys
from typing import Callable

from pydantic import ValidationError

from models import Connection, Hub, Level, ZoneType


class ParseError(Exception):
    """Raised when the input map file cannot be parsed."""


def format_validation_error(error: ValidationError) -> str:
    """Convert a pydantic validation error into readable text."""
    parts: list[str] = []
    for err in error.errors():
        field = ".".join(str(part) for part in err["loc"])
        message = err["msg"].removeprefix("Value error, ")
        parts.append(f"{field}: {message}" if field else message)
    return "; ".join(parts)


class Parser:
    """Parses a map file into a Level instance."""

    def __init__(self) -> None:
        self._reset()
        self.handlers: dict[str, Callable[[str], None]] = {
            "nb_drones": self.handle_nb_drones,
            "start_hub": self.handle_start,
            "hub": self.handle_hub,
            "end_hub": self.handle_end,
            "connection": self.handle_connection,
        }

    def _reset(self) -> None:
        self.nb_drones: int | None = None
        self.hubs: dict[str, Hub] = {}
        self.connections: list[Connection] = []
        self.connection_keys: set[frozenset[str]] = set()
        self.start: str | None = None
        self.end: str | None = None

    def parse(self, path: str) -> Level:
        """Parse a level file from disk."""
        self._reset()
        first = True
        with open(path, encoding="utf-8") as handle:
            for number, raw in enumerate(handle, start=1):
                line = raw.strip()
                if not line or line.startswith("#"):
                    continue
                if ":" not in line:
                    raise ParseError(f"Line {number}: missing ':'")
                key, value = line.split(":", 1)
                key = key.strip()
                handler = self.handlers.get(key)
                if handler is None:
                    raise ParseError(f"Line {number}: unknown key '{key}'")
                if first and key != "nb_drones":
                    raise ParseError(
                        f"Line {number}: first line must define nb_drones"
                    )
                first = False
                try:
                    handler(value.strip())
                except ValidationError as error:
                    raise ParseError(
                        f"Line {number}: {format_validation_error(error)}"
                    ) from error
                except ValueError as error:
                    raise ParseError(f"Line {number}: {error}") from error

        if self.nb_drones is None:
            raise ParseError("missing 'nb_drones'")
        if self.start is None:
            raise ParseError("missing 'start_hub'")
        if self.end is None:
            raise ParseError("missing 'end_hub'")
        try:
            return Level(
                nb_drones=self.nb_drones,
                start=self.start,
                end=self.end,
                hubs=self.hubs,
                connections=self.connections,
            )
        except ValidationError as error:
            raise ParseError(format_validation_error(error)) from error

    def parse_metadata(self, text: str) -> dict[str, str]:
        """Parse a metadata block like [zone=priority color=red]."""
        meta: dict[str, str] = {}
        for pair in text.split():
            if "=" not in pair:
                raise ValueError(f"bad metadata entry '{pair}'")
            key, value = pair.split("=", 1)
            if not key or not value:
                raise ValueError(f"bad metadata entry '{pair}'")
            if key in meta:
                raise ValueError(f"duplicate metadata key '{key}'")
            meta[key] = value
        return meta

    def split_meta(self, value: str) -> tuple[str, dict[str, str]]:
        """Split a value into its raw content and metadata."""
        if "[" not in value:
            return value.strip(), {}
        main, meta = value.split("[", 1)
        metadata = meta.strip()
        if not metadata.endswith("]"):
            raise ValueError("missing closing ']'")
        return main.strip(), self.parse_metadata(metadata[:-1])

    def add_hub(self, value: str, terminal: bool = False) -> str:
        """Create a hub from a raw line fragment."""
        main, meta = self.split_meta(value)
        parts = main.split()
        if len(parts) != 3:
            raise ValueError(f"expected '<name> <x> <y>', got '{main}'")
        name, x, y = parts
        if name in self.hubs:
            raise ValueError(f"duplicate hub '{name}'")
        if terminal:
            meta.pop("max_drones", None)

        zone_value = meta.pop("zone", "normal")
        color = meta.pop("color", None)
        max_drones = int(meta.pop("max_drones", 1))
        if meta:
            unknown = ", ".join(sorted(meta))
            raise ValueError(f"unknown hub metadata: {unknown}")

        self.hubs[name] = Hub(
            hub_name=name,
            loc_x=int(x),
            loc_y=int(y),
            zone=ZoneType(zone_value),
            color=color,
            max_drones=max_drones,
        )
        return name

    def handle_nb_drones(self, value: str) -> None:
        """Handle the nb_drones header."""
        if self.nb_drones is not None:
            raise ValueError("nb_drones defined twice")
        try:
            count = int(value)
        except ValueError:
            raise ValueError(f"nb_drones must be an integer, got '{value}'")
        if count < 1:
            raise ValueError("nb_drones must be a positive integer")
        self.nb_drones = count

    def handle_start(self, value: str) -> None:
        """Create the unique start hub."""
        if self.start is not None:
            raise ValueError("start_hub defined twice")
        self.start = self.add_hub(value, terminal=True)

    def handle_hub(self, value: str) -> None:
        """Create a regular hub."""
        self.add_hub(value)

    def handle_end(self, value: str) -> None:
        """Create the unique end hub."""
        if self.end is not None:
            raise ValueError("end_hub defined twice")
        self.end = self.add_hub(value, terminal=True)

    def handle_connection(self, value: str) -> None:
        """Create a connection between two previously-defined hubs."""
        main, meta = self.split_meta(value)
        parts = main.split("-")
        if len(parts) != 2:
            raise ValueError(f"expected '<a>-<b>', got '{main}'")
        first, second = parts[0].strip(), parts[1].strip()
        for name in (first, second):
            if name not in self.hubs:
                raise ValueError(
                    f"unknown zone '{name}' (define zones before connections)"
                )
        key = frozenset((first, second))
        if key in self.connection_keys:
            raise ValueError(f"duplicate connection {first}-{second}")
        max_link_capacity = int(meta.pop("max_link_capacity", 1))
        if meta:
            unknown = ", ".join(sorted(meta))
            raise ValueError(f"unknown connection metadata: {unknown}")
        connection = Connection(
            point1=first,
            point2=second,
            max_link_capacity=max_link_capacity,
        )
        self.connection_keys.add(key)
        self.connections.append(connection)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: python3 {sys.argv[0]} <level_file>")
        sys.exit(1)
    try:
        level = Parser().parse(sys.argv[1])
    except (OSError, ParseError) as error:
        print(f"Error: {error}")
        sys.exit(1)
    print(level.model_dump_json(indent=2))
