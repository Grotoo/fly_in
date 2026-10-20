import sys
from typing import Callable
from pydantic import ValidationError
from models import Connection, Hub, Level


class ParseError(Exception):
    pass


def format_validation_error(e: ValidationError) -> str:
    parts = []
    for err in e.errors():
        field = ".".join(str(p) for p in err["loc"])
        msg = err["msg"].removeprefix("Value error, ")
        parts.append(f"{field}: {msg}" if field else msg)
    return "; ".join(parts)


class Parser:
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
        self._reset()
        first = True
        with open(path, encoding="utf-8") as f:
            for number, raw in enumerate(f, start=1):
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
                except ValidationError as e:
                    raise ParseError(
                        f"Line {number}: {format_validation_error(e)}"
                    ) from e
                except ValueError as e:
                    raise ParseError(f"Line {number}: {e}") from e

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
        except ValidationError as e:
            raise ParseError(format_validation_error(e)) from e

    # ---------- helpers ----------
    def parse_metadata(self, text: str) -> dict[str, str]:
        meta: dict[str, str] = {}
        for pair in text.split():
            if "=" not in pair:
                raise ValueError(f"bad metadata entry '{pair}'")
            k, v = pair.split("=", 1)
            if not k or not v:
                raise ValueError(f"bad metadata entry '{pair}'")
            if k in meta:
                raise ValueError(f"duplicate metadata key '{k}'")
            meta[k] = v
        return meta

    def split_meta(self, value: str) -> tuple[str, dict[str, str]]:
        if "[" not in value:
            return value.strip(), {}
        main, meta = value.split("[", 1)
        meta = meta.strip()
        if not meta.endswith("]"):
            raise ValueError("missing closing ']'")
        return main.strip(), self.parse_metadata(meta[:-1])

    def add_hub(self, value: str, terminal: bool = False) -> str:
        main, meta = self.split_meta(value)
        parts = main.split()
        if len(parts) != 3:
            raise ValueError(f"expected '<name> <x> <y>', got '{main}'")
        name, x, y = parts
        if name in self.hubs:
            raise ValueError(f"duplicate hub '{name}'")
        if terminal:
            # max_drones is ignored on start/end hubs (per subject)
            meta.pop("max_drones", None)
        self.hubs[name] = Hub(hub_name=name, loc_x=x, loc_y=y, **meta)
        return name

    # ---------- handlers ----------
    def handle_nb_drones(self, value: str) -> None:
        if self.nb_drones is not None:
            raise ValueError("nb_drones defined twice")
        try:
            n = int(value)
        except ValueError:
            raise ValueError(
                f"nb_drones must be an integer, got '{value}'"
            ) from None
        if n < 1:
            raise ValueError("nb_drones must be a positive integer")
        self.nb_drones = n

    def handle_start(self, value: str) -> None:
        if self.start is not None:
            raise ValueError("start_hub defined twice")
        self.start = self.add_hub(value, terminal=True)

    def handle_hub(self, value: str) -> None:
        self.add_hub(value)

    def handle_end(self, value: str) -> None:
        if self.end is not None:
            raise ValueError("end_hub defined twice")
        self.end = self.add_hub(value, terminal=True)

    def handle_connection(self, value: str) -> None:
        main, meta = self.split_meta(value)
        parts = main.split("-")
        if len(parts) != 2:
            raise ValueError(f"expected '<a>-<b>', got '{main}'")
        a, b = parts[0].strip(), parts[1].strip()
        for name in (a, b):
            if name not in self.hubs:
                raise ValueError(
                    f"unknown zone '{name}' "
                    "(define zones before connections)"
                )
        key = frozenset((a, b))
        if key in self.connection_keys:
            raise ValueError(f"duplicate connection {a}-{b}")
        conn = Connection(point1=a, point2=b, **meta)
        self.connection_keys.add(key)
        self.connections.append(conn)


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print(f"Usage: python3 {sys.argv[0]} <level_file>")
        sys.exit(1)
    try:
        level = Parser().parse(sys.argv[1])
    except (OSError, ParseError) as e:
        print(f"Error: {e}")
        sys.exit(1)
    print(level.model_dump_json(indent=2))