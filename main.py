from __future__ import annotations

import sys

from models import Level, ZoneType
from parser import ParseError, Parser
from scheduler import Simulation


def _ansi_for_hub(level: Level, hub_name: str) -> str:
    """Return ANSI color code for a hub based on map metadata."""
    hub = level.hubs[hub_name]
    if hub.color is not None:
        palette = {
            "black": "\033[30m",
            "red": "\033[31m",
            "green": "\033[32m",
            "yellow": "\033[33m",
            "blue": "\033[34m",
            "magenta": "\033[35m",
            "purple": "\033[35m",
            "cyan": "\033[36m",
            "white": "\033[37m",
            "gray": "\033[90m",
            "grey": "\033[90m",
            "orange": "\033[38;5;208m",
        }
        return palette.get(hub.color.lower(), "\033[96m")
    if hub.zone_type == ZoneType.RESTRICTED:
        return "\033[38;5;208m"
    if hub.zone_type == ZoneType.PRIORITY:
        return "\033[36m"
    if hub.zone_type == ZoneType.BLOCKED:
        return "\033[90m"
    return "\033[97m"


def _colorize_token(token: str, level: Level) -> str:
    """Color a movement token using destination hub metadata."""
    if "-" not in token:
        return token
    _, payload = token.split("-", 1)

    destination = payload
    if payload not in level.hubs and "-" in payload:
        parts = payload.split("-", 1)
        destination = parts[1]
    if destination not in level.hubs:
        return token

    return f"{_ansi_for_hub(level, destination)}{token}\033[0m"


def _colorize_line(line: str, level: Level) -> str:
    """Apply terminal colors to all movement tokens in one output line."""
    return " ".join(_colorize_token(token, level) for token in line.split())


def main() -> int:
    """Parse a map file and run the drone simulation."""
    if len(sys.argv) < 2:
        print(
            f"Usage: python3 {sys.argv[0]} "
            "[--gui] [--show-usage] [--color|--no-color] [level_file]"
        )
        return 1

    raw_args = sys.argv[1:]
    gui_mode = "--gui" in raw_args
    force_color = "--color" in raw_args
    disable_color = "--no-color" in raw_args

    if force_color and disable_color:
        print("Error: use only one of --color or --no-color")
        return 1

    args = [
        arg
        for arg in raw_args
        if arg not in ("--gui", "--color", "--no-color")
    ]
    if not gui_mode and not args:
        print(
            f"Usage: python3 {sys.argv[0]} "
            "[--gui] [--show-usage] [--color|--no-color] [level_file]"
        )
        return 1

    if gui_mode:
        from visual import run_gui

        map_path = args[0] if args else None
        run_gui(map_path)
        return 0

    map_path = args[0]

    try:
        level = Parser().parse(map_path)
        lines = Simulation(level).run()
    except (OSError, ParseError, ValueError) as error:
        print(f"Error: {error}")
        return 1

    use_color = force_color or (not disable_color and sys.stdout.isatty())
    for line in lines:
        print(_colorize_line(line, level) if use_color else line)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
