from __future__ import annotations

from enum import Enum

from pydantic import BaseModel, ConfigDict, Field, model_validator


class ZoneType(str, Enum):
    """Supported zone kinds in the drone network."""

    NORMAL = "normal"
    BLOCKED = "blocked"
    RESTRICTED = "restricted"
    PRIORITY = "priority"


class Hub(BaseModel):
    """Represents a single zone in the drone network."""

    model_config = ConfigDict(extra="forbid")

    hub_name: str = Field(pattern=r"^[^\s-]+$")
    loc_x: int
    loc_y: int
    zone_type: ZoneType = Field(default=ZoneType.NORMAL, alias="zone")
    color: str | None = None
    max_drones: int = Field(default=1, ge=1)


class Connection(BaseModel):
    """Bidirectional link between two hubs."""

    model_config = ConfigDict(extra="forbid")

    point1: str
    point2: str
    max_link_capacity: int = Field(default=1, ge=1)


class Level(BaseModel):
    """Complete description of a drone map."""

    nb_drones: int = Field(ge=1)
    start: str
    end: str
    hubs: dict[str, Hub]
    connections: list[Connection]

    @model_validator(mode="after")
    def check_consistency(self) -> "Level":
        """Validate the map is internally coherent."""
        for name in (self.start, self.end):
            if name not in self.hubs:
                raise ValueError(f"unknown hub '{name}'")
        for connection in self.connections:
            if (
                connection.point1 not in self.hubs
                or connection.point2 not in self.hubs
            ):
                raise ValueError(
                    f"connection {connection.point1}-{connection.point2}"
                    " uses unknown hub"
                )
        return self

    def zone_capacity(self, name: str) -> int | None:
        """Return max number of simultaneous drones in a zone."""
        if name in (self.start, self.end):
            return None
        return self.hubs[name].max_drones
