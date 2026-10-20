from pydantic import BaseModel, ConfigDict, Field, model_validator
from enum import Enum


class ZoneType(str, Enum):
    NORMAL = "normal"
    BLOCKED = "blocked"
    RESTRICTED = "restricted"
    PRIORITY = "priority"


class Hub(BaseModel):
    model_config = ConfigDict(extra="forbid")
    hub_name: str = Field(pattern=r"^[^\s-]+$")
    loc_x: int
    loc_y: int
    zone_type: ZoneType = Field(default=ZoneType.NORMAL, alias="zone")
    color: str | None = None
    max_drones: int = Field(default=1, ge=1)


class Connection(BaseModel):
    model_config = ConfigDict(extra="forbid")
    point1: str
    point2: str
    max_link_capacity: int = Field(default=1, ge=1)


class Level(BaseModel):
    nb_drones: int = Field(ge=1)
    start: str
    end: str
    hubs: dict[str, Hub]
    connections: list[Connection]

    @model_validator(mode="after")
    def check_consistency(self) -> "Level":
        for name in (self.start, self.end):
            if name not in self.hubs:
                raise ValueError(f"unknown hub '{name}'")
        for c in self.connections:
            if c.point1 not in self.hubs or c.point2 not in self.hubs:
                raise ValueError(
                    f"connection {c.point1}-{c.point2}"
                    " uses unknown hub"
                )
        return self

    def zone_capacity(self, name: str) -> int | None:
        """None means unlimited (start and end zones)"""
        if name in (self.start, self.end):
            return None
        return self.hubs[name].max_drones