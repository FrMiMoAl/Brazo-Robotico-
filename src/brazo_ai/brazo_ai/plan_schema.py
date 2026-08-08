from enum import Enum
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field


class ActionName(str, Enum):
    APPROACH = "approach"
    GRASP = "grasp"
    LIFT = "lift"
    PLACE = "place"
    HOME = "home"
    ABORT = "abort"


class PlanStep(BaseModel):
    model_config = ConfigDict(extra="forbid")

    action: ActionName
    object_id: str | None = None
    zone_id: str | None = None
    height_m: float | None = Field(
        default=None,
        ge=0.0,
        le=0.15,
    )


class RobotPlan(BaseModel):
    model_config = ConfigDict(extra="forbid")

    schema_version: Literal["1.0"]
    task: Literal[
        "pick",
        "pick_and_place",
        "home",
        "abort",
    ]
    target_object_id: str | None = None
    destination_zone_id: str | None = None
    steps: list[PlanStep] = Field(
        min_length=1,
        max_length=8,
    )
