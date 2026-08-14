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

    schema_version: Literal["1.0"] = "1.0"
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
    reason: str | None = None

    @classmethod
    def create_abort(cls, reason: str = "unknown_reason") -> "RobotPlan":
        return cls(
            schema_version="1.0",
            task="abort",
            target_object_id=None,
            destination_zone_id=None,
            steps=[
                PlanStep(
                    action=ActionName.ABORT,
                    object_id=None,
                    zone_id=None,
                    height_m=None,
                )
            ],
            reason=reason,
        )

