from pydantic import BaseModel
from typing import Literal, TypedDict, Optional

VideoTransformType = Literal[
    "none",
    "clockwise",
    "counterclockwise",
    "rotate-180",
    "horizontal-flip",
    "vertical-flip",
    "upper-left-diagonal",
    "upper-right-diagonal",
    "automatic",
]

CameraAction = Literal[
    "group-description",
    "kill",
    "request-groups",
    "request-stream",
    "group-terminated",
    "device-disconnect",
]


class Resolution(TypedDict):
    width: int
    height: int


class CameraEventData(BaseModel):
    resolution: Optional[Resolution] = None
    transform: Optional[VideoTransformType] = None
    forceRestart: Optional[bool] = None
    file: Optional[str] = None


class CameraEventType(BaseModel):
    type: Literal["camera"]
    action: CameraAction
    target: Optional[str] = None
    devices: Optional[list[str]] = None
    data: Optional[CameraEventData] = None
