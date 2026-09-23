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


class Device(BaseModel):
    dev: str
    name: Optional[str] = None
    serverName: str


class CameraEventData(BaseModel):
    resolution: Optional[Resolution] = None
    transform: Optional[VideoTransformType] = None
    forceRestart: Optional[bool] = None
    file: Optional[str] = None


class CameraEventType(BaseModel):
    type: Literal["camera"]
    action: CameraAction
    target: Optional[Device] = None
    devices: Optional[list[Device]] = None
    data: Optional[CameraEventData] = None
