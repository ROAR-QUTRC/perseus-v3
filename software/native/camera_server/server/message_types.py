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
    "kill-stream",
    "request-groups",
    "request-stream",
    "group-terminated",
    "device-disconnect",
]


class Resolution(TypedDict):
    width: int
    height: int


class DeviceInfo(BaseModel):
    device: str
    name: Optional[str] = None


class CameraEventData(BaseModel):
    resolution: Optional[Resolution] = None
    transform: Optional[VideoTransformType] = None
    forceRestart: Optional[bool] = None
    redirect: Optional[str] = None


class CameraEventType(BaseModel):
    type: Literal["camera"]
    group: Optional[str] = None
    action: CameraAction
    target: Optional[DeviceInfo] = None
    devices: Optional[list[DeviceInfo]] = None
    data: Optional[CameraEventData] = None
