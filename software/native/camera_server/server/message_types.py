from pydantic import BaseModel
from typing import Literal, TypedDict, Optional

# The following types deviate from the python naming convention as
# they are required to match the types used in the webui

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
    "device-connect",
]


class Resolution(TypedDict):
    width: int
    height: int


class Device(BaseModel):
    name: Optional[str] = None
    dev: str
    hostname: str
    isVirtual: Optional[bool] = None


class CameraEventData(BaseModel):
    resolution: Optional[Resolution] = None
    transform: Optional[VideoTransformType] = None
    forceRestart: Optional[bool] = None
    redirect: Optional[Device] = None


class CameraEventType(BaseModel):
    type: Literal["camera"]
    action: CameraAction
    target: Optional[Device] = None
    devices: Optional[list[Device]] = None
    data: Optional[CameraEventData] = None
