from pydantic import BaseModel, Field, ConfigDict
from typing import Literal, TypedDict, Optional


VideoTransformType = Literal[
    "none", "clockwise", "counterclockwise", "rotate-180", "horizontal-flip",
    "vertical-flip", "upper-left-diagonal", "upper-right-diagonal", "automatic",
]

CameraAction = Literal[
    "group-description", "kill", "request-groups", "request-stream",
    "group-terminated", "device-disconnect",
]


class Resolution(TypedDict):
    width: int
    height: int


class CameraEventData(BaseModel):
    devices: Optional[list[str]] = None
    deviceNames: Optional[dict[str, str]] = None
    resolution: Optional[Resolution] = None
    transform: Optional[VideoTransformType] = None
    forceRestart: Optional[bool] = None
    file: Optional[str] = None
    convertFromJpeg: Optional[bool] = None


class CameraEventType(BaseModel):
    type: Literal["camera"]
    action: CameraAction
    data: CameraEventData


# Tracking Gst Object Instances
class GstInstance(BaseModel):
    model_config = ConfigDict(arbitrary_types_allowed=True)

    device: str
    resolution: Resolution
    transform: VideoTransformType
    gst_object: object = Field(repr=False)  # hide the actual Gst object from repr