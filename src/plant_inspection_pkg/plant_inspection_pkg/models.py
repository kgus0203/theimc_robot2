from dataclasses import asdict, dataclass
from datetime import datetime
from typing import Dict, List, Optional


def utc_offset_timestamp() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def make_inspection_id(timestamp: str, bed_id: str, hole_id: str) -> str:
    compact = datetime.fromisoformat(timestamp).strftime("%Y%m%d_%H%M%S")
    return f"INS_{compact}_B{bed_id}_H{hole_id}"


@dataclass
class Location:
    region: str
    device_number: int
    bed_id: int
    hole_id: int


@dataclass
class Environment:
    temperature_c: Optional[float]
    humidity_pct: Optional[float]
    co2_ppm: Optional[int]
    solar_radiation_w_m2: Optional[float]


@dataclass
class Position:
    x: float
    y: float
    z: float


@dataclass
class Orientation:
    x: float
    y: float
    z: float
    w: float


@dataclass
class CameraPose:
    frame_id: str
    position: Position
    orientation: Orientation


@dataclass
class ArmPose:
    joints_deg: List[float]
    coords: List[float]


@dataclass
class Capture:
    viewpoint_id: str
    captured_at: str
    camera_pose: CameraPose
    arm_pose: ArmPose
    files: Dict[str, str]
    aligned_depth: bool


@dataclass
class Inspection:
    inspection_id: str
    timestamp: str
    location: Location
    environment: Environment
    captures: List[Capture]

    def to_dict(self) -> Dict:
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> "Inspection":
        location = Location(**data["location"])
        environment_data = dict(data["environment"])
        # Old queued inspections contain lux, which cannot be reinterpreted as W/m².
        environment_data.pop("illuminance_lux", None)
        environment_data.setdefault("solar_radiation_w_m2", None)
        environment = Environment(**environment_data)
        captures = []
        for item in data["captures"]:
            camera = item["camera_pose"]
            arm = item["arm_pose"]
            captures.append(
                Capture(
                    viewpoint_id=item["viewpoint_id"],
                    captured_at=item["captured_at"],
                    camera_pose=CameraPose(
                        frame_id=camera["frame_id"],
                        position=Position(**camera["position"]),
                        orientation=Orientation(**camera["orientation"]),
                    ),
                    arm_pose=ArmPose(**arm),
                    files=dict(item["files"]),
                    aligned_depth=bool(item.get("aligned_depth", True)),
                )
            )
        return cls(
            inspection_id=data["inspection_id"],
            timestamp=data["timestamp"],
            location=location,
            environment=environment,
            captures=captures,
        )


@dataclass
class FilePayload:
    field_name: str
    filename: str
    content_type: str
    data: bytes
