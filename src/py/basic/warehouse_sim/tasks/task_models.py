from dataclasses import dataclass
from enum import Enum, auto

class TaskPhase(Enum):
    IDLE = auto()
    NAVIGATE_TO_PICK = auto()
    PICK_BOX = auto()
    NAVIGATE_TO_CONVEYOR = auto()
    PLACE_ON_CONVEYOR = auto()
    COMPLETE = auto()

@dataclass
class BoxTask:
    task_id: int
    box_id: str
    pick_xy: tuple[float, float]
    drop_xy: tuple[float, float]
    assigned_robot: str | None = None
    phase: TaskPhase = TaskPhase.IDLE
