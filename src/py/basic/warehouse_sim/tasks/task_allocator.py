import math
from warehouse_sim.tasks.task_models import BoxTask, TaskPhase

class TaskAllocator:
    @staticmethod
    def _dist(a: tuple[float, float], b: tuple[float, float]) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)
    
    def assign(self, robot_name: str, robot_xy: tuple[float, float], tasks: list[BoxTask]) -> BoxTask | None:
        candidates = [t for t in tasks if t.assigned_robot is None and t.phase == TaskPhase.IDLE]

        if not candidates:
            return None

        best = min(candidates, key=lambda t: self._dist(robot_xy, t.pick_xy))
        best.assigned_robot = robot_name
        best.phase = TaskPhase.NAVIGATE_TO_PICK
        return best
