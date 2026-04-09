import math
import numpy as np

class WaypointTracker:
    def compute_cmd(self, pos_xy: tuple[float, float], goal_xy: tuple[float, float]) -> tuple[float, float, bool]:
        dx = goal_xy[0] - pos_xy[0]
        dy = goal_xy[1] - pos_xy[1]
        d = math.sqrt(dx * dx + dy * dy)
        reached = d < 0.15

        if reached:
            return 0.0, 0.0, True
        
        heading = math.atan2(dy, dx)

        lin = float(np.clip(0.4 * d, 0.02, 0.2))
        ang = float(np.clip(1.2 * heading, -1.0, 1.0))

        return lin, ang, False
