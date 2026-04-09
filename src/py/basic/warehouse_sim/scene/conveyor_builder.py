import numpy as np

class ConveyorBuilder:
    def __init__(self, world):
        self.world = world

    def build_mock_lane(self, lane_id: int = 0, start_xy=(-2.0, 0.0), end_xy=(4.0, 0.0)) -> dict:
        from isaacsim.core.api.objects import VisualCuboid

        sx, sy = start_xy
        ex, ey = end_xy
        cx, cy = (sx + ex) / 2.0, (sy + ey) / 2.0
        length = ((ex - sx) ** 2 + (ey - sy) ** 2) ** 0.5

        self.world.scene.add(
            VisualCuboid(
                prim_path=f"/World/Warehouse/Conveyors/Lane_{lane_id}",
                name=f"conveyor_lane_{lane_id}",
                position=np.array([cx, cy, 0.05]),
                scale=np.array([length, 0.6, 0.1]),
                color=np.array([70, 70, 70]),
                size=1.0,
            )
        )

        return {"lane_id": lane_id, "start": start_xy, "end": end_xy}
        