import numpy as np

class WarehouseSceneBuilder:
    def __init__(self, world):
        self.world = world

    def build_fallback_floor(self) -> None:
        from isaacsim.core.api.objects import VisualCuboid

        self.world.scene.add(
            VisualCuboid(
                prim_path="/World/Warehouse/Floor",
                name="warehouse_floor",
                position=np.array([0.0, 0.0, -0.02]),
                scale=np.array([20.0, 20.0, 0.04]),
                color=np.array([120, 120, 120]),
                size=1.0,
            )
        )
        