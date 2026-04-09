import numpy as np

class DockZoneBuilder:
    def __init__(self, world):
        self.world = world


    def build_outbound_docks(self, count: int = 2) -> list[tuple[float, float]]:
        from isaacsim.core.api.objects import VisualCuboid

        docks = []

        for i in range(count):
            y = -1.5 + i * 3.0
            x = 6.0
            docks.append((x, y))
            self.world.scene.add(
                VisualCuboid(
                    prim_path=f"/World/Warehouse/Docks/Dock_{i}",
                    name=f"dock_{i}",
                    position=np.array([x, y, 0.02]),
                    scale=np.array([1.2, 1.8, 0.04]),
                    color=np.array([0, 150, 220]),
                    size=1.0,
                )
            )

        return docks
