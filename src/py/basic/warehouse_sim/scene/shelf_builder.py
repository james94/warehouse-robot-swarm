import numpy as np

class ShelfBuilder:
    def __init__(self, world):
        self.world = world

    def build_rows(self, rows: int = 2, cols: int = 5) -> None:
        from isaacsim.core.api.objects import VisualCuboid

        for r in range(rows):
            for c in range(cols):
                self.world.scene.add(
                    VisualCuboid(
                        prim_path=f"/World/Warehouse/Shelves/Shelf_{r}_{c}",
                        name=f"shelf_{r}_{c}",
                        position=np.array([c * 1.8 - 4.0, r * 3.0 - 1.5, 0.6]),
                        scale=np.array([1.5, 0.6, 1.2]),
                        color=np.array([90, 90, 110]),
                        size=1.0,
                    )
                )
