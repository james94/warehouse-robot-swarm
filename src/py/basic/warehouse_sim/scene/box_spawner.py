import numpy as np

class BoxSpawner:
    def __init__(self, world):
        self.world = world
    
    def spawn(self, count: int = 10) -> list[str]:
        from isaacsim.core.api.objects import DynamicCuboid

        ids = []
        for i in range(count):
            x = -3.0 + (i % 5) * 0.5
            y = -1.0 + (i // 5) * 0.5
            prim = f"/World/Warehouse/Boxes/Box_{i}"

            self.world.scene.add(
                DynamicCuboid(
                    prim_path=prim,
                    name=f"box_{i}",
                    position=np.array([x, y, 0.15]),
                    size=0.25,
                    color=np.array([180, 120, 60]),
                )
            )

            ids.append(f"box_{i}")
        
        return ids
