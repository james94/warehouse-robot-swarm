


# Core
#
# Simulation Bootstrap:
#
# - Owns SimulationApp lifecycle
# - Creates and returns World
# - Handles reset, stop, shutdown safely
#
# Config Loader:
#
# - Loads YAML configs for assets, robot count, tasks, and runtime flags

from isaacsim import SimulationApp

class SimulationBootstrap:
    def __init__(self, headless: bool = False, stage_units: float = 1.0):
        self.app = SimulationApp({"headless": headless})
        from isaacsim.core.api import World
        from isaacsim.core.utils.prims import create_prim

        self._create_prim = create_prim
        self.world = World(stage_units_in_meters=stage_units)

    def setup_default_scene(self) -> None:
        self.world.scene.add_default_ground_plane()
        self._create_prim("/DistantLight", "DistantLight")

    def shutdown(self) -> None:
        self.world.stop()
        self.app.close()
