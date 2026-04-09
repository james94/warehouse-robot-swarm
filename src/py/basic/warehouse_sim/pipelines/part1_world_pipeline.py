import random
import numpy as np

from warehouse_sim.core.simulation_bootstrap import SimulationBootstrap
from warehouse_sim.scene.warehouse_scene_builder import WarehouseSceneBuilder
from warehouse_sim.scene.shelf_builder import ShelfBuilder
from warehouse_sim.scene.box_spawner import BoxSpawner
from warehouse_sim.scene.conveyor_builder import ConveyorBuilder
from warehouse_sim.scene.dock_zone_builder import DockZoneBuilder

def run_part1(config_path: str, headless: bool, max_steps: int, seed: int) -> None:
    random.seed(seed)
    np.random.seed(seed)

    boot = SimulationBootstrap(headless=headless)
    boot.setup_default_scene()

    try:
        WarehouseSceneBuilder(boot.world).build_fallback_floor()
        ShelfBuilder(boot.world).build_rows(rows=2, cols=5)
        BoxSpawner(boot.world).spawn(count=10)
        ConveyorBuilder(boot.world).build_mock_lane(lane_id=0)
        DockZoneBuilder(boot.world).build_outbound_docks(count=2)
        boot.world.reset()

        for _ in range(max_steps):
            boot.world.step(render=True)
    finally:
        boot.shutdown()
