# Isaac Sim Modular Class-Based Python Solution V2

## 1) Why This V2 Exists

This V2 improves the earlier solution by adding:

- A concrete project folder structure.
- A two-part implementation path aligned to your request:
  - Part 1: build and validate warehouse + shelves + boxes + conveyor flow.
  - Part 2: add mobile robots to pick boxes and deliver to conveyors.
- A more explicit class decomposition for modular testing.
- Step-by-step checkpoints so you can verify each module before proceeding.

This document is built from your two problem statements plus local Isaac Sim references from your installation.

---

## 2) Combined Problem Statement (V1 + V2 Synthesis)

Design and implement a modular Isaac Sim Python application that simulates a smart warehouse logistics flow with multi-robot autonomy.

Core goals:

1. Programmatically construct a warehouse digital twin scene.
2. Create package flow from warehouse staging zones to outbound conveyor belts.
3. Add mobile robots to execute pick-and-deliver behavior.
4. Keep architecture modular for autonomy stack growth:
   - sensing
   - perception hooks
   - planning
   - control
   - logging/analytics
5. Support future ROS2, OmniGraph/Cortex, and optimization integrations.

---

## 3) Local Isaac Sim Evidence Used

The design below follows API patterns visible in your local install, especially:

- standalone_examples/tutorials/getting_started.py
- standalone_examples/tutorials/getting_started_robot.py
- standalone_examples/api/isaacsim.core.api/data_logging.py
- standalone_examples/api/isaacsim.robot.wheeled_robots.examples/jetbot_differential_move.py
- standalone_examples/api/isaacsim.robot.manipulators/franka/multiple_tasks.py
- standalone_examples/api/isaacsim.ros2.bridge/carter_multiple_robot_navigation.py
- standalone_examples/api/isaacsim.ros2.bridge/subscriber.py
- standalone_examples/api/isaacsim.cortex.framework/demo_ur10_conveyor_main.py
- standalone_examples/replicator/cosmos_writer_warehouse.py

Additional local evidence used to align this document with requested doc categories:

- Python Scripting and Tutorials coverage:
  - standalone_examples/api/isaacsim.simulation_app/hello_world.py
  - standalone_examples/tutorials/getting_started.py
  - standalone_examples/tutorials/getting_started_robot.py
  - standalone_examples/api/isaacsim.core.api/data_logging.py
  - standalone_examples/api/isaacsim.robot.manipulators/franka/multiple_tasks.py
- Digital Twin and conveyor/cortex coverage:
  - docs/py/source/extensions/isaacsim.asset.gen.conveyor/docs/index.html
  - docs/py/source/extensions/isaacsim.asset.gen.conveyor/docs/ogn/OgnIsaacConveyor.html
  - docs/py/source/extensions/isaacsim.cortex.framework/docs/index.html
  - docs/py/source/extensions/isaacsim.cortex.behaviors/docs/index.html
- Synthetic Data Generation coverage:
  - standalone_examples/api/isaacsim.replicator.examples/sdg_getting_started_01.py
  - standalone_examples/api/isaacsim.replicator.examples/sdg_getting_started_02.py
  - standalone_examples/api/isaacsim.replicator.examples/sdg_getting_started_03.py
  - standalone_examples/api/isaacsim.replicator.examples/sdg_getting_started_04.py
  - standalone_examples/api/isaacsim.replicator.grasping/grasping_workflow_sdg.py
  - standalone_examples/replicator/mobility_gen/replay_directory.py

API conventions adopted here:

- from isaacsim import SimulationApp
- World and scene-based composition
- add_reference_to_stage and asset-root discovery
- WheeledRobot + DifferentialController for mobile base control
- World.get_data_logger() for telemetry capture

---

## 4) Proposed Project Folder Structure

Use this as your implementation scaffold:

```text
warehouse_isaacsim_v2/
  README.md
  requirements.txt
  pyproject.toml
  configs/
    sim_config.yaml
    scene_assets.yaml
    tasks.yaml
  scripts/
    run_part1_world_pipeline.py
    run_part2_robot_pipeline.py
    run_full_pipeline.py
  src/
    warehouse_sim/
      __init__.py
      core/
        simulation_bootstrap.py
        config_loader.py
      scene/
        warehouse_scene_builder.py
        shelf_builder.py
        box_spawner.py
        conveyor_builder.py
        dock_zone_builder.py
      robots/
        mobile_robot_factory.py
        robot_registry.py
      tasks/
        task_models.py
        task_allocator.py
        task_state_machine.py
      control/
        fleet_controller.py
        waypoint_tracker.py
      manipulation/
        box_transfer_manager.py
      perception/
        camera_manager.py
        label_reader_stub.py
      integration/
        ros2_bridge_manager.py
        cortex_hook_manager.py
      telemetry/
        metrics_logger.py
        kpi_tracker.py
      pipelines/
        part1_world_pipeline.py
        part2_robot_pipeline.py
        full_pipeline.py
  tests/
    test_part1_scene_smoke.py
    test_part2_robot_smoke.py
    test_task_allocator.py
    test_waypoint_tracker.py
```

Why this structure works:

- Part 1 and Part 2 are independent pipelines.
- All classes are testable without monolithic script edits.
- You can swap assets/controllers without touching orchestration.

---

## 5) Module Responsibilities

### Core

- SimulationBootstrap
  - Owns SimulationApp lifecycle.
  - Creates and returns World.
  - Handles reset, stop, shutdown safely.

- ConfigLoader
  - Loads YAML configs for assets, robot count, tasks, and runtime flags.

### Scene (Part 1 heavy)

- WarehouseSceneBuilder
  - Loads warehouse USD or falls back to primitive mock layout.

- ShelfBuilder
  - Spawns shelf rows and aisle zones.

- BoxSpawner
  - Creates package boxes in source staging zones.

- ConveyorBuilder
  - Creates conveyor lane geometry or references conveyor assets.
  - Defines inbound/outbound ports.

- DockZoneBuilder
  - Marks truck docking zones and transfer endpoints.

### Robots + Tasks (Part 2 heavy)

- MobileRobotFactory
  - Spawns and initializes fleet robots.

- TaskAllocator
  - Assigns jobs based on distance/availability (extendable to battery/capacity).

- TaskStateMachine
  - Handles transitions:
    - IDLE
    - NAVIGATE_TO_PICK
    - PICK_BOX
    - NAVIGATE_TO_CONVEYOR
    - PLACE_ON_CONVEYOR
    - COMPLETE

- FleetController
  - Low-level mobile base command using DifferentialController.

- BoxTransferManager
  - Simulated pickup/place operation and ownership transfer of boxes.

### Perception + Telemetry

- CameraManager
  - Creates overhead/robot cameras and frame retrieval hooks.

- LabelReaderStub
  - Placeholder OCR/parser integration point.

- MetricsLogger and KPITracker
  - Collect run traces: task latency, throughput, idle time, completion rate.

### Optional Integrations

- Ros2BridgeManager
  - Enables isaacsim.ros2.bridge and topic scaffolding.

- CortexHookManager
  - Entry point for decider-network or behavior tree migration.

---

## 6) Two-Part Development Plan You Requested

## Part 1: Warehouse + Conveyor + Box Flow (No Robots Yet)

Goal:

- Auto-create the warehouse environment.
- Add shelves with packaged boxes.
- Add conveyor path from warehouse interior to outbound truck dock zones.

Deliverables:

- World starts and scene loads.
- Boxes spawn at pick zones.
- Conveyor lanes and dock zones are visible and labeled.
- Optional: simple scripted box movement on conveyor lane for visual validation.

Verification checklist:

1. World starts with ground and lighting.
2. Warehouse base stage loaded or primitive fallback active.
3. At least 2 shelf blocks and 1 outbound conveyor lane visible.
4. At least 10 boxes spawned in valid shelf/staging area.
5. Dock zone markers visible where truck interface would connect.

## Part 2: Add Mobile Robots for Pick-and-Place to Conveyor

Goal:

- Spawn N mobile robots.
- Assign pick tasks for spawned boxes.
- Navigate to pick positions, then to conveyor drop points.
- Mark task complete when box is placed on conveyor lane.

Deliverables:

- Robots move autonomously using modular controller classes.
- Task states transition reliably.
- Throughput and completion metrics logged.

Verification checklist:

1. Fleet spawns collision-free.
2. Each robot gets assigned tasks only when idle.
3. Box ownership transfers from shelf zone to robot to conveyor zone.
4. Task completion events logged for each delivered box.
5. KPI file saved with run summary.

---

## 7) Step-by-Step Build Sequence

This section now includes starter code for every Python script in Section 4.

### Step 0: Create package and script entrypoints

Create these files first:

1. src/warehouse_sim/__init__.py
2. scripts/run_part1_world_pipeline.py
3. scripts/run_part2_robot_pipeline.py
4. scripts/run_full_pipeline.py

Code template: src/warehouse_sim/__init__.py

```python
"""Warehouse Isaac Sim package."""

__all__ = ["core", "scene", "robots", "tasks", "control", "manipulation", "perception", "integration", "telemetry", "pipelines"]
```

Code template: scripts/run_part1_world_pipeline.py

```python
import argparse

from warehouse_sim.pipelines.part1_world_pipeline import run_part1


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--config", default="configs/sim_config.yaml")
  parser.add_argument("--headless", action="store_true")
  parser.add_argument("--steps", type=int, default=1500)
  parser.add_argument("--seed", type=int, default=42)
  args = parser.parse_args()
  run_part1(config_path=args.config, headless=args.headless, max_steps=args.steps, seed=args.seed)


if __name__ == "__main__":
  main()
```

Code template: scripts/run_part2_robot_pipeline.py

```python
import argparse

from warehouse_sim.pipelines.part2_robot_pipeline import run_part2


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--config", default="configs/sim_config.yaml")
  parser.add_argument("--headless", action="store_true")
  parser.add_argument("--steps", type=int, default=2500)
  parser.add_argument("--seed", type=int, default=42)
  args = parser.parse_args()
  run_part2(config_path=args.config, headless=args.headless, max_steps=args.steps, seed=args.seed)


if __name__ == "__main__":
  main()
```

Code template: scripts/run_full_pipeline.py

```python
import argparse

from warehouse_sim.pipelines.full_pipeline import run_full


def main() -> None:
  parser = argparse.ArgumentParser()
  parser.add_argument("--config", default="configs/sim_config.yaml")
  parser.add_argument("--headless", action="store_true")
  parser.add_argument("--mode", choices=["only_scene", "scene_plus_robots", "with_ros2", "with_cortex_hooks"], default="scene_plus_robots")
  parser.add_argument("--steps", type=int, default=2500)
  parser.add_argument("--seed", type=int, default=42)
  args = parser.parse_args()
  run_full(config_path=args.config, headless=args.headless, mode=args.mode, max_steps=args.steps, seed=args.seed)


if __name__ == "__main__":
  main()
```

### Step 1: Core modules

Create:

1. src/warehouse_sim/core/simulation_bootstrap.py
2. src/warehouse_sim/core/config_loader.py

Code template: src/warehouse_sim/core/simulation_bootstrap.py

```python
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
```

Code template: src/warehouse_sim/core/config_loader.py

```python
from pathlib import Path
import yaml


class ConfigLoader:
  @staticmethod
  def load_yaml(path: str) -> dict:
    p = Path(path)
    if not p.exists():
      raise FileNotFoundError(f"Config not found: {path}")
    with p.open("r", encoding="utf-8") as f:
      return yaml.safe_load(f) or {}

  @staticmethod
  def validate_required(cfg: dict, required_keys: list[str]) -> None:
    missing = [k for k in required_keys if k not in cfg]
    if missing:
      raise ValueError(f"Missing config keys: {missing}")
```

### Step 2: Scene modules for Part 1

Create:

1. src/warehouse_sim/scene/warehouse_scene_builder.py
2. src/warehouse_sim/scene/shelf_builder.py
3. src/warehouse_sim/scene/box_spawner.py
4. src/warehouse_sim/scene/conveyor_builder.py
5. src/warehouse_sim/scene/dock_zone_builder.py

Code template: src/warehouse_sim/scene/warehouse_scene_builder.py

```python
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
```

Code template: src/warehouse_sim/scene/shelf_builder.py

```python
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
```

Code template: src/warehouse_sim/scene/box_spawner.py

```python
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
```

Code template: src/warehouse_sim/scene/conveyor_builder.py

```python
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
```

Code template: src/warehouse_sim/scene/dock_zone_builder.py

```python
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
```

### Step 3: Robot and task modules for Part 2

Create:

1. src/warehouse_sim/robots/mobile_robot_factory.py
2. src/warehouse_sim/robots/robot_registry.py
3. src/warehouse_sim/tasks/task_models.py
4. src/warehouse_sim/tasks/task_state_machine.py
5. src/warehouse_sim/tasks/task_allocator.py
6. src/warehouse_sim/control/waypoint_tracker.py
7. src/warehouse_sim/control/fleet_controller.py
8. src/warehouse_sim/manipulation/box_transfer_manager.py

Code template: src/warehouse_sim/robots/mobile_robot_factory.py

```python
import numpy as np


class MobileRobotFactory:
  def __init__(self, world, assets_root_path: str):
    self.world = world
    self.assets_root_path = assets_root_path

  def spawn_jetbots(self, count: int, spacing: float = 1.0) -> dict:
    from isaacsim.robot.wheeled_robots.robots import WheeledRobot

    usd = self.assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
    robots = {}
    for i in range(count):
      y = (i - (count - 1) / 2.0) * spacing
      name = f"robot_{i}"
      robots[name] = self.world.scene.add(
        WheeledRobot(
          prim_path=f"/World/Fleet/{name}",
          name=name,
          wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
          create_robot=True,
          usd_path=usd,
          position=np.array([0.0, y, 0.05]),
        )
      )
    return robots
```

Code template: src/warehouse_sim/robots/robot_registry.py

```python
class RobotRegistry:
  def __init__(self):
    self._robots = {}

  def set(self, robots: dict) -> None:
    self._robots = dict(robots)

  def get(self, name: str):
    return self._robots[name]

  def names(self) -> list[str]:
    return list(self._robots.keys())
```

Code template: src/warehouse_sim/tasks/task_models.py

```python
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
```

Code template: src/warehouse_sim/tasks/task_state_machine.py

```python
from warehouse_sim.tasks.task_models import TaskPhase, BoxTask


class TaskStateMachine:
  def on_reached_pick(self, task: BoxTask) -> None:
    task.phase = TaskPhase.PICK_BOX

  def on_picked(self, task: BoxTask) -> None:
    task.phase = TaskPhase.NAVIGATE_TO_CONVEYOR

  def on_reached_drop(self, task: BoxTask) -> None:
    task.phase = TaskPhase.PLACE_ON_CONVEYOR

  def on_placed(self, task: BoxTask) -> None:
    task.phase = TaskPhase.COMPLETE
```

Code template: src/warehouse_sim/tasks/task_allocator.py

```python
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
```

Code template: src/warehouse_sim/control/waypoint_tracker.py

```python
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
```

Code template: src/warehouse_sim/control/fleet_controller.py

```python
from warehouse_sim.control.waypoint_tracker import WaypointTracker


class FleetController:
  def __init__(self, robots: dict):
    from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController

    self.robots = robots
    self.tracker = WaypointTracker()
    self.ctrl = {n: DifferentialController(name=f"{n}_ctrl", wheel_radius=0.03, wheel_base=0.1125) for n in robots}

  def step_to_goal(self, robot_name: str, goal_xy: tuple[float, float]) -> bool:
    robot = self.robots[robot_name]
    pos, _ = robot.get_world_pose()
    lin, ang, reached = self.tracker.compute_cmd((float(pos[0]), float(pos[1])), goal_xy)
    robot.apply_wheel_actions(self.ctrl[robot_name].forward(command=[lin, ang]))
    return reached
```

Code template: src/warehouse_sim/manipulation/box_transfer_manager.py

```python
class BoxTransferManager:
  def __init__(self):
    self.box_owner = {}

  def mark_picked(self, box_id: str, robot_name: str) -> None:
    self.box_owner[box_id] = robot_name

  def mark_placed_on_conveyor(self, box_id: str) -> None:
    self.box_owner[box_id] = "conveyor"
```

### Step 4: Perception, integration, telemetry modules

Create:

1. src/warehouse_sim/perception/camera_manager.py
2. src/warehouse_sim/perception/label_reader_stub.py
3. src/warehouse_sim/integration/ros2_bridge_manager.py
4. src/warehouse_sim/integration/cortex_hook_manager.py
5. src/warehouse_sim/telemetry/metrics_logger.py
6. src/warehouse_sim/telemetry/kpi_tracker.py

Code template: src/warehouse_sim/perception/camera_manager.py

```python
import numpy as np


class CameraManager:
  def __init__(self, world):
    self.world = world
    self.camera = None

  def create_overhead(self) -> None:
    from isaacsim.sensors.camera import Camera
    import isaacsim.core.utils.numpy.rotations as rot_utils

    self.camera = Camera(
      prim_path="/World/Sensors/OverheadCamera",
      position=np.array([0.0, 0.0, 10.0]),
      frequency=10,
      resolution=(640, 480),
      orientation=rot_utils.euler_angles_to_quats(np.array([0, 90, 0]), degrees=True),
    )
    self.camera.initialize()

  def get_rgba(self):
    return None if self.camera is None else self.camera.get_rgba()
```

Code template: src/warehouse_sim/perception/label_reader_stub.py

```python
class LabelReaderStub:
  def infer(self, rgba_frame):
    return {"labels": [], "confidence": []}
```

Code template: src/warehouse_sim/integration/ros2_bridge_manager.py

```python
class Ros2BridgeManager:
  def enable(self) -> None:
    from isaacsim.core.utils.extensions import enable_extension

    enable_extension("isaacsim.ros2.bridge")
```

Code template: src/warehouse_sim/integration/cortex_hook_manager.py

```python
class CortexHookManager:
  def setup(self) -> None:
    # Hook for future decider-network integration.
    return
```

Code template: src/warehouse_sim/telemetry/metrics_logger.py

```python
class MetricsLogger:
  def __init__(self, world, robot_states):
    self.logger = world.get_data_logger()
    self.robot_states = robot_states

  def start(self) -> None:
    def _frame(tasks, scene):
      return {name: state for name, state in self.robot_states.items()}

    self.logger.add_data_frame_logging_func(_frame)
    self.logger.start()

  def save(self, path: str) -> None:
    self.logger.save(log_path=path)
```

Code template: src/warehouse_sim/telemetry/kpi_tracker.py

```python
class KPITracker:
  def __init__(self):
    self.completed = 0
    self.total = 0

  def set_total(self, total: int) -> None:
    self.total = total

  def mark_completed(self) -> None:
    self.completed += 1

  def summary(self) -> dict:
    rate = 0.0 if self.total == 0 else self.completed / self.total
    return {"completed": self.completed, "total": self.total, "completion_rate": rate}
```

### Step 5: Pipeline modules

Create:

1. src/warehouse_sim/pipelines/part1_world_pipeline.py
2. src/warehouse_sim/pipelines/part2_robot_pipeline.py
3. src/warehouse_sim/pipelines/full_pipeline.py

Code template: src/warehouse_sim/pipelines/part1_world_pipeline.py

```python
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
```

Code template: src/warehouse_sim/pipelines/part2_robot_pipeline.py

```python
import random
import numpy as np

from warehouse_sim.core.simulation_bootstrap import SimulationBootstrap
from warehouse_sim.scene.warehouse_scene_builder import WarehouseSceneBuilder
from warehouse_sim.scene.shelf_builder import ShelfBuilder
from warehouse_sim.scene.box_spawner import BoxSpawner
from warehouse_sim.scene.conveyor_builder import ConveyorBuilder
from warehouse_sim.scene.dock_zone_builder import DockZoneBuilder
from warehouse_sim.robots.mobile_robot_factory import MobileRobotFactory
from warehouse_sim.control.fleet_controller import FleetController


def run_part2(config_path: str, headless: bool, max_steps: int, seed: int) -> None:
  random.seed(seed)
  np.random.seed(seed)

  boot = SimulationBootstrap(headless=headless)
  boot.setup_default_scene()
  try:
    WarehouseSceneBuilder(boot.world).build_fallback_floor()
    ShelfBuilder(boot.world).build_rows(rows=2, cols=5)
    BoxSpawner(boot.world).spawn(count=10)
    ConveyorBuilder(boot.world).build_mock_lane(lane_id=0)
    docks = DockZoneBuilder(boot.world).build_outbound_docks(count=2)

    from isaacsim.storage.native import get_assets_root_path

    assets_root = get_assets_root_path()
    robots = MobileRobotFactory(boot.world, assets_root).spawn_jetbots(count=3, spacing=1.2)
    controller = FleetController(robots)

    boot.world.reset()
    goal = docks[0]
    for _ in range(max_steps):
      boot.world.step(render=True)
      for name in robots.keys():
        controller.step_to_goal(name, goal)
  finally:
    boot.shutdown()
```

Code template: src/warehouse_sim/pipelines/full_pipeline.py

```python
from warehouse_sim.pipelines.part1_world_pipeline import run_part1
from warehouse_sim.pipelines.part2_robot_pipeline import run_part2


def run_full(config_path: str, headless: bool, mode: str, max_steps: int, seed: int) -> None:
  if mode == "only_scene":
    run_part1(config_path=config_path, headless=headless, max_steps=max_steps, seed=seed)
    return

  if mode in ["scene_plus_robots", "with_ros2", "with_cortex_hooks"]:
    run_part2(config_path=config_path, headless=headless, max_steps=max_steps, seed=seed)
    return

  raise ValueError(f"Unsupported mode: {mode}")
```

Pass criteria for all steps:

1. Each script imports cleanly.
2. Part 1 runs with shelves, boxes, conveyors, and docks visible.
3. Part 2 runs with robot motion and at least one successful box handoff simulation path.
4. Full pipeline mode switches work without manual file edits.

---

## 8) Reference Class Skeletons (Concise)

## 8.1 task_models.py

```python
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional, Tuple


class TaskPhase(Enum):
    IDLE = auto()
    NAVIGATE_TO_PICK = auto()
    PICK_BOX = auto()
    NAVIGATE_TO_DROP = auto()
    PLACE_BOX = auto()
    COMPLETE = auto()


@dataclass
class BoxTask:
    task_id: int
    box_id: str
    pick_xy: Tuple[float, float]
    drop_xy: Tuple[float, float]
    assigned_robot: Optional[str] = None
    phase: TaskPhase = TaskPhase.IDLE
```

## 8.2 simulation_bootstrap.py

```python
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
```

## 8.3 conveyor_builder.py

```python
import numpy as np


class ConveyorBuilder:
    def __init__(self, world):
        self.world = world

    def build_mock_conveyor_lane(self, start_xy=(-2.0, 0.0), end_xy=(2.0, 0.0)) -> dict:
        from isaacsim.core.api.objects import VisualCuboid

        sx, sy = start_xy
        ex, ey = end_xy
        cx, cy = (sx + ex) / 2.0, (sy + ey) / 2.0
        length = ((ex - sx) ** 2 + (ey - sy) ** 2) ** 0.5

        self.world.scene.add(
            VisualCuboid(
                prim_path="/World/Conveyors/Lane_0",
                name="conveyor_lane_0",
                position=np.array([cx, cy, 0.05]),
                scale=np.array([length, 0.5, 0.1]),
                color=np.array([80, 80, 80]),
                size=1.0,
            )
        )
        return {"lane_id": "lane_0", "start": start_xy, "end": end_xy}
```

## 8.4 mobile_robot_factory.py

```python
import numpy as np


class MobileRobotFactory:
    def __init__(self, world, assets_root_path: str):
        self.world = world
        self.assets_root_path = assets_root_path

    def spawn_jetbots(self, count: int, spacing: float = 1.0):
        from isaacsim.robot.wheeled_robots.robots import WheeledRobot

        robots = {}
        usd_path = self.assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
        for i in range(count):
            y = (i - (count - 1) / 2.0) * spacing
            name = f"robot_{i}"
            robots[name] = self.world.scene.add(
                WheeledRobot(
                    prim_path=f"/World/Fleet/{name}",
                    name=name,
                    wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
                    create_robot=True,
                    usd_path=usd_path,
                    position=np.array([0.0, y, 0.05]),
                )
            )
        return robots
```

## 8.5 fleet_controller.py

```python
import math
import numpy as np


class FleetController:
    def __init__(self, robots):
        from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController

        self.robots = robots
        self.controllers = {
            name: DifferentialController(name=f"{name}_ctrl", wheel_radius=0.03, wheel_base=0.1125)
            for name in robots
        }

    def drive_to_xy(self, name: str, goal_xy: tuple, reach_thresh: float = 0.15) -> bool:
        robot = self.robots[name]
        ctrl = self.controllers[name]
        pos, _ = robot.get_world_pose()
        x, y = float(pos[0]), float(pos[1])
        gx, gy = goal_xy
        dx, dy = gx - x, gy - y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist <= reach_thresh:
            robot.apply_wheel_actions(ctrl.forward(command=[0.0, 0.0]))
            return True

        heading = math.atan2(dy, dx)
        lin = np.clip(0.4 * dist, 0.02, 0.2)
        ang = np.clip(1.0 * heading, -1.0, 1.0)
        robot.apply_wheel_actions(ctrl.forward(command=[lin, ang]))
        return False
```

---

## 9) Pipeline Orchestration Pattern

## part1_world_pipeline.py

- Build world.
- Build warehouse static layout.
- Spawn shelves and boxes.
- Build conveyor lanes and truck dock zones.
- Run scene-only simulation for visual inspection.

## part2_robot_pipeline.py

- Start from Part 1 world.
- Spawn robot fleet.
- Initialize task graph from available boxes.
- Run allocation + movement + place-on-conveyor loop.
- Log metrics and task completion.

## full_pipeline.py

- Loads both pipelines.
- Enables toggles:
  - only_scene
  - scene_plus_robots
  - with_ros2
  - with_cortex_hooks

---

## 10) Testing Strategy (Incremental)

This section is now mapped directly to Section 7 scripts.

### A) Fast unit tests

Create:

1. tests/test_task_allocator.py
2. tests/test_waypoint_tracker.py
3. tests/test_config_loader.py

Checks:

1. deterministic assignment for fixed robot and task inputs
2. controller command values stay within expected bounds
3. missing config keys raise explicit errors

### B) Part 1 smoke tests

Create:

1. tests/test_part1_scene_smoke.py
2. tests/test_scene_fallback_assets.py

Checks:

1. part1_world_pipeline builds scene without exception
2. shelf, box, conveyor, dock prims are created
3. fallback scene path works when warehouse USD is unavailable

### C) Part 2 smoke tests

Create:

1. tests/test_part2_robot_smoke.py
2. tests/test_task_state_machine.py
3. tests/test_box_transfer_manager.py

Checks:

1. robot fleet spawns and can step
2. task transitions follow IDLE -> NAVIGATE_TO_PICK -> PICK_BOX -> NAVIGATE_TO_CONVEYOR -> PLACE_ON_CONVEYOR -> COMPLETE
3. ownership transitions shelf -> robot -> conveyor

### D) Telemetry and KPI tests

Create:

1. tests/test_metrics_logger.py
2. tests/test_kpi_tracker.py

Checks:

1. telemetry frame writing succeeds
2. KPI completion rate summary is computed correctly

### E) Optional integration tests

Create:

1. tests/test_ros2_bridge_optional.py
2. tests/test_cortex_hooks_optional.py
3. tests/test_replicator_optional.py

Checks:

1. ros2 bridge enable call does not break base pipeline
2. cortex hook setup does not break base pipeline
3. optional synthetic data capture path can run in isolation

### Recommended command order

```bash
cd /home/ubuntu/isaacsim
./python.sh -m pytest /home/ubuntu/src/warehouse-robot-swarm/<path-to>/warehouse_isaacsim_v2/tests/test_config_loader.py -q
./python.sh -m pytest /home/ubuntu/src/warehouse-robot-swarm/<path-to>/warehouse_isaacsim_v2/tests/test_part1_scene_smoke.py -q
./python.sh -m pytest /home/ubuntu/src/warehouse-robot-swarm/<path-to>/warehouse_isaacsim_v2/tests/test_part2_robot_smoke.py -q
./python.sh -m pytest /home/ubuntu/src/warehouse-robot-swarm/<path-to>/warehouse_isaacsim_v2/tests -q
```

### Manual visual acceptance checklist

1. conveyor lanes are oriented toward dock zones
2. boxes are visible before robot run begins
3. robots spawn without collision
4. at least one box reaches a conveyor drop zone
5. telemetry output files are produced

---

## 11) Asset and Path Notes

To keep the solution robust across Isaac Sim versions:

1. Keep USD paths in configs/scene_assets.yaml.
2. Validate each configured asset path at startup.
3. If warehouse/conveyor asset is missing, use primitive fallback builders.

This avoids hard failures when asset library layouts differ.

---

## 12) How To Execute

From Isaac Sim root:

```bash
cd /home/ubuntu/isaacsim
./python.sh /home/ubuntu/src/warehouse-robot-swarm/<path-to>/warehouse_isaacsim_v2/scripts/run_part1_world_pipeline.py
./python.sh /home/ubuntu/src/warehouse-robot-swarm/<path-to>/warehouse_isaacsim_v2/scripts/run_part2_robot_pipeline.py
```

Expected outputs:

- Part 1:
  - warehouse, shelves, boxes, conveyor lanes, dock markers
- Part 2:
  - robots completing pick-and-deliver tasks
- Logs:
  - telemetry and KPI files under output directory

---

## 13) How This V2 Improves V1

Compared to the earlier modular writeup, this V2 adds:

- explicit project layout for real implementation
- phased delivery model (Part 1 then Part 2)
- dedicated conveyor and dock-zone modules
- dedicated box transfer manager for pick/place lifecycle
- stronger testing strategy and pass criteria per stage
- clean integration hooks for ROS2 and Cortex upgrade paths

---

## 14) Next Recommended Expansion

1. Replace mock conveyors with conveyor extension-driven motion behavior.
2. Add manipulator-equipped mobile platform for physically realistic pickup.
3. Introduce queue-aware scheduler for dock throughput optimization.
4. Integrate cuOpt or external optimizer for fleet-level routing.
5. Add OCR-backed package validation in the perception pipeline.

This keeps the architecture stable while scaling from MVP simulation to research-grade autonomy experiments.
