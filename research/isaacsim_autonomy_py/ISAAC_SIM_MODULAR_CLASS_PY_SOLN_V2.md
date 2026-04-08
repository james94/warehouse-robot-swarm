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

### Step 0: Bootstrap project and configs

- Create folder structure above.
- Put all tunables into configs:
  - scene_assets.yaml
  - sim_config.yaml
  - tasks.yaml

### Step 1: Implement SimulationBootstrap and ConfigLoader

Pass criteria:

- Script runs for 200 steps without scene content errors.

### Step 2: Implement scene module classes for Part 1

Order:

1. WarehouseSceneBuilder
2. ShelfBuilder
3. BoxSpawner
4. ConveyorBuilder
5. DockZoneBuilder

Pass criteria:

- Full static logistics layout appears consistently after reset.

### Step 3: Implement Part 1 pipeline orchestrator

- Compose all scene classes.
- Add deterministic random seeds for reproducibility.

Pass criteria:

- Scene rebuilds identically across runs.

### Step 4: Implement robot modules for Part 2

Order:

1. MobileRobotFactory
2. Task models/state machine
3. TaskAllocator
4. FleetController
5. BoxTransferManager

Pass criteria:

- At least one end-to-end box delivery to conveyor succeeds.

### Step 5: Add telemetry

- Implement metrics_logger and kpi_tracker.

Pass criteria:

- JSON/CSV log outputs contain task durations and completion totals.

### Step 6: Add perception hooks

- Add overhead camera.
- Expose frame API for OCR integration.

Pass criteria:

- Camera frame retrieval works during simulation run.

### Step 7: Optional ROS2/Cortex upgrade

- ROS2 bridge topic stubs.
- Cortex decision layer migration for high-level behavior.

Pass criteria:

- Control loop still stable with integration enabled.

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

Unit-level tests:

- test_task_allocator.py
- test_waypoint_tracker.py

Smoke tests with Isaac Sim runtime:

- test_part1_scene_smoke.py
  - checks scene construction does not throw
  - validates prim counts for shelves/boxes/conveyor

- test_part2_robot_smoke.py
  - checks robot spawn and one-task completion path

Manual visual checks:

- verify conveyor lanes point toward dock zones
- verify boxes transition from pick zones to conveyor drop zones

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
