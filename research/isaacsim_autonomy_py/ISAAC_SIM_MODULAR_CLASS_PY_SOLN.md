# Isaac Sim Modular Class-Based Python Solution

## 1) Combined Problem Statement (V1 + V2 Synthesis)

Build a scalable multi-robot warehouse simulation in NVIDIA Isaac Sim where a fleet of autonomous mobile robots (with a path to mobile-manipulation) performs pick-transport-drop workflows under centralized task allocation.

The solution should:

- Use Isaac Sim Python APIs to programmatically create and run the world.
- Support modular autonomy layers: sensing, perception hooks, planning, and control.
- Provide fleet coordination with dynamic task assignment.
- Be structured so each subsystem is testable independently.
- Support future extensions: ROS2 bridge, OmniGraph/behavior logic, data logging, synthetic data, and training loops.

This document provides a modular class-based implementation pattern that you can build step-by-step and verify at each stage.

---

## 2) What Was Parsed From Your Local Isaac Sim Install

The structure and APIs below are grounded in your installed examples/docs, including:

- `standalone_examples/tutorials/getting_started.py`
- `standalone_examples/tutorials/getting_started_robot.py`
- `standalone_examples/api/isaacsim.core.api/add_frankas.py`
- `standalone_examples/api/isaacsim.core.api/control_robot.py`
- `standalone_examples/api/isaacsim.core.api/data_logging.py`
- `standalone_examples/api/isaacsim.robot.wheeled_robots.examples/jetbot_differential_move.py`
- `standalone_examples/api/isaacsim.robot.manipulators/franka/multiple_tasks.py`
- `standalone_examples/api/isaacsim.ros2.bridge/carter_multiple_robot_navigation.py`
- `standalone_examples/api/isaacsim.ros2.bridge/subscriber.py`
- `standalone_examples/api/isaacsim.cortex.framework/demo_ur10_conveyor_main.py`

The code below follows the API style shown in those examples (not legacy omni.isaac.kit style):

- `from isaacsim import SimulationApp`
- `World`, `Robot`, `WheeledRobot`, `DifferentialController`
- `add_reference_to_stage`, `get_assets_root_path`
- `World.get_data_logger()` for run-time data capture

---

## 3) Modular Architecture

Use the following class decomposition:

1. `SimulationBootstrap`
2. `WarehouseSceneBuilder`
3. `RobotFactory`
4. `TaskAllocator`
5. `FleetController`
6. `PerceptionManager` (MVP camera hooks)
7. `MetricsLogger`
8. `WarehouseSimulationRunner`

Data model classes:

- `SimConfig`
- `RobotState`
- `TaskOrder`

This keeps your pipeline readable and allows you to validate each layer independently.

---

## 4) Step-by-Step Build Plan (Recommended)

### Step 1: Boot Isaac Sim + World

Goal: launch app, build `World`, add ground/light.

Verification:

- Script opens Isaac Sim GUI.
- Ground plane is visible.
- Simulation steps without errors for 200-300 frames.

### Step 2: Spawn Multiple Mobile Robots

Goal: spawn N Jetbots (or Carter/Nova Carter when desired) with deterministic start poses.

Verification:

- Robots appear at expected positions.
- No startup collisions.

### Step 3: Add Fleet Motion Controller

Goal: per-robot differential drive control and waypoint tracking.

Verification:

- Each robot can drive toward a target waypoint.
- Wheel actions are applied each frame.

### Step 4: Add Task Allocation

Goal: central assignment of pick/drop tasks to idle robots with a simple cost function.

Verification:

- Idle robots receive tasks.
- Task state transitions occur (`IDLE -> TO_PICK -> TO_DROP -> IDLE`).

### Step 5: Add Logging + KPI Tracking

Goal: capture frame-by-frame telemetry and save JSON for analysis.

Verification:

- Log file created.
- Frames contain robot pose/task state data.

### Step 6: Add Perception Hooks

Goal: attach camera(s) to world/robots and collect frames for OCR/perception pipeline integration.

Verification:

- Camera initializes and returns RGBA frames.

### Step 7 (Optional): ROS2/Cortex/OmniGraph

Goal: publish fleet state and receive tasks via ROS2, or switch high-level logic to Cortex/OmniGraph.

Verification:

- ROS2 extension enabled.
- Topic loop runs while simulation is playing.

---

## 5) Modular Class-Based Reference Implementation

Create a script, for example: `warehouse_modular_sim.py`

```python
"""
Modular multi-robot warehouse simulation using Isaac Sim Python API.

Tested against API style present in local examples under:
- standalone_examples/api/isaacsim.core.api
- standalone_examples/api/isaacsim.robot.wheeled_robots.examples
"""

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Dict, List, Optional, Tuple
import math
import sys

import numpy as np
from isaacsim import SimulationApp


# -----------------------------
# 0) Config + Data Models
# -----------------------------

@dataclass
class SimConfig:
	headless: bool = False
	num_robots: int = 4
	stage_units_in_meters: float = 1.0
	max_steps: int = 2000
	dt_hint: float = 1.0 / 60.0
	waypoint_reached_threshold: float = 0.18
	robot_spacing: float = 1.2
	log_path: str = "./warehouse_swarm_log.json"


@dataclass
class TaskOrder:
	task_id: int
	pick_xy: Tuple[float, float]
	drop_xy: Tuple[float, float]
	assigned_robot: Optional[str] = None
	complete: bool = False


class TaskPhase(Enum):
	IDLE = auto()
	TO_PICK = auto()
	TO_DROP = auto()


@dataclass
class RobotState:
	name: str
	phase: TaskPhase = TaskPhase.IDLE
	active_task_id: Optional[int] = None
	current_goal_xy: Optional[Tuple[float, float]] = None


# -----------------------------
# 1) Bootstrap + World
# -----------------------------

class SimulationBootstrap:
	def __init__(self, cfg: SimConfig):
		self.cfg = cfg
		self.app = SimulationApp({"headless": cfg.headless})

		# Isaac modules are imported after SimulationApp creation
		import carb
		from isaacsim.core.api import World
		from isaacsim.core.utils.prims import create_prim

		self._carb = carb
		self._World = World
		self._create_prim = create_prim

		self.world = self._World(stage_units_in_meters=cfg.stage_units_in_meters)

	def setup_basic_scene(self) -> None:
		self.world.scene.add_default_ground_plane()
		self._create_prim("/DistantLight", "DistantLight")

	def shutdown(self) -> None:
		self.world.stop()
		self.app.close()


class WarehouseSceneBuilder:
	def __init__(self, bootstrap: SimulationBootstrap):
		self.bootstrap = bootstrap
		self.world = bootstrap.world

	def add_visual_waypoint_markers(self, waypoints: List[Tuple[float, float]], z: float = 0.05) -> None:
		from isaacsim.core.api.objects import VisualCuboid

		for i, (x, y) in enumerate(waypoints):
			self.world.scene.add(
				VisualCuboid(
					prim_path=f"/World/Waypoints/WP_{i}",
					name=f"wp_{i}",
					position=np.array([x, y, z]),
					size=0.12,
					color=np.array([255, 215, 0]),
				)
			)


# -----------------------------
# 2) Robot Factory
# -----------------------------

class RobotFactory:
	def __init__(self, bootstrap: SimulationBootstrap):
		self.bootstrap = bootstrap
		self.world = bootstrap.world

		from isaacsim.robot.wheeled_robots.robots import WheeledRobot
		from isaacsim.storage.native import get_assets_root_path

		self.WheeledRobot = WheeledRobot
		self.get_assets_root_path = get_assets_root_path

	def spawn_jetbot_fleet(self, n: int, spacing: float) -> Dict[str, object]:
		assets_root = self.get_assets_root_path()
		if assets_root is None:
			self.bootstrap._carb.log_error("Could not find Isaac Sim assets root path")
			raise RuntimeError("Assets root not available")

		jetbot_usd = assets_root + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
		robots: Dict[str, object] = {}

		for i in range(n):
			x = 0.0
			y = (i - (n - 1) / 2.0) * spacing
			name = f"robot_{i}"
			prim_path = f"/World/Fleet/{name}"
			robots[name] = self.world.scene.add(
				self.WheeledRobot(
					prim_path=prim_path,
					name=name,
					wheel_dof_names=["left_wheel_joint", "right_wheel_joint"],
					create_robot=True,
					usd_path=jetbot_usd,
					position=np.array([x, y, 0.05]),
				)
			)
		return robots


# -----------------------------
# 3) Task Allocation
# -----------------------------

class TaskAllocator:
	def __init__(self, tasks: List[TaskOrder]):
		self.tasks = {t.task_id: t for t in tasks}

	@staticmethod
	def _distance_xy(a: Tuple[float, float], b: Tuple[float, float]) -> float:
		return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

	def assign_if_idle(self, robot_name: str, robot_xy: Tuple[float, float], robot_state: RobotState) -> None:
		if robot_state.phase != TaskPhase.IDLE:
			return

		candidates = [t for t in self.tasks.values() if not t.complete and t.assigned_robot is None]
		if not candidates:
			return

		best = min(candidates, key=lambda t: self._distance_xy(robot_xy, t.pick_xy))
		best.assigned_robot = robot_name

		robot_state.active_task_id = best.task_id
		robot_state.current_goal_xy = best.pick_xy
		robot_state.phase = TaskPhase.TO_PICK

	def advance_phase_if_needed(self, robot_state: RobotState) -> None:
		if robot_state.active_task_id is None:
			return

		task = self.tasks[robot_state.active_task_id]
		if robot_state.phase == TaskPhase.TO_PICK:
			robot_state.phase = TaskPhase.TO_DROP
			robot_state.current_goal_xy = task.drop_xy
		elif robot_state.phase == TaskPhase.TO_DROP:
			task.complete = True
			robot_state.phase = TaskPhase.IDLE
			robot_state.active_task_id = None
			robot_state.current_goal_xy = None


# -----------------------------
# 4) Fleet Control
# -----------------------------

class FleetController:
	def __init__(self, cfg: SimConfig, robots: Dict[str, object], robot_states: Dict[str, RobotState]):
		from isaacsim.robot.wheeled_robots.controllers.differential_controller import DifferentialController

		self.cfg = cfg
		self.robots = robots
		self.robot_states = robot_states
		self.controllers = {
			name: DifferentialController(
				name=f"{name}_controller",
				wheel_radius=0.03,
				wheel_base=0.1125,
			)
			for name in robots
		}

	@staticmethod
	def _yaw_from_quat_wxyz(q: np.ndarray) -> float:
		# Isaac quaternions are usually [w, x, y, z]
		w, x, y, z = q
		siny_cosp = 2.0 * (w * z + x * y)
		cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
		return math.atan2(siny_cosp, cosy_cosp)

	def get_robot_xy(self, robot_name: str) -> Tuple[float, float]:
		pos, _ = self.robots[robot_name].get_world_pose()
		return float(pos[0]), float(pos[1])

	def drive_to_goal(self, robot_name: str, goal_xy: Tuple[float, float]) -> bool:
		robot = self.robots[robot_name]
		controller = self.controllers[robot_name]

		pos, quat = robot.get_world_pose()
		x, y = float(pos[0]), float(pos[1])
		yaw = self._yaw_from_quat_wxyz(quat)

		gx, gy = goal_xy
		dx, dy = gx - x, gy - y
		dist = math.sqrt(dx * dx + dy * dy)

		if dist < self.cfg.waypoint_reached_threshold:
			robot.apply_wheel_actions(controller.forward(command=[0.0, 0.0]))
			return True

		target_heading = math.atan2(dy, dx)
		heading_error = math.atan2(math.sin(target_heading - yaw), math.cos(target_heading - yaw))

		lin = np.clip(0.45 * dist, 0.02, 0.20)
		ang = np.clip(1.6 * heading_error, -1.0, 1.0)

		robot.apply_wheel_actions(controller.forward(command=[lin, ang]))
		return False


# -----------------------------
# 5) Perception + Logging
# -----------------------------

class PerceptionManager:
	def __init__(self, world):
		self.world = world
		self.camera = None

	def add_overhead_camera(self) -> None:
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

	def get_latest_rgba(self):
		if self.camera is None:
			return None
		return self.camera.get_rgba()


class MetricsLogger:
	def __init__(self, world, robot_states: Dict[str, RobotState], robots: Dict[str, object]):
		self.world = world
		self.robot_states = robot_states
		self.robots = robots
		self.logger = world.get_data_logger()

	def start(self):
		def _frame_fn(tasks, scene):
			frame = {}
			for name, robot in self.robots.items():
				pos, _ = robot.get_world_pose()
				st = self.robot_states[name]
				frame[name] = {
					"x": float(pos[0]),
					"y": float(pos[1]),
					"phase": st.phase.name,
					"active_task_id": st.active_task_id,
				}
			return frame

		self.logger.add_data_frame_logging_func(_frame_fn)
		self.logger.start()

	def save(self, path: str):
		self.logger.save(log_path=path)


# -----------------------------
# 6) Runner
# -----------------------------

class WarehouseSimulationRunner:
	def __init__(self, cfg: SimConfig):
		self.cfg = cfg
		self.bootstrap = SimulationBootstrap(cfg)
		self.bootstrap.setup_basic_scene()

		self.scene_builder = WarehouseSceneBuilder(self.bootstrap)
		self.robot_factory = RobotFactory(self.bootstrap)

		self.robots = self.robot_factory.spawn_jetbot_fleet(n=cfg.num_robots, spacing=cfg.robot_spacing)
		self.robot_states = {name: RobotState(name=name) for name in self.robots.keys()}

		self.tasks = [
			TaskOrder(task_id=1, pick_xy=(2.5, -1.0), drop_xy=(-2.5, -1.2)),
			TaskOrder(task_id=2, pick_xy=(2.5, 1.0), drop_xy=(-2.5, 1.2)),
			TaskOrder(task_id=3, pick_xy=(1.0, 2.2), drop_xy=(-1.0, -2.2)),
			TaskOrder(task_id=4, pick_xy=(0.0, -2.4), drop_xy=(0.0, 2.4)),
		]

		all_waypoints = [t.pick_xy for t in self.tasks] + [t.drop_xy for t in self.tasks]
		self.scene_builder.add_visual_waypoint_markers(all_waypoints)

		self.allocator = TaskAllocator(self.tasks)
		self.fleet_controller = FleetController(cfg, self.robots, self.robot_states)
		self.perception = PerceptionManager(self.bootstrap.world)
		self.metrics = MetricsLogger(self.bootstrap.world, self.robot_states, self.robots)

	def run(self) -> None:
		world = self.bootstrap.world
		world.reset()

		self.perception.add_overhead_camera()
		self.metrics.start()

		for step in range(self.cfg.max_steps):
			world.step(render=True)

			for robot_name, st in self.robot_states.items():
				xy = self.fleet_controller.get_robot_xy(robot_name)
				self.allocator.assign_if_idle(robot_name, xy, st)

				if st.current_goal_xy is not None:
					reached = self.fleet_controller.drive_to_goal(robot_name, st.current_goal_xy)
					if reached:
						self.allocator.advance_phase_if_needed(st)

			if all(t.complete for t in self.tasks):
				print(f"All tasks complete at step={step}")
				break

		self.metrics.save(self.cfg.log_path)
		print(f"Saved metrics to {self.cfg.log_path}")

	def close(self) -> None:
		self.bootstrap.shutdown()


def main():
	cfg = SimConfig(headless=False, num_robots=4, max_steps=2500)
	runner = WarehouseSimulationRunner(cfg)
	try:
		runner.run()
	except Exception as exc:
		print(f"Simulation error: {exc}")
		raise
	finally:
		runner.close()


if __name__ == "__main__":
	main()
```

---

## 6) How To Run (Stepwise)

From Isaac Sim root:

```bash
cd /home/ubuntu/isaacsim
./python.sh /path/to/warehouse_modular_sim.py
```

For each step, comment out advanced modules until the current stage passes:

1. Run only bootstrap + ground + stepping loop.
2. Add robot spawn and verify all agents appear.
3. Add differential controller and single waypoint chase.
4. Add allocator and task state transitions.
5. Add data logger save/load inspection.
6. Add camera and verify image frames are available.

---

## 7) Expected Outputs For Verification

- Console prints:
  - task completion progress
  - final "All tasks complete" message
  - log save path
- Visual:
  - robots move from spawn to pick markers, then to drop markers
- Artifact:
  - `warehouse_swarm_log.json` with per-frame robot state

---

## 8) Mapping to Your Research Objectives

This modular solution directly maps your combined problem statement to implementation:

- Swarm task management: `TaskAllocator`
- Environment and scene composition: `WarehouseSceneBuilder`
- Mobile control and execution: `FleetController`
- Perception hooks (OCR-ready): `PerceptionManager`
- Performance/KPI and experiments: `MetricsLogger`
- End-to-end lifecycle orchestration: `WarehouseSimulationRunner`

---

## 9) Extension Path (After MVP Works)

1. Replace Jetbot with Carter/Nova Carter for richer warehouse navigation primitives.
2. Add manipulator stage at dock zones (Franka/UR10) for true mobile-manipulation workflow.
3. Enable ROS2 bridge (`enable_extension("isaacsim.ros2.bridge")`) and publish robot/task topics.
4. Add OmniGraph/Cortex high-level behavior network for decider logic.
5. Integrate cuOpt or external planner for global fleet optimization.
6. Add synthetic data + OCR pipeline for label recognition and task validation.

This sequence keeps complexity controlled while preserving your long-term autonomy stack goals.
