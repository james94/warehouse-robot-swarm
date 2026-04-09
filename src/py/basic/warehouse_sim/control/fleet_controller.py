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
        