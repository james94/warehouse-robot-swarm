import numpy as np

class MobileRobotFactory:
    def __init__(self, world, assets_root_path: str):
        self.world = world
        self.assets_root_path = assets_root_path

    def spawn_jetbots(self, count: int, spacing: float = 1.0) -> dict:
        from isaacsim.robot.wheeled_robots.robots import WheeledRobot
        # Resolve USD path for the Jetbot asset. If an assets root path
        # was supplied, append the relative USD location, otherwise fall
        # back to the known USD path inside Isaac assets.
        if self.assets_root_path:
            assets_root = str(self.assets_root_path).rstrip("/")
            if assets_root.endswith(".usd"):
                usd = assets_root
            else:
                usd = assets_root + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
        else:
            usd = "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd"
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
        