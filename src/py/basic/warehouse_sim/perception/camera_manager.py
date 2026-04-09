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
