class Ros2BridgeManager:
    def enable(self) -> None:
        from isaacsim.core.utils.extensions import enable_extension

        enable_extension("isaacsim.ros2.bridge")
        