class RobotRegistry:
    def __init__(self):
        self._robots = {}
    
    def set(self, robots: dict) -> None:
        self._robots = dict(robots)
    
    def get(self, name: str):
        return self._robots[name]
    
    def names(self) -> list[str]:
        return list(self._robots.keys())
