class BoxTransferManager:
    def __init__(self):
        self.box_owner = {}

    def mark_picked(self, box_id: str, robot_name: str) -> None:
        self.box_owner[box_id] = robot_name

    def mark_placed_on_conveyor(self, box_id: str) -> None:
        self.box_owner[box_id] = "conveyor"
