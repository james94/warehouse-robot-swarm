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
        