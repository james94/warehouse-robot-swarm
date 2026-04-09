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
        