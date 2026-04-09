class KPITracker:
    def __init__(self):
        self.completed = 0
        self.total = 0

    def set_total(self, total: int) -> None:
        self.total = total
    
    def mark_completed(self) -> None:
        self.completed += 1
    
    def summary(self) -> dict:
        rate = 0.0 if self.total == 0 else self.completed / self.total
        return {"completed": self.completed, "total": self.total, "completion_rate": rate}
        