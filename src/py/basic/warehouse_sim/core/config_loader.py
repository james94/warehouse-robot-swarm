from pathlib import Path
import yaml

class ConfigLoader:
    @staticmethod
    def load_yaml(path: str) -> dict:
        p = Path(path)

        if not p.exists():
            raise FileNotFoundError(f"Config not found: {path}")
        
        with p.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    
    @staticmethod
    def validate_required(cfg: dict, required_keys: list[str]) -> None:
        missing = [k for k in required_keys if k not in cfg]

        if missing:
            raise ValueError(f"Missing config keys: {missing}")
