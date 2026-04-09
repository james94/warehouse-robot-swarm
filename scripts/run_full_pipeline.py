import argparse
import sys
from pathlib import Path


def _ensure_project_import_path() -> None:
    """Allow running this script from any CWD (e.g., ~/isaacsim)."""
    repo_root = Path(__file__).resolve().parents[1]
    package_root = repo_root / "src" / "py" / "basic"
    package_root_str = str(package_root)
    if package_root_str not in sys.path:
        sys.path.insert(0, package_root_str)


_ensure_project_import_path()

from warehouse_sim.pipelines.full_pipeline import run_full

def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", default="configs/sim_config.yaml")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--mode", choices=["only_scene", "scene_plus_robots", "with_ros2", "with_cortex_hooks"], default="scene_plus_robots")
    parser.add_argument("--steps", type=int, default=2500)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()

    run_full(config_path=args.config, headless=args.headless, mode=args.mode, max_steps=args.steps, seed=args.seed)

if __name__ == "__main__":
    main()
