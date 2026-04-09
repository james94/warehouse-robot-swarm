from warehouse_sim.pipelines.part1_world_pipeline import run_part1
from warehouse_sim.pipelines.part2_robot_pipeline import run_part2

def run_full(config_path: str, headless: bool, mode: str, max_steps: int, seed: int) -> None:
    if mode == "only_scene":
        run_part1(config_path=config_path, headless=headless, max_steps=max_steps, seed=seed)
        return

    if mode in ["scene_plus_robots", "with_ros2", "with_cortex_hooks"]:
        run_part2(config_path=config_path, headless=headless, max_steps=max_steps, seed=seed)
        return

    raise ValueError(f"Unsupported mode: {mode}")
    