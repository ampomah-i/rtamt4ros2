from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]


def test_launch_files_are_valid_python():
    for launch_file in (ROOT / "launch").glob("*.launch.py"):
        compile(launch_file.read_text(encoding="utf-8"), str(launch_file), "exec")


def test_default_config_targets_launch_node_and_has_complete_mapping():
    config = yaml.safe_load((ROOT / "config" / "monitor.yaml").read_text())
    parameters = config["stl_monitor"]["ros__parameters"]

    assert len(parameters["vars"]) == len(parameters["input_topics"])
    assert parameters["sampling_period"] > 0
    assert parameters["formula"].startswith("out =")
