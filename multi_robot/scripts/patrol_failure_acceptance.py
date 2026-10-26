#!/usr/bin/env python3
"""Exercise a real patrol timeout and prove that every simulated robot stops."""

import argparse
import json
import math
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid
import yaml
from fleet_control.geometry import Grid
from runtime_context import RuntimeClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    run = Path(args.run_dir or (root / "runs/latest.txt").read_text()).resolve()
    api = RuntimeClient(run)
    api.verify_ros_domain()
    config = yaml.safe_load((run / "assets/fleet.yaml").read_text())
    rclpy.init()
    node = rclpy.create_node("patrol_timeout_acceptance")
    maps = []
    node.create_subscription(OccupancyGrid, "/map", maps.append,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    deadline = time.monotonic() + 5
    while not maps and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.try_shutdown()
    assert maps
    message = maps[-1]
    grid = Grid(np.asarray(message.data).reshape(message.info.height, message.info.width),
        message.info.resolution, (message.info.origin.position.x, message.info.origin.position.y),
        config["robot_radius"], config["region_margin"])
    state = api.state()
    assert state["ready"] and state["mode"] in ["IDLE", "PATROL_DONE"]
    name = "robot1"
    pose = state["robots"][name]["pose"]
    allowed = grid.allowed(state["regions"][0])
    candidates = [grid.point(c) for c in np.argwhere(allowed)[::30]]
    candidates = sorted((p for p in candidates if math.dist(p, pose[:2]) > 2.0), key=lambda p: math.dist(p, pose[:2]))
    target = next((p for p in candidates if grid.astar(pose, p, allowed)), None)
    assert target is not None, "当前观测地图没有足够长的可达测试路线"
    report = {"target": target, "goal_timeout": 5.0}
    def truth():
        return json.loads((run / "truth.jsonl").read_text().splitlines()[-1])["robots"]
    try:
        plan = {"routes": {name: [[*target, 0.0]]}, "goal_timeout": 5.0}
        result = api.request("command", {"command":"patrol", "argument":json.dumps(plan)})
        assert result["success"], result
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            state = api.state()
            if state["mode"] == "FAULT":
                break
            time.sleep(0.1)
        assert state["mode"] == "FAULT" and "超时" in state["message"], state["message"]
        assert state["patrol"]["status"] == "failed"
        time.sleep(1.0)
        before = truth()
        time.sleep(1.0)
        after = truth()
        displacement = {n: math.dist(p[:2], after[n][:2]) for n, p in before.items()}
        assert max(displacement.values()) < 0.025, displacement
        assert not any(r["safety"].get("moving_allowed") for r in api.state()["robots"].values())
        report.update(passed=True, message=state["message"], stop_displacement=displacement)
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    finally:
        api.request("command", {"command":"pause"})
        (run / "patrol_failure_acceptance.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
