#!/usr/bin/env python3
"""Check the loaded plugin, three real planning requests and fleet diagnostics."""

import argparse
import json
import time
from pathlib import Path

import rclpy
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from diagnostic_msgs.msg import DiagnosticArray
from rcl_interfaces.srv import GetParameters
import yaml
from runtime_context import RuntimeClient


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir")
    args = parser.parse_args()
    run = Path(args.run_dir or (root / "runs/latest.txt").read_text()).resolve()
    api = RuntimeClient(run)
    assert api.state()["ready"]
    api.verify_ros_domain()
    cfg = yaml.safe_load((run / "assets/fleet.yaml").read_text())
    rclpy.init()
    node = rclpy.create_node("fleet_planner_acceptance")
    node.set_parameters([rclpy.parameter.Parameter("use_sim_time", value=True)])
    diagnostics = {}
    def on_diagnostics(message):
        for status in message.status:
            if status.name.endswith("/fleet_health"):
                level = status.level[0] if isinstance(status.level, bytes) else status.level
                diagnostics[status.name] = {"level": level, "message": status.message}
    node.create_subscription(DiagnosticArray, "/diagnostics", on_diagnostics, 10)
    def wait(future, timeout=10):
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        if not future.done():
            raise TimeoutError("规划服务或动作响应超时")
        return future.result()
    report = {"planner": cfg["planner"], "robots": {}}
    try:
        for i, robot in enumerate(cfg["robots"]):
            name = robot["name"]
            client = node.create_client(GetParameters, f"/{name}/planner_server/get_parameters")
            assert client.wait_for_service(timeout_sec=10)
            plugin = wait(client.call_async(GetParameters.Request(names=["GridBased.plugin"]))).values[0].string_value
            expected = "nav2_smac_planner/SmacPlanner2D" if cfg["planner"] == "smac2d" else "nav2_navfn_planner/NavfnPlanner"
            assert plugin == expected, plugin
            node.destroy_client(client)
            action = ActionClient(node, ComputePathToPose, f"/{name}/compute_path_to_pose")
            assert action.wait_for_server(timeout_sec=10)
            request = ComputePathToPose.Goal()
            request.goal.header.frame_id = "map"
            request.goal.pose.position.x = robot["start"][0] + (0.6 if i == 2 else -0.6)
            request.goal.pose.position.y = robot["start"][1]
            request.goal.pose.orientation.w = 1.0
            request.planner_id = "GridBased"
            request.use_start = False
            handle = wait(action.send_goal_async(request))
            assert handle.accepted
            result = wait(handle.get_result_async())
            assert result.status == 4 and result.result.path.poses
            duration = result.result.planning_time
            report["robots"][name] = {"plugin": plugin, "status": result.status,
                "path_poses": len(result.result.path.poses), "planning_seconds": duration.sec + duration.nanosec * 1e-9}
            action.destroy()
        deadline = time.monotonic() + 5
        while len(diagnostics) < 3 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert len(diagnostics) == 3 and all(s["level"] == 0 for s in diagnostics.values()), diagnostics
        report["diagnostics"] = diagnostics
        report["passed"] = True
        (run / "planner_acceptance.json").write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False, indent=2), flush=True)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
