#!/usr/bin/env python3
"""Load the saved PGM in a real Nav2 map_server and compare all cells."""
import argparse, signal, subprocess, time
from pathlib import Path
import numpy as np
import rclpy
from lifecycle_msgs.srv import ChangeState
from nav_msgs.srv import GetMap
from fleet_mapping.grid import load_map

p = argparse.ArgumentParser()
p.add_argument("map")
a = p.parse_args()
path = Path(a.map).resolve()
expected, _ = load_map(path)
log = open(path.parent / "map_server_roundtrip.log", "w")
proc = subprocess.Popen(
    [
        "ros2",
        "run",
        "nav2_map_server",
        "map_server",
        "--ros-args",
        "-r",
        "__ns:=/fleet_map_audit",
        "-p",
        f"yaml_filename:={path}",
        "-p",
        "use_sim_time:=false",
    ],
    stdout=log,
    stderr=subprocess.STDOUT,
    start_new_session=True,
)
rclpy.init()
n = rclpy.create_node("map_roundtrip_probe")


def call(typ, service, req):
    c = n.create_client(typ, service)
    if not c.wait_for_service(timeout_sec=10):
        raise RuntimeError("Map server service unavailable")
    f = c.call_async(req)
    rclpy.spin_until_future_complete(n, f, timeout_sec=10)
    if not f.done():
        raise RuntimeError("Map server timed out")
    result = f.result()
    n.destroy_client(c)
    return result


try:
    req = ChangeState.Request()
    req.transition.id = 1
    assert call(ChangeState, "/fleet_map_audit/map_server/change_state", req).success
    req.transition.id = 3
    assert call(ChangeState, "/fleet_map_audit/map_server/change_state", req).success
    m = call(GetMap, "/fleet_map_audit/map_server/map", GetMap.Request()).map
    actual = np.asarray(m.data, dtype=np.int8).reshape(m.info.height, m.info.width)
    assert np.array_equal(actual, expected), "Nav2 map_server 栅格不一致"
    report = {
        "passed": True,
        "cells": actual.size,
        "unknown": int((actual < 0).sum()),
        "free": int((actual == 0).sum()),
        "occupied": int((actual == 100).sum()),
    }
    import json

    (path.parent / "map_roundtrip.json").write_text(json.dumps(report, indent=2))
    print(report)
finally:
    import os

    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)
        proc.wait()
    log.close()
    n.destroy_node()
    rclpy.try_shutdown()
