"""Bind acceptance requests to one invocation, including custom dashboard ports."""

import json
import os
import time
import shlex
from pathlib import Path
import urllib.request
from urllib.parse import quote
import yaml


def reproduction_command(config, seed):
    return shlex.join(["./run.sh", "--config", str(config), "--seed", str(seed)])


class RuntimeClient:
    def __init__(self, run, base_url=None):
        self.run = Path(run).resolve()
        self.override_url = base_url

    def wait_ready(self, timeout=180.0):
        """Wait for this invocation without ever sending to a foreign backend."""
        end = time.monotonic() + timeout
        last_error = "尚未收到本轮状态"
        while time.monotonic() < end:
            try:
                state = self.state(timeout=min(2.0, max(0.1, end - time.monotonic())))
            except (OSError, ValueError, RuntimeError) as error:
                # HTTP can come up before the first coordinator state; its
                # empty run identity must not be mistaken for a ready backend.
                last_error = str(error)
            else:
                if state.get("mode") in ["FAULT", "ESTOP"]:
                    raise RuntimeError(state.get("message", "系统已故障或急停"))
                if state.get("ready"):
                    return state
                last_error = state.get("message", "本轮尚未就绪")
            time.sleep(min(0.5, max(0.0, end - time.monotonic())))
        raise TimeoutError("等待本轮系统就绪超时：" + last_error)

    @property
    def base_url(self):
        if self.override_url:
            return self.override_url.rstrip("/")
        config = self.run / "assets/fleet.yaml"
        if not config.exists():
            config = self.run / "requested_fleet.yaml"
        port = yaml.safe_load(config.read_text())["dashboard_port"] if config.exists() else 8765
        return f"http://127.0.0.1:{port}"

    def state(self, timeout=5):
        with urllib.request.urlopen(self.base_url + "/api/state", timeout=timeout) as response:
            result = json.load(response)
        if not isinstance(result, dict):
            raise ValueError("网页状态须为 JSON 对象")
        if result.get("run_dir") != str(self.run):
            raise RuntimeError("网页后端不属于本次运行目录，拒绝接管其他实例")
        return result

    def request(self, kind, data, timeout=20):
        request = urllib.request.Request(self.base_url + "/api/" + kind,
            json.dumps(data, allow_nan=False).encode(),
            {"Content-Type":"application/json", "X-Fleet-Run":quote(str(self.run))}, method="POST")
        with urllib.request.urlopen(request, timeout=timeout) as response:
            return json.load(response)

    def verify_ros_domain(self):
        manifest = json.loads((self.run / "manifest.json").read_text())
        expected = manifest["environment"]["ROS_DOMAIN_ID"]
        if os.environ.get("ROS_DOMAIN_ID", "0") != expected:
            raise RuntimeError(f"ROS 域与本次运行不符：应为 {expected}，请先加载相同的 env.sh 环境")
