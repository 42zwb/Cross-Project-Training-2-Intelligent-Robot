"""Bind acceptance requests to one invocation, including custom dashboard ports."""

import json
import os
from pathlib import Path
import urllib.request
from urllib.parse import quote
import yaml


class RuntimeClient:
    def __init__(self, run, base_url=None):
        self.run = Path(run).resolve()
        self.override_url = base_url

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
