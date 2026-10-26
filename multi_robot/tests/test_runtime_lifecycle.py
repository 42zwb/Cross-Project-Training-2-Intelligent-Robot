import sys
import os
import subprocess
from pathlib import Path
from types import SimpleNamespace
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from runtime_context import RuntimeClient
import runtime_context
import shutdown_context

ROOT=Path(__file__).resolve().parents[1]
BRIDGE=ROOT/"install/fleet_bridge/lib/fleet_bridge/parameter_bridge"


def test_wait_ready_retries_initial_identity_without_sending_commands(monkeypatch):
    client = RuntimeClient("/tmp/new-run")
    replies = iter([RuntimeError("no current identity"), {"ready":False,"mode":"BOOTSTRAP"},
                    {"ready":True,"mode":"IDLE"}])
    def state(**kwargs):
        value = next(replies)
        if isinstance(value,Exception):raise value
        return value
    monkeypatch.setattr(client,"state",state)
    monkeypatch.setattr(runtime_context.time,"sleep",lambda seconds:None)
    assert client.wait_ready(1)["ready"]


def test_wait_ready_reports_actual_fault_immediately(monkeypatch):
    client=RuntimeClient("/tmp/new-run")
    monkeypatch.setattr(client,"state",lambda **kw:{"ready":False,"mode":"FAULT","message":"雷达故障"})
    with pytest.raises(RuntimeError,match="雷达故障"):
        client.wait_ready(1)


def test_wait_ready_times_out_for_foreign_backend_without_sending(monkeypatch):
    client=RuntimeClient("/tmp/new-run")
    now=[0.0]
    monkeypatch.setattr(runtime_context.time,"monotonic",lambda:now[0])
    monkeypatch.setattr(runtime_context.time,"sleep",lambda amount:now.__setitem__(0,now[0]+amount))
    def foreign(**kwargs):raise RuntimeError("foreign backend")
    monkeypatch.setattr(client,"state",foreign)
    monkeypatch.setattr(client,"request",lambda *a,**kw:pytest.fail("must not send a command"))
    with pytest.raises(TimeoutError,match="foreign backend"):
        client.wait_ready(1)


def test_state_rejects_non_object_json(monkeypatch):
    import io
    monkeypatch.setattr(runtime_context.urllib.request,"urlopen",lambda *a,**kw:io.BytesIO(b"[]"))
    with pytest.raises(ValueError,match="JSON"):
        RuntimeClient("/tmp/new-run").state()


def test_reproduction_command_keeps_config_and_quotes_shell_characters():
    import shlex
    from runtime_context import reproduction_command
    path="/tmp/中文 空格/$(touch should_not_run)'quoted.yaml"
    assert shlex.split(reproduction_command(path,0))==["./run.sh","--config",path,"--seed","0"]


def test_shutdown_cannot_claim_success_if_no_owned_simulator_is_found(monkeypatch):
    original_glob = Path.glob
    monkeypatch.setattr(Path,"glob",lambda self, pattern: [] if str(self)=="/proc" else original_glob(self,pattern))
    result=shutdown_context.quiesce(Path("/tmp/run"),process_group=123,base_only=True)
    assert not result["simulator_stopped"]
    assert result["simulator_pids"]==[]


@pytest.mark.skipif(not BRIDGE.exists(),reason="fleet_bridge must be built")
@pytest.mark.parametrize("spec", [
    "bad", "@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
    "/probe@rosgraph_msgs/msg/Clock", "/probe@rosgraph_msgs/msg/Clock[",
    "/probe@[ignition.msgs.Clock", "/probe@rosgraph_msgs/msg/Clock@",
    "/probe@unknown_msgs/msg/Unknown[ignition.msgs.Clock",
    "/probe@unknown_msgs/srv/Unknown",
])
def test_native_bridge_rejects_invalid_specs_without_hanging_or_aborting(spec):
    env=dict(os.environ,ROS_DOMAIN_ID="229",IGN_PARTITION="fleet_bridge_invalid_spec")
    result=subprocess.run([str(BRIDGE),spec],capture_output=True,text=True,env=env,timeout=5)
    assert result.returncode>0
    assert "corrupted" not in result.stderr
