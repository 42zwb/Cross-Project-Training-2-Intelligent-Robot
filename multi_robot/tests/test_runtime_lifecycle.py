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


def test_supervisor_reaps_launch_when_post_spawn_initialization_fails(tmp_path):
    import shutil
    import time
    project = tmp_path / 'project'
    scripts = project / 'scripts'
    scripts.mkdir(parents=True)
    for name in ['supervise.py', 'shutdown_context.py', 'runtime_context.py']:
        shutil.copy2(ROOT / 'scripts' / name, scripts / name)
    for name in ['env.sh', 'build.sh', 'run.sh', 'demo.sh']:
        shutil.copy2(ROOT / name, project / name)
    # Make latest.txt unwritable as a file after Popen has succeeded.
    (project / 'runs' / 'latest.txt').mkdir(parents=True)
    bin_dir = tmp_path / 'bin'
    bin_dir.mkdir()
    ros = bin_dir / 'ros2'
    ros.write_text('#!/usr/bin/python3\nimport time\ntime.sleep(60)\n')
    ros.chmod(0o755)
    pid_file = tmp_path / 'child.pid'
    hook = tmp_path / 'hook'
    hook.mkdir()
    (hook / 'sitecustomize.py').write_text('''import subprocess, os
original = subprocess.Popen.__init__
def wrapped(self, args, *a, **kw):
    original(self, args, *a, **kw)
    if isinstance(args, list) and args and args[0] == "ros2":
        open(os.environ["TEST_LAUNCH_PID"], "w").write(str(self.pid))
subprocess.Popen.__init__ = wrapped
''')
    env = dict(os.environ, PATH=str(bin_dir) + os.pathsep + os.environ['PATH'],
        PYTHONPATH=str(hook) + os.pathsep + os.environ.get('PYTHONPATH',''),
        ROS_DOMAIN_ID='226', IGN_PARTITION='fleet-supervisor-failure-test-226',
        TEST_LAUNCH_PID=str(pid_file))
    result = subprocess.run([sys.executable, str(scripts/'supervise.py'), '--base-only',
        '--run-dir', str(tmp_path/'external'), '--config', str(ROOT/'src/fleet_bringup/config/fleet.yaml'),
        '--seed', '24'], env=env, capture_output=True, text=True, timeout=30)
    assert result.returncode != 0 and 'IsADirectoryError' in result.stderr
    pid = int(pid_file.read_text())
    try:
        for _ in range(50):
            if not Path(f'/proc/{pid}').exists(): break
            time.sleep(0.02)
        assert not Path(f'/proc/{pid}').exists()
    finally:
        try: os.killpg(pid, 9)
        except ProcessLookupError: pass


def test_supervisor_creates_latest_parent_for_external_run(tmp_path):
    import shutil
    project = tmp_path/'project'
    scripts = project/'scripts'
    scripts.mkdir(parents=True)
    for name in ['supervise.py', 'shutdown_context.py', 'runtime_context.py']:
        shutil.copy2(ROOT/'scripts'/name, scripts/name)
    for name in ['env.sh', 'build.sh', 'run.sh', 'demo.sh']:
        shutil.copy2(ROOT/name, project/name)
    bin_dir = tmp_path/'bin'
    bin_dir.mkdir()
    ros = bin_dir/'ros2'
    ros.write_text('#!/usr/bin/python3\nimport time\ntime.sleep(60)\n')
    ros.chmod(0o755)
    env = dict(os.environ, PATH=str(bin_dir)+os.pathsep+os.environ['PATH'],
               ROS_DOMAIN_ID='227', IGN_PARTITION='fleet-supervisor-new-runs-test-227')
    run = tmp_path/'external'
    result = subprocess.run([sys.executable, str(scripts/'supervise.py'), '--base-only',
        '--run-dir', str(run), '--config', str(ROOT/'src/fleet_bringup/config/fleet.yaml'),
        '--seed', '24'], env=env, capture_output=True, text=True, timeout=30)
    # This fixture intentionally omits the resource sampler, which triggers cleanup.
    assert result.returncode != 0
    assert (project/'runs/latest.txt').read_text() == str(run)
    assert (run/'cleanup.json').exists()


def test_supervisor_starts_peer_recording_only_after_metadata_is_published(tmp_path):
    import json
    import shutil
    import time

    project = tmp_path/'project'
    scripts = project/'scripts'
    scripts.mkdir(parents=True)
    for filename in ('supervise.py', 'shutdown_context.py', 'runtime_context.py'):
        shutil.copy2(ROOT/'scripts'/filename, scripts/filename)
    for filename in ('env.sh', 'build.sh', 'run.sh', 'demo.sh'):
        shutil.copy2(ROOT/filename, project/filename)

    (scripts/'resources.py').write_text('import time\ntime.sleep(60)\n')
    (scripts/'peer_recorder.py').write_text('''import json
from pathlib import Path
import sys
run = Path(sys.argv[1])
with (run/'recorder-calls').open('a') as calls:
    calls.write('started\\n')
assert json.loads((run/'architecture.json').read_text())['session_id'] == 'delayed'
raise SystemExit(7)
''')
    (project/'tools').mkdir()
    (project/'tools/truth_probe.cpp').touch()
    probe = project/'build/tools/truth_probe'
    probe.parent.mkdir(parents=True)
    probe.write_text('''#!/usr/bin/python3
from pathlib import Path
import sys
import time
Path(sys.argv[1]).touch()
time.sleep(60)
''')
    probe.chmod(0o755)
    ros = tmp_path/'ros2'
    ros.write_text('''#!/usr/bin/python3
from pathlib import Path
import sys
import time
assets = next(arg.split(':=', 1)[1] for arg in sys.argv if arg.startswith('assets:='))
run = Path(assets).parent
while not (run/'publish-metadata').exists():
    time.sleep(.02)
metadata = run/'architecture.json'
temporary = metadata.with_suffix('.json.tmp')
temporary.write_text('{"architecture":"decentralized","session_id":"delayed"}')
temporary.replace(metadata)
time.sleep(60)
''')
    ros.chmod(0o755)

    env = dict(os.environ, PATH=str(tmp_path)+os.pathsep+os.environ['PATH'],
               ROS_DOMAIN_ID='228', IGN_PARTITION='fleet-supervisor-delayed-metadata-test-228')
    run = tmp_path/'external'
    with subprocess.Popen([sys.executable, str(scripts/'supervise.py'), '--base-only',
                           '--record-truth', '--run-dir', str(run),
                           '--config', str(ROOT/'src/fleet_bringup/config/fleet.yaml')],
                          env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True) as supervisor:
        try:
            deadline = time.monotonic()+15
            while not (run/'truth.jsonl').exists() and supervisor.poll() is None and time.monotonic()<deadline:
                time.sleep(.02)
            assert (run/'truth.jsonl').is_file()
            assert supervisor.poll() is None
            assert not (run/'architecture.json').exists()
            assert not (run/'recorder-calls').exists()
            (run/'publish-metadata').touch()
            output, _ = supervisor.communicate(timeout=20)
            assert supervisor.returncode != 0, output
        finally:
            if supervisor.poll() is None:
                supervisor.terminate()
                supervisor.communicate(timeout=15)

    assert (run/'recorder-calls').read_text().splitlines() == ['started']
    cleanup = json.loads((run/'cleanup.json').read_text())
    assert cleanup['auxiliary_error'] == {'process': 'peer_recorder', 'returncode': 7}
    assert cleanup['residual_before_final_kill'] == []
