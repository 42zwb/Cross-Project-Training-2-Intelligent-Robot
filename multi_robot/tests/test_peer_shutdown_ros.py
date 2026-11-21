"""Signals must close real peer processes; active runtime faults stay failures."""
import os
from pathlib import Path
import select
import signal
import subprocess
import sys
import time
import pytest
import yaml


@pytest.fixture
def config(tmp_path):
    p = tmp_path/'agent.yaml'
    p.write_text(yaml.safe_dump(dict(architecture='decentralized', robot_id='robot1',
        members=['robot1', 'robot2', 'robot3'], session_id='shutdown-test', boot_id='b1',
        map_epoch=1, resolution=.05, robot_radius=.18, max_speed=.2, region_margin=.35,
        run_dir=str(tmp_path), dashboard_port=8765, assets=str(tmp_path))))
    return p


@pytest.mark.parametrize('module,cls', [
    ('local_mapper', 'LocalMapper'), ('local_safety', 'LocalSafety'), ('peer_agent', 'PeerAgent')])
@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM])
def test_peer_main_closes_normally_on_ros_shutdown(config, module, cls, stop_signal):
    script = '''import importlib
m = importlib.import_module('fleet_control.'+__import__('sys').argv[1])
c = getattr(m, __import__('sys').argv[2])
original = c.__init__
def initialized(self, *a, **kw):
    original(self, *a, **kw)
    print('PEER_READY', flush=True)
c.__init__ = initialized
del __import__('sys').argv[1:3]
m.main()
'''
    env = dict(os.environ, ROS_DOMAIN_ID='225')
    child = subprocess.Popen([sys.executable, '-u', '-c', script, module, cls,
        '--ros-args', '-r', '__ns:=/robot1', '-p', 'config:='+str(config)],
        env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    output = ''
    try:
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            if select.select([child.stdout], [], [], .1)[0]:
                line = child.stdout.readline()
                output += line
                if 'PEER_READY' in line:
                    break
            assert child.poll() is None, output
        else:
            pytest.fail('peer startup timeout: '+output)
        time.sleep(.2)
        child.send_signal(stop_signal)
        tail, _ = child.communicate(timeout=6)
        output += tail
        assert child.returncode == 0, output
        assert 'Traceback' not in output
    finally:
        if child.poll() is None:
            child.kill()
            child.wait()
        child.stdout.close()


def test_active_runtime_error_is_not_hidden_as_normal_shutdown(config):
    script = '''from fleet_control import local_safety as m
def fail(self):
    raise RuntimeError('injected active safety failure')
m.LocalSafety.tick = fail
m.main()
'''
    result = subprocess.run([sys.executable, '-c', script, '--ros-args',
        '-r', '__ns:=/robot1', '-p', 'config:='+str(config)],
        env=dict(os.environ, ROS_DOMAIN_ID='225'), capture_output=True, text=True, timeout=10)
    assert result.returncode != 0
    assert 'injected active safety failure' in result.stderr
