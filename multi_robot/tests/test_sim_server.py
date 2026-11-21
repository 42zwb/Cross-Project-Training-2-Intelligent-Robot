"""Exercise the real native host's signals, thread join and invalid-world exit."""
import os
from pathlib import Path
import select
import signal
import subprocess
import time
import pytest


SERVER = Path(__file__).resolve().parents[1]/'install/fleet_bridge/lib/fleet_bridge/fleet_sim_server'


@pytest.mark.skipif(not SERVER.exists(), reason='build fleet_bridge first')
@pytest.mark.parametrize('stop_signal', [signal.SIGINT, signal.SIGTERM])
def test_native_host_releases_world_on_one_signal(tmp_path, stop_signal):
    world = tmp_path/'world.sdf'
    world.write_text('<sdf version="1.7"><world name="host_lifecycle">'
        '<plugin filename="ignition-gazebo-physics-system" '
        'name="ignition::gazebo::systems::Physics"/></world></sdf>')
    env = dict(os.environ, IGN_PARTITION='fleet-host-test-'+str(os.getpid()),
               GZ_PARTITION='fleet-host-test-'+str(os.getpid()), ROS_DOMAIN_ID='228')
    child = subprocess.Popen([str(SERVER), str(world)], env=env,
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, start_new_session=True)
    output = ''
    try:
        end = time.monotonic()+8
        while time.monotonic() < end:
            if select.select([child.stdout], [], [], .1)[0]:
                line = child.stdout.readline()
                output += line
                if 'fleet_sim_server: running' in line:
                    break
            assert child.poll() is None, output
        else:
            pytest.fail('native host readiness timeout: '+output)
        child.send_signal(stop_signal)
        tail, _ = child.communicate(timeout=8)
        output += tail
        assert child.returncode == 0, output
        assert 'fleet_sim_server: world released' in output
        assert 'failed to stop' not in output
    finally:
        if child.poll() is None:
            child.kill()
            child.wait()
        child.stdout.close()


@pytest.mark.skipif(not SERVER.exists(), reason='build fleet_bridge first')
def test_native_host_rejects_malformed_world_without_claiming_success(tmp_path):
    path = tmp_path/'broken.sdf'
    path.write_text('<sdf><broken')
    env = dict(os.environ, IGN_PARTITION='fleet-host-invalid-'+str(os.getpid()),
               GZ_PARTITION='fleet-host-invalid-'+str(os.getpid()), ROS_DOMAIN_ID='228')
    child = subprocess.run([str(SERVER), str(path)], env=env,
        capture_output=True, text=True, timeout=8)
    assert child.returncode > 0, child.stdout+child.stderr
