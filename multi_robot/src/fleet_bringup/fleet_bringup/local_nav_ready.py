"""Launch only this robot's Nav2 when its own map/TF are valid."""
import signal
from collections import deque
import subprocess
import time
from pathlib import Path
import rclpy
from nav_msgs.msg import OccupancyGrid
from tf2_ros import Buffer, TransformListener, TransformException
from fleet_control.peer_common import load_config, STATE_QOS, local_frame
from fleet_mapping.grid import valid_grid_message
from fleet_control.ros_runtime import RosRuntime


def main():
    with RosRuntime() as runtime:
        node = rclpy.create_node('local_navigation_bootstrap')
        cfg = load_config(node)
        name = cfg['robot_id']
        buffer = Buffer()
        listener = TransformListener(buffer, node)
        grids = deque(maxlen=1)
        node.create_subscription(OccupancyGrid, f'/{name}/navigation_map',
            lambda m: grids.append(m) if valid_grid_message(m) else None, STATE_QOS)
        proc = None
        log = None
        try:
            end = time.monotonic()+120
            while runtime.ok() and time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=.1)
                try:
                    tf = buffer.lookup_transform(local_frame(name, cfg['map_epoch']), name+'/base_link', rclpy.time.Time())
                    age = (node.get_clock().now()-rclpy.time.Time.from_msg(tf.header.stamp)).nanoseconds*1e-9
                    if grids and -.2 < age < .5:
                        break
                except TransformException:
                    pass
            else:
                if not runtime.ok():
                    return
                raise RuntimeError(name+' 本车 SLAM/TF 未能就绪')
            config = str(Path(cfg['assets'])/'agents'/f'{name}.yaml')
            logpath = Path(cfg['run_dir'])/name/f'navigation-{cfg["map_epoch"]}.log'
            logpath.parent.mkdir(parents=True, exist_ok=True)
            log = logpath.open('a')
            proc = subprocess.Popen(['ros2', 'launch', 'fleet_bringup', 'robot_navigation.launch.py',
                                     'robot_config:='+config], stdin=subprocess.DEVNULL,
                                    stdout=log, stderr=subprocess.STDOUT)
            listener.unregister()
            node.destroy_node()
            node = None
            while runtime.ok() and proc.poll() is None:
                time.sleep(.2)
        finally:
            if proc and proc.poll() is None:
                proc.send_signal(signal.SIGINT)
                try:
                    proc.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    proc.terminate()
                    try:
                        proc.wait(timeout=4)
                    except subprocess.TimeoutExpired:
                        proc.kill()
                        proc.wait()
            if log:
                log.close()
            if node:
                node.destroy_node()
