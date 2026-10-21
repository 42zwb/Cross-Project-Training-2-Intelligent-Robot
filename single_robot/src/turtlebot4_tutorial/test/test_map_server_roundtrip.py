from collections import Counter
import os
from pathlib import Path
import signal
import subprocess
import threading
import time

from ament_index_python.packages import get_package_prefix
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile


def test_real_map_saver_and_map_server_preserve_all_three_cell_classes(tmp_path):
    context = Context()
    domain = 160 + os.getpid() % 40
    rclpy.init(context=context, domain_id=domain)
    node = rclpy.create_node('map_roundtrip_test', context=context)
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    publisher = node.create_publisher(OccupancyGrid, 'test_input', qos)
    message = OccupancyGrid()
    message.header.frame_id = 'map'
    message.info.resolution = 0.05
    message.info.width = 3
    message.info.height = 3
    message.info.origin.orientation.w = 1.0
    message.data = [-1, 0, 100] * 3
    timer = node.create_timer(0.1, lambda: publisher.publish(message))
    received = []
    subscription = node.create_subscription(
        OccupancyGrid, 'map', lambda m: received.append(Counter(m.data)), qos)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin)
    thread.start()
    prefix = Path(get_package_prefix('turtlebot4_tutorial')) / 'lib/turtlebot4_tutorial'
    env = dict(os.environ, ROS_DOMAIN_ID=str(domain))
    loader = None
    try:
        result = subprocess.run(
            [str(prefix / 'map_saver'), str(tmp_path / 'map'), '--topic', 'test_input'],
            env=env, capture_output=True, text=True, timeout=30)
        assert result.returncode == 0, result.stdout + result.stderr
        with (tmp_path / 'loader.log').open('w') as log:
            loader = subprocess.Popen(
                [str(prefix / 'map_loader'), str(tmp_path / 'map.yaml')],
                env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic() + 10.0
            while not received and time.monotonic() < deadline:
                time.sleep(0.1)
            assert received, (tmp_path / 'loader.log').read_text()
            assert received[-1] == Counter({-1: 3, 0: 3, 100: 3})
    finally:
        if loader is not None and loader.poll() is None:
            os.killpg(loader.pid, signal.SIGINT)
            try:
                loader.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(loader.pid, signal.SIGKILL)
                loader.wait(timeout=3)
        executor.shutdown(timeout_sec=3)
        thread.join(timeout=3)
        node.destroy_subscription(subscription)
        node.destroy_timer(timer)
        node.destroy_node()
        context.shutdown()
