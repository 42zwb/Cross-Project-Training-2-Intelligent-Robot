import math
import sys

from geometry_msgs.msg import PoseStamped
from rclpy.utilities import remove_ros_args
from rclpy.validate_namespace import validate_namespace


def make_pose(node, x, y, yaw, frame_id='map'):
    """Create a stamped planar pose from x, y and yaw."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
    pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return pose


def duration_seconds(duration_msg):
    return float(duration_msg.sec) + float(duration_msg.nanosec) * 1e-9


def parse_cli_args(parser):
    """Parse application arguments while leaving ROS arguments for rclpy."""
    return parser.parse_args(remove_ros_args(args=sys.argv)[1:])


def normalize_namespace(value, allow_empty=True):
    """Return a validated relative ROS namespace without surrounding slashes."""
    namespace = value.strip().strip('/')
    if not namespace:
        if allow_empty:
            return ''
        raise ValueError('机器人命名空间不能为空')
    validate_namespace('/' + namespace)
    return namespace
