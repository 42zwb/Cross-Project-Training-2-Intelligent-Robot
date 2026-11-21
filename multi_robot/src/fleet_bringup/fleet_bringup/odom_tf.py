"""Publish only this robot's encoder-odometry TF in its private TF topic."""
import math
import uuid
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from tf2_ros import TransformBroadcaster
from fleet_control.peer_common import load_config, STATE_QOS
from fleet_control.ros_runtime import RosRuntime


def main():
    with RosRuntime() as runtime:
        node = rclpy.create_node('odom_tf')
        cfg = load_config(node)
        name = cfg['robot_id']
        broadcaster = TransformBroadcaster(node)
        publisher = node.create_publisher(Odometry, f'/{name}/odom', 10)
        identity = node.create_publisher(String, f'/{name}/odometry_identity', STATE_QOS)
        epoch = uuid.uuid4().hex
        identity.publish(String(data=epoch))

        def received(message):
            p, q = message.pose.pose.position, message.pose.pose.orientation
            if (message.header.frame_id != name+'/odom' or message.child_frame_id != name+'/base_link'
                    or not all(math.isfinite(v) for v in (p.x, p.y, p.z, q.x, q.y, q.z, q.w))
                    or not .99 <= q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w <= 1.01):
                return
            t = TransformStamped()
            t.header = message.header
            t.child_frame_id = message.child_frame_id
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
            t.transform.rotation = q
            publisher.publish(message)
            broadcaster.sendTransform(t)
        node.create_subscription(Odometry, f'/{name}/wheel_odom', received, qos_profile_sensor_data)
        try:
            runtime.spin(node)
        finally:
            node.destroy_node()
