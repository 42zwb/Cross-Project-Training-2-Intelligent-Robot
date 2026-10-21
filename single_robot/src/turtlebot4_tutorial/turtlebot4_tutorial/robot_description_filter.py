#!/usr/bin/env python3
"""
Generate a TurtleBot 4 URDF with a Gazebo Fortress LiDAR workaround.

The installed description selects the legacy Ogre renderer, which returns the
sensor minimum for every simulated RPLIDAR ray on this machine.  Ogre 2 keeps
the original robot geometry and produces useful scan data.
"""
import argparse
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from irobot_create_common_bringup.namespace import GetNamespacedName

from .common import parse_cli_args


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', choices=['lite', 'standard'], default='lite')
    parser.add_argument('--namespace', default='')
    parser.add_argument('--sensors-plugin', choices=['true', 'false'],
                        default='true')
    args = parse_cli_args(parser)
    description = get_package_share_directory('turtlebot4_description')
    xacro = os.path.join(
        description, 'urdf', args.model, 'turtlebot4.urdf.xacro')
    command = ['xacro', xacro, 'gazebo:=ignition']
    if args.namespace:
        command.append(f'namespace:={args.namespace}')
    result = subprocess.run(command, check=True, capture_output=True, text=True)
    root = ET.fromstring(result.stdout)
    control = get_package_share_directory('turtlebot4_tutorial')
    for parameters in root.findall('.//plugin/parameters'):
        if parameters.text and parameters.text.endswith('/config/control.yaml'):
            parameters.text = os.path.join(control, 'config', 'control.yaml')
    sensors_plugin = root.find(
        ".//plugin[@name='ignition::gazebo::systems::Sensors']")
    if sensors_plugin is None:
        raise RuntimeError('生成的 URDF 中没有 Sensors/render_engine')
    if args.sensors_plugin == 'true':
        render_engine = sensors_plugin.find('render_engine')
        if render_engine is None:
            raise RuntimeError('生成的 URDF 中没有 Sensors/render_engine')
        render_engine.text = 'ogre2'
        gazebo = next(parent for parent in root.iter() if sensors_plugin in parent)
        ET.SubElement(gazebo, 'plugin', {
            'filename': 'libignition-gazebo-imu-system.so',
            'name': 'ignition::gazebo::systems::Imu',
        })
    else:
        for parent in root.iter():
            if sensors_plugin in parent:
                parent.remove(sensors_plugin)
                break
    imu = root.find(".//sensor[@type='imu']")
    if imu is None:
        raise RuntimeError('生成的 URDF 中没有 IMU 传感器')
    topic = ET.SubElement(imu, 'topic')
    robot_name = GetNamespacedName(args.namespace, 'turtlebot4').perform()
    topic.text = f'/model/{robot_name}/imu'
    sys.stdout.write(ET.tostring(root, encoding='unicode'))


if __name__ == '__main__':
    main()
