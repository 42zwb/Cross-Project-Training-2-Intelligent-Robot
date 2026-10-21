from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'turtlebot4_tutorial'


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='bwz',
    maintainer_email='bwz@example.com',
    description='ROS 2 Humble TurtleBot 4 ground robot tutorial nodes and launch files.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'keyboard_control = turtlebot4_tutorial.keyboard_control:main',
            'joystick_control = turtlebot4_tutorial.joystick_control:main',
            'map_loader = turtlebot4_tutorial.map_loader:main',
            'map_saver = turtlebot4_tutorial.map_saver:main',
            'nav_commander_demo = turtlebot4_tutorial.nav_commander_demo:main',
            'nav_action_demo = turtlebot4_tutorial.nav_action_demo:main',
            'robot_status_publisher = turtlebot4_tutorial.robot_status_publisher:main',
            'multi_robot_patrol = turtlebot4_tutorial.multi_robot_patrol:main',
            'collision_monitor = turtlebot4_tutorial.collision_monitor:main',
            'mapping_patrol = turtlebot4_tutorial.mapping_patrol:main',
            'robot_description_filter = turtlebot4_tutorial.robot_description_filter:main',
            'initial_pose_publisher = turtlebot4_tutorial.initial_pose_publisher:main',
        ],
    },
)
