import subprocess
import sys
import xml.etree.ElementTree as ET


def test_second_robot_omits_duplicate_sensors_system():
    def plugins(setting):
        result = subprocess.run(
            [sys.executable, '-m', 'turtlebot4_tutorial.robot_description_filter',
             '--model', 'lite', '--namespace', 'robot2',
             '--sensors-plugin', setting],
            check=True, capture_output=True, text=True)
        root = ET.fromstring(result.stdout)
        assert root.findtext(".//sensor[@type='imu']/topic") == '/model/robot2/turtlebot4/imu'
        imu = root.findall(".//plugin[@name='ignition::gazebo::systems::Imu']")
        assert len(imu) == (1 if setting == 'true' else 0)
        return root.findall(".//plugin[@name='ignition::gazebo::systems::Sensors']")

    first = plugins('true')
    second = plugins('false')
    assert len(first) == 1
    assert first[0].findtext('render_engine') == 'ogre2'
    assert second == []
