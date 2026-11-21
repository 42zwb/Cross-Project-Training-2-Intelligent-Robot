"""The simulated GPU lidar must not see the fixed camera-bracket mesh."""
import xml.etree.ElementTree as ET
from fleet_bringup.prepare import remove_sim_lidar_self_occlusion


def test_only_occluding_visuals_are_removed_collisions_are_preserved():
    root = ET.fromstring('''<robot><link name="oakd_camera_bracket">
      <visual/><collision/><inertial/></link><link name="rplidar_link"><visual/><collision/></link><link name="base_link"><visual/></link>
      <joint name="oakd_camera_bracket_joint"/></robot>''')
    remove_sim_lidar_self_occlusion(root)
    bracket = root.find("link[@name='oakd_camera_bracket']")
    assert bracket.find('visual') is None
    assert bracket.find('collision') is not None
    assert bracket.find('inertial') is not None
    assert root.find("link[@name='rplidar_link']/visual") is None
    assert root.find("link[@name='rplidar_link']/collision") is not None
    assert root.find("link[@name='base_link']/visual") is not None
    assert root.find("joint[@name='oakd_camera_bracket_joint']") is not None
