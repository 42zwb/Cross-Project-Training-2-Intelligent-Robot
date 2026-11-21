"""Each raw lidar can see peer bodies without seeing its own housing."""
import xml.etree.ElementTree as ET
from fleet_bringup.peer_assets import sensor_visible_peer_model


def test_visibility_masks_exclude_only_own_geometry_and_keep_world_and_peers():
    masks, flags = [], []
    for index in range(3):
        root = ET.fromstring('<sdf version="1.9"><model name="robot"><link name="base">'
            '<visual name="housing"><geometry><box><size>1 1 1</size></box></geometry></visual>'
            '<collision name="body"><geometry><box><size>1 1 1</size></box></geometry></collision>'
            '<sensor name="rplidar" type="gpu_lidar"><pose>0 0 .14 0 0 0</pose><lidar/>'
            '</sensor></link></model></sdf>')
        old_collision = ET.tostring(root.find('.//collision'))
        root = sensor_visible_peer_model(root, index, .18)
        assert ET.tostring(root.find('.//collision')) == old_collision
        mask = int(root.findtext('.//lidar/visibility_mask'))
        flag = int(root.findtext(".//visual[@name='peer_lidar_envelope']/visibility_flags"))
        assert not mask & flag and mask & 0xffffffff
        assert float(root.findtext(".//visual[@name='peer_lidar_envelope']/geometry/cylinder/radius")) == .18
        assert root.findtext(".//visual[@name='peer_lidar_envelope']/pose") == '0 0 .14 0 0 0'
        masks.append(mask)
        flags.append(flag)
    assert all(masks[i] & flags[j] for i in range(3) for j in range(3) if i != j)
