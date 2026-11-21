"""Simulation-only preparation and strict separation of robot configuration."""
import json
import math
import copy
import subprocess
from pathlib import Path
import uuid
import xml.etree.ElementTree as ET
import yaml
from .prepare import prepare, world
from .scene import walls


def sensor_visible_peer_model(sdf, index, radius):
    """Opaque peer footprint at the scan plane, hidden only from its own lidar.

    Renderer masks stay in simulation assets. No peer pose or association is
    passed to a robot. Physical mass, collision geometry and TF are unchanged.
    """
    sdf.set('version', '1.9')
    bit = 1 << index
    for visual in sdf.findall('.//visual'):
        flags = visual.find('visibility_flags')
        if flags is None:
            flags = ET.SubElement(visual, 'visibility_flags')
        flags.text = str(bit)
    for link in sdf.findall('.//link'):
        sensor = link.find("sensor[@name='rplidar']")
        if sensor is None:
            continue
        lidar = sensor.find('lidar')
        if lidar is None:
            lidar = sensor.find('ray')
        if lidar is None:
            raise ValueError('converted lidar configuration missing')
        ET.SubElement(lidar, 'visibility_mask').text = str(0xffffffff ^ bit)
        envelope = ET.SubElement(link, 'visual', name='peer_lidar_envelope')
        if sensor.find('pose') is not None:
            envelope.append(copy.deepcopy(sensor.find('pose')))
        geometry = ET.SubElement(envelope, 'geometry')
        cylinder = ET.SubElement(geometry, 'cylinder')
        ET.SubElement(cylinder, 'radius').text = str(radius)
        ET.SubElement(cylinder, 'length').text = '.12'
        ET.SubElement(envelope, 'visibility_flags').text = str(bit)
        return sdf
    raise ValueError('converted robot lacks lidar')


def apply_spawn_layout(cfg, layout):
    if layout not in (0, 1, 2):
        raise ValueError('spawn-layout must be 0, 1 or 2')
    positions = [(-5., -5., 0.), (0., -5., 0.), (5., -5., 0.)]
    if layout == 1:
        positions = [(-5., -5., math.pi/2), (0., -5., -math.pi/3), (5., -5., math.pi)]
    if layout == 2:
        positions = [(-5., -4.5, .7), (.5, -5.5, -1.3), (5.5, -4.5, 2.4)]
    for robot, pose in zip(cfg['robots'], positions):
        robot['start'] = list(pose)
    return cfg


def prepare_agents(out, config, session_id, no_dashboard=False, acceptance_scene='none'):
    out = Path(out)
    cfg = prepare(out, config, remove_lidar_visuals=False)
    if acceptance_scene != 'none':
        if acceptance_scene not in ('separated-rooms', 'symmetric-square'):
            raise ValueError('unknown negative acceptance scene')
        geometry = walls()
        if acceptance_scene == 'separated-rooms':
            geometry += [('partition_west', -2.5, -5., .2, 5.),
                         ('partition_east', 2.5, -5., .2, 5.),
                         ('partition_top', 0., -2.6, 15., .2)]
        world(out, geometry, dict(generator='acceptance-'+acceptance_scene, negative_case=True,
                                 arena=cfg['arena'], seed=cfg['scene']['seed']))
    agents = out/'agents'
    agents.mkdir(exist_ok=True)
    for robot in cfg['robots']:
        name = robot['name']
        tree = ET.fromstring((out/f'{name}.urdf').read_text())
        plugin = tree.find(".//plugin[@name='ign_ros2_control::IgnitionROS2ControlPlugin']")
        for remap in plugin.findall('ros/remapping'):
            if remap.text.startswith('/tf:='):
                remap.text = f'/tf:=/{name}/tf'
            elif remap.text.startswith('/tf_static:='):
                remap.text = f'/tf_static:=/{name}/tf_static'
            elif remap.text == '~/odom:=odom':
                remap.text = '~/odom:=wheel_odom'
        (out/f'{name}.urdf').write_text(ET.tostring(tree, encoding='unicode'))
        nav = yaml.safe_load((out/f'{name}_nav.yaml').read_text())

        def local_frames(value):
            if isinstance(value, dict):
                for key, child in list(value.items()):
                    if key == 'global_frame' and child == 'map':
                        value[key] = name+'/map'
                    elif key == 'filters':
                        # Each robot's navigation_map already includes its own
                        # certified region and peer masks. No shared filter server.
                        # Humble's YAML parser cannot infer the type of an
                        # empty array. Omitting it uses Costmap2DROS's typed
                        # empty-string-array default.
                        del value[key]
                    else:
                        local_frames(child)
            elif isinstance(value, list):
                for child in value:
                    local_frames(child)
        local_frames(nav)
        (out/f'{name}_nav.yaml').write_text(yaml.safe_dump(nav))
        safe = {key: cfg[key] for key in ('resolution', 'robot_radius', 'max_speed',
                                          'region_margin', 'dashboard_port')}
        safe.update(architecture='decentralized', robot_id=name,
                    members=[r['name'] for r in cfg['robots']], session_id=session_id,
                    boot_id=uuid.uuid4().hex, map_epoch=1, assets=str(out.resolve()),
                    run_dir=str(out.resolve().parent), no_dashboard=bool(no_dashboard))
        (agents/f'{name}.yaml').write_text(yaml.safe_dump(safe))
    # All gz_ros2_control plugins share the Gazebo process. Per-plugin /tf
    # remaps can therefore interfere. Publish odometry TF in separate per-robot
    # processes from each robot's encoder odometry instead.
    controls = yaml.safe_load((out/'controllers.yaml').read_text())
    for robot in cfg['robots']:
        name = robot['name']
        controls[f'/{name}/diffdrive_controller']['ros__parameters']['enable_odom_tf'] = False
        path = out/f'{name}_control.yaml'
        local = yaml.safe_load(path.read_text())
        local[f'/{name}/diffdrive_controller']['ros__parameters']['enable_odom_tf'] = False
        path.write_text(yaml.safe_dump(local))
    (out/'controllers.yaml').write_text(yaml.safe_dump(controls))
    for index, robot in enumerate(cfg['robots']):
        name = robot['name']
        converted = subprocess.check_output(['ign', 'sdf', '-p', str(out/f'{name}.urdf')], text=True)
        sdf = sensor_visible_peer_model(ET.fromstring(converted), index, cfg['robot_radius'])
        (out/f'{name}.sdf').write_text(ET.tostring(sdf, encoding='unicode'))
    architecture = out.parent/'architecture.json'
    temporary = architecture.with_suffix('.json.tmp')
    temporary.write_text(json.dumps({
        'architecture': 'decentralized', 'initialization': 'unknown',
        'session_id': session_id, 'agents': [r['name'] for r in cfg['robots']],
        'no_dashboard': bool(no_dashboard)}, indent=2))
    temporary.replace(architecture)
    return cfg
