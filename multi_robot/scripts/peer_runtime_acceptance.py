#!/usr/bin/env python3
"""Read-only audit of the actual peer graph and per-robot actuator ownership."""
import argparse
import json
from pathlib import Path
import time
import rclpy
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, TransformListener
from fleet_control.peer_client import PeerClient
from fleet_control.peer_common import STATE_QOS, local_frame
from fleet_control.peer_protocol import MEMBERS
import yaml


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--run-dir', required=True)
    a = p.parse_args()
    run = Path(a.run_dir).resolve()
    rclpy.init()
    client = PeerClient(run)
    trees, subscriptions = {}, []
    for name in MEMBERS:
        trees[name] = {}
        def received(msg, name=name):
            for transform in msg.transforms:
                trees[name].setdefault(transform.child_frame_id, set()).add(transform.header.frame_id)
        subscriptions += [client.node.create_subscription(TFMessage, f'/{name}/tf', received, 100),
                          client.node.create_subscription(TFMessage, f'/{name}/tf_static', received, STATE_QOS)]
    try:
        state = client.state(6.)
        nodes = client.node.get_node_names_and_namespaces()
        assert not any(ns == '/' and n in ('mapping', 'coordinator', 'safety') for n, ns in nodes), nodes
        report = dict(architecture='decentralized', passed=True, nodes=nodes, robots={})
        for name in MEMBERS:
            parents = trees[name]
            assert parents and all(len(v) == 1 for v in parents.values()), (name, parents)
            assert all(c.startswith(name+'/') and all(p.startswith(name+'/') for p in ps)
                       for c, ps in parents.items()), (name, parents)
            cfg = yaml.safe_load((run/'assets/agents'/f'{name}.yaml').read_text())
            frame = local_frame(name, cfg['map_epoch'])
            child, visited = name+'/base_link', set()
            while child in parents and child not in visited:
                visited.add(child)
                child = next(iter(parents[child]))
            assert child == frame, (name, child, parents)
            drive = client.node.get_publishers_info_by_topic(f'/{name}/diffdrive_controller/cmd_vel_unstamped')
            assert len(drive) == 1 and drive[0].node_name == 'local_safety' and drive[0].node_namespace == '/'+name
            for topic in ('scan', 'odom', 'navigation_map'):
                assert len(client.node.get_publishers_info_by_topic(f'/{name}/{topic}')) == 1, (name, topic)
            report['robots'][name] = dict(tf_frames=len(parents), root=child,
                unique_velocity_publisher='/'+name+'/local_safety', local_ready=state['robots'][name]['ready'])
        dependencies = {}
        for topic in ('/map', '/fleet/state'):
            subscribers = client.node.get_subscriptions_info_by_topic(topic)
            assert not any(s.node_namespace in ('/'+n for n in MEMBERS) for s in subscribers), (topic, subscribers)
            dependencies[topic] = [s.node_namespace+'/'+s.node_name for s in subscribers]
        report['monitor_subscriptions'] = dependencies
        (run/'peer-runtime-acceptance.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False, indent=2))
    finally:
        client.close()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
