#!/usr/bin/env python3
"""Read-only truth comparison of raw peer visibility in the empty test arena."""
import argparse
import json
import math
from pathlib import Path
import time
import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer
from rclpy.qos import qos_profile_sensor_data
from fleet_control.peer_common import STATE_QOS
from fleet_control.peer_protocol import MEMBERS
from fleet_mapping.registration import compose, inverse, angle
from fleet_control.relative_evaluate import rows


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-dir', required=True)
    run = Path(parser.parse_args().run_dir).resolve()
    assert json.loads((run/'manifest.json').read_text())['acceptance_scene'] == 'symmetric-square'
    rclpy.init()
    node = rclpy.create_node('independent_raw_lidar_acceptance')
    scans, buffers = {}, {n: Buffer() for n in MEMBERS}
    for name, buffer in buffers.items():
        node.create_subscription(LaserScan, f'/{name}/scan',
            lambda m, n=name: scans.__setitem__(n, m), qos_profile_sensor_data)
        def transform(message, buffer=buffer):
            for tf in message.transforms:
                buffer.set_transform_static(tf, 'independent_check')
        node.create_subscription(TFMessage, f'/{name}/tf_static', transform, STATE_QOS)
    try:
        end = time.monotonic()+30
        while time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=.1)
            if len(scans) == 3 and all(buffers[n].can_transform(
                    n+'/base_link', n+'/rplidar_link', rclpy.time.Time()) for n in MEMBERS):
                break
        assert len(scans) == 3
        truth, checks, coverage = rows(run/'truth.jsonl')[-1]['robots'], [], {}
        for own, buffer in buffers.items():
            tf = buffer.lookup_transform(own+'/base_link', own+'/rplidar_link', rclpy.time.Time()).transform
            p, q = tf.translation, tf.rotation
            sensor = compose(truth[own][:3], (p.x, p.y,
                math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
            scan = scans[own]
            angles = scan.angle_min+np.arange(len(scan.ranges))*scan.angle_increment
            values = np.asarray(scan.ranges)
            coverage[own] = float(np.mean((np.isfinite(values) & (values >= scan.range_min))
                                           | np.isposinf(values)))
            peers = {n: compose(inverse(sensor), truth[n][:3]) for n in MEMBERS if n != own}
            for other, point in peers.items():
                distance, theta = math.hypot(*point[:2]), math.atan2(point[1], point[0])
                if distance > scan.range_max:
                    continue
                if any(math.hypot(*a[:2]) < distance-.4 and
                       abs(angle(math.atan2(a[1], a[0])-theta)) < math.asin(.23/math.hypot(*a[:2]))
                       for n, a in peers.items() if n != other):
                    continue
                delta = np.arctan2(np.sin(angles-theta), np.cos(angles-theta))
                near = values[np.abs(delta) < math.asin(.23/distance)]
                checks.append(dict(observer=own, target=other, center_distance=distance,
                    visible=bool(np.any(np.isfinite(near) & (np.abs(near-distance) < .3))),
                    finite_ranges=near[np.isfinite(near)].tolist()))
        report = dict(passed=len(checks) >= 4 and all(x['visible'] for x in checks)
            and min(coverage.values()) >= .98, checks=checks, valid_coverage=coverage,
            method='raw scan only; truth used by this read-only observer; no control publications')
        (run/'peer-lidar-acceptance.json').write_text(json.dumps(report, ensure_ascii=False, indent=2))
        print(json.dumps(report, ensure_ascii=False, indent=2))
        assert report['passed'], report
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
