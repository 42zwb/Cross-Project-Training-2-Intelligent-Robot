"""Independent raw-source checkpoints for acceptance and failure diagnosis."""
import os
import json
import time
from collections import deque
from pathlib import Path
import tempfile
import numpy as np
from .peer_common import snapshot_from_message


def atomic_npz(path, **values):
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(dir=path.parent, suffix='.npz', delete=False) as stream:
            temporary = Path(stream.name)
            np.savez_compressed(stream, **values)
        os.replace(temporary, path)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


class RawMapRecorder:
    def __init__(self, run, session):
        self.folder = Path(run)/'raw-map-checkpoints'
        self.folder.mkdir(exist_ok=True)
        self.session, self.latest, self.aligned = session, {}, set()

    def save(self, path, snapshot):
        atomic_npz(path, robot=snapshot.robot, boot=snapshot.boot,
            epoch=snapshot.epoch, revision=snapshot.revision, data=snapshot.data,
            resolution=snapshot.resolution, origin=snapshot.origin)

    def receive(self, robot, message):
        if message.session_id != self.session or message.robot_id != robot:
            return
        old = self.latest.get(robot)
        if old and (message.map_epoch < old.epoch
                    or message.map_epoch == old.epoch and message.revision <= old.revision):
            return
        snapshot = snapshot_from_message(message)
        self.latest[robot] = snapshot
        self.save(self.folder/f'{robot}-e{snapshot.epoch}-latest.npz', snapshot)

    def checkpoint(self, state):
        for name, robot in state['robots'].items():
            key = (name, robot.get('boot_id'), robot.get('map_epoch'))
            raw = self.latest.get(name)
            if (key in self.aligned or not robot.get('ready') or not robot.get('aligned')
                    or raw is None or (raw.boot, raw.epoch) != key[1:]):
                continue
            for source, snapshot in self.latest.items():
                self.save(self.folder/f'{name}-e{key[2]}-first-aligned-{source}.npz', snapshot)
            (self.folder/f'{name}-e{key[2]}-first-aligned.json').write_text(
                json.dumps(state, ensure_ascii=False, allow_nan=False))
            self.aligned.add(key)


class NavigationRecorder:
    """Read-only, bounded checkpoints of the inputs behind a navigation failure.

    Record actual received grids, including stale frames and occupied cells.
    No clearing, map fusion, target selection or actuator publication happens here.
    """
    def __init__(self, run):
        self.folder = Path(run)/'navigation-checkpoints'
        self.folder.mkdir(exist_ok=True)
        self.grids, self.scans, self.transforms, self.last = {}, {}, {}, {}

    def grid(self, robot, channel, message):
        self.grids[robot, channel] = time.monotonic(), message

    def scan(self, robot, message):
        self.scans[robot] = time.monotonic(), message

    def tf(self, robot, channel, message):
        self.transforms.setdefault((robot, channel), deque(maxlen=60)).append(
            (time.monotonic(), message))

    def checkpoint(self, state, peer_states):
        from rosidl_runtime_py.convert import message_to_ordereddict
        now = time.monotonic()
        for name, robot in state['robots'].items():
            failed = robot.get('mode') == 'FAULT'
            if not failed and (robot.get('mode') not in ('FORMING', 'PATROLLING')
                               or not robot.get('goal')):
                continue
            if not failed and now-self.last.get(name, 0.) < 2.:
                continue
            peer = peer_states.get(name)
            if not peer or peer[1].boot_id != robot.get('boot_id'):
                continue
            expected = name+'/map' if robot['map_epoch'] == 1 else f'{name}/map_e{robot["map_epoch"]}'
            metadata = dict(steady_time=now, expected_frame=expected, state=robot,
                coordinate_frame=state.get('coordinate_frame'),
                peer_received=peer[0], peer_state=message_to_ordereddict(peer[1]), grids={})
            arrays = {}
            for (owner, channel), (received, message) in self.grids.items():
                if owner != name:
                    continue
                meta = message_to_ordereddict(message)
                del meta['data']
                metadata['grids'][channel] = dict(received=received, message=meta,
                    expected_frame=message.header.frame_id == expected)
                if (0 < message.info.width*message.info.height <= 4_000_000
                        and len(message.data) == message.info.width*message.info.height):
                    arrays[channel] = np.asarray(message.data, np.int16).reshape(
                        message.info.height, message.info.width)
            if name in self.scans:
                received, scan = self.scans[name]
                meta = message_to_ordereddict(scan)
                del meta['ranges'], meta['intensities']
                metadata['scan'] = dict(received=received, message=meta)
                arrays['ranges'] = np.asarray(scan.ranges, np.float32)
                arrays['intensities'] = np.asarray(scan.intensities, np.float32)
            metadata['tf'] = {channel: [dict(received=received,
                message=message_to_ordereddict(message)) for received, message in messages]
                for (owner, channel), messages in self.transforms.items() if owner == name}
            arrays['metadata'] = json.dumps(metadata, ensure_ascii=False, allow_nan=False)
            tag = 'fault' if failed else 'latest'
            atomic_npz(self.folder/f'{name}-e{robot["map_epoch"]}-{tag}.npz', **arrays)
            self.last[name] = now
