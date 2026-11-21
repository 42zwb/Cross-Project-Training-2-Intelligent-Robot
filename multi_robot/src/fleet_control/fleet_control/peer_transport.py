"""Bounded receive-boundary network faults for reproducible local acceptance.

Absent a session-qualified profile this is a direct callback. It never changes
missions, transforms, sensor data or actuators. Delayed packets retain their
original identities, sequences and send times, including after profile removal.
"""
import copy
import hashlib
import heapq
import json
import math
from pathlib import Path
import random
import time
from rclpy.clock import Clock, ClockType
from .peer_protocol import MEMBERS


class FaultQueue:
    def __init__(self, robot, session, component, maximum_bytes=8*1024*1024):
        self.robot, self.session = robot, session
        self.rng = random.Random(int.from_bytes(hashlib.sha256((robot+component).encode()).digest()[:4], 'big'))
        self.rules, self.pending, self.bytes, self.serial = [], [], 0, 0
        self.maximum_bytes, self.dropped, self.delivered = maximum_bytes, 0, 0

    def profile(self, value):
        if not isinstance(value, dict) or value.get('session_id') != self.session:
            raise ValueError('fault profile session mismatch')
        links = value.get('links', [])
        if not isinstance(links, list) or len(links) > 40:
            raise ValueError('too many fault rules')
        for rule in links:
            if (not isinstance(rule, dict) or rule.get('source') not in MEMBERS
                    or rule.get('target') not in MEMBERS or not isinstance(rule.get('channels', []), list)
                    or set(rule.get('channels', []))-{'state', 'event', 'map', 'constraint', 'mapping', 'continuity'}):
                raise ValueError('invalid peer fault rule')
            for key, maximum in (('drop', 1.), ('delay', 2.), ('reorder', 2.), ('duplicates', 3.)):
                item = rule.get(key, 0)
                if not isinstance(item, (int, float)) or not math.isfinite(item) or not 0 <= item <= maximum:
                    raise ValueError('invalid fault parameter')
        self.rules = copy.deepcopy(links)

    def receive(self, source, channel, callback, args, now, size=1024):
        rule = next((r for r in self.rules if source != self.robot and r['source'] == source
                     and r['target'] == self.robot and channel in r.get('channels', [])), None)
        if rule is None:
            callback(*args)
            self.delivered += 1
            return
        if self.rng.random() < rule.get('drop', 0.):
            self.dropped += 1
            return
        copies = 1+int(rule.get('duplicates', 0))
        for _ in range(copies):
            delay = rule.get('delay', 0.)+self.rng.random()*rule.get('reorder', 0.)
            if not delay:
                callback(*args)
                self.delivered += 1
                continue
            if len(self.pending) >= 512 or self.bytes+size > self.maximum_bytes:
                self.dropped += 1
                continue
            self.serial += 1
            heapq.heappush(self.pending, (now+delay, self.serial, size, callback, copy.deepcopy(args)))
            self.bytes += size

    def flush(self, now, maximum=100):
        count = 0
        while self.pending and self.pending[0][0] <= now and count < maximum:
            _, _, size, callback, args = heapq.heappop(self.pending)
            self.bytes -= size
            callback(*args)
            self.delivered += 1
            count += 1


class PeerInbox:
    def __init__(self, node, cfg):
        self.queue = FaultQueue(cfg['robot_id'], cfg['session_id'], node.get_name())
        self.path = Path(cfg['run_dir'])/'peer_network_faults.json'
        self.check_after, self.signature = 0., None
        self.timer = node.create_timer(.02, self.tick, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def tick(self):
        now = time.monotonic()
        if now >= self.check_after:
            self.check_after = now+.2
            try:
                stat = self.path.stat()
                signature = stat.st_mtime_ns, stat.st_size
                if signature != self.signature:
                    if stat.st_size > 65536:
                        raise ValueError('oversized fault profile')
                    self.queue.profile(json.loads(self.path.read_text()))
                    self.signature = signature
            except FileNotFoundError:
                self.queue.rules, self.signature = [], None
            except (OSError, ValueError, TypeError):
                pass  # A partial/bad profile cannot undo an active isolation.
        self.queue.flush(now)

    def receive(self, source, channel, callback, *args):
        message = args[-1]
        size = 1024+len(getattr(getattr(message, 'grid', None), 'data', []))
        size += len(getattr(message, 'payload_json', '').encode())+len(getattr(message, 'details_json', '').encode())
        size += len(getattr(getattr(message, 'task', None), 'argument', '').encode())
        self.queue.receive(source, channel, callback, args, time.monotonic(), size)
