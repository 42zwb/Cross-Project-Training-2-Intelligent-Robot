"""ROS-independent fencing, replay protection and intersecting vote leases.

There is no leader. A motion holder needs two distinct, fresh grants. The
holder's 0.5 s authorization is deliberately shorter than each voter's 2 s
reservation, including the native controller's stopping allowance.
"""
from dataclasses import dataclass
import hashlib
import json
import math

MEMBERS = ('robot1', 'robot2', 'robot3')
HOLDER_TTL = 0.5
VOTER_TTL = 2.0
PEER_TIMEOUT = 0.8


def command_targets_boot(command, name, boot):
    if command.kind in ('pause', 'estop'):
        return True
    targets = command.participant_boot_ids
    return (len(targets) == len(MEMBERS) and name in MEMBERS
            and all(len(value) <= 64 for value in targets)
            and targets[MEMBERS.index(name)] == boot)


def command_is_current(command, now, peers):
    if command.kind in ('pause', 'estop'):
        return True
    if not math.isfinite(command.expires_at) or command.expires_at <= 0:
        return False
    if now <= command.expires_at:
        return True
    targets = command.participant_boot_ids
    if len(targets) != len(MEMBERS):
        return False
    # Only an already committed, currently connected quorum can keep a task
    # active beyond the operator's proposal deadline. Delayed votes alone cannot.
    return sum(n in peers and now-peers[n][0] < PEER_TIMEOUT
               and peers[n][1].boot_id == targets[i]
               and peers[n][1].mission_id == command.command_id
               for i, n in enumerate(MEMBERS)) >= 2


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'),
                                    allow_nan=False).encode()).hexdigest()[:24]


def finite_pose(pose):
    return (isinstance(pose, (list, tuple)) and len(pose) == 3
            and all(isinstance(x, (int, float)) and not isinstance(x, bool)
                    and math.isfinite(x) for x in pose))


def resources_conflict(a, b):
    # Different certified Voronoi regions may move in parallel. A team-wide
    # transit/bootstrap/formation request conflicts with every regional holder.
    return a == b or not (a.startswith('region/') and b.startswith('region/'))


def choose_motion_owner(contenders):
    """Deterministic contention order, not an authority or a fixed leader."""
    if not contenders:
        return None
    held = {n: d for n, d in contenders.items() if d.get('held')}
    candidates = held or contenders
    def priority(item):
        name, value = item
        if value.get('kind') == 'patrol':
            return (0, value.get('completed', 0), name)
        return (1, -value.get('score', 0.), name)
    return min(candidates.items(), key=priority)[0]


@dataclass
class Reservation:
    resource: str
    owner: str
    boot: str
    request: str
    mission: str
    sequence: int
    until: float


class LeaseVoter:
    def __init__(self, robot, started):
        if robot not in MEMBERS:
            raise ValueError('unknown member')
        self.robot = robot
        self.quarantine_until = started + VOTER_TTL
        self.reservations = {}
        self.released = set()

    def grant(self, resource, owner, boot, request, mission, sequence, now):
        if (now < self.quarantine_until or owner not in MEMBERS or not boot or not request
                or not mission or not isinstance(sequence, int) or isinstance(sequence, bool)
                or sequence < 1 or (boot, request) in self.released
                or resource not in ('team_motion', 'region/'+owner)):
            return False
        for key, old in list(self.reservations.items()):
            if old.until <= now:
                del self.reservations[key]
                continue
            same = (old.owner, old.boot, old.request, old.mission) == (owner, boot, request, mission)
            if resources_conflict(resource, old.resource) and not same:
                return False
            if same and (sequence <= old.sequence or resource != old.resource):
                return False
        self.reservations[resource] = Reservation(resource, owner, boot, request,
                                                 mission, sequence, now + VOTER_TTL)
        return True

    def release(self, owner, boot, request, stopped):
        if not stopped:
            return False
        self.released.add((boot, request))
        for key, old in list(self.reservations.items()):
            if (old.owner, old.boot, old.request) == (owner, boot, request):
                del self.reservations[key]
        return True


class ReplayWindow:
    """A boot change is explicit; delayed packets cannot switch an identity back."""
    def __init__(self):
        self.current = {}
        self.retired = set()
        self.epochs = {}

    def accept(self, robot, boot, sequence, epoch=None):
        if (robot not in MEMBERS or not boot or (robot, boot) in self.retired
                or not isinstance(sequence, int) or isinstance(sequence, bool) or sequence < 1):
            return False
        previous = self.current.get(robot)
        if epoch is not None:
            if not isinstance(epoch, int) or isinstance(epoch, bool) or epoch < 1:
                return False
            old_epoch = self.epochs.get(robot)
            if old_epoch is not None and (epoch < old_epoch
                    or epoch == old_epoch and previous and previous[0] != boot
                    or epoch > old_epoch and previous and previous[0] == boot):
                return False
        if previous and previous[0] != boot:
            self.retired.add((robot, previous[0]))
        elif previous and sequence <= previous[1]:
            return False
        self.current[robot] = (boot, sequence)
        if epoch is not None:
            self.epochs[robot] = epoch
        return True


class GrantTracker:
    """Safety independently observes the original votes, never an agent's count."""
    def __init__(self):
        self.requests = {}
        self.grants = {}

    def requested(self, request, mission, resource, sequence, sent, now):
        if not 0 <= now - sent < HOLDER_TTL:
            return
        key = (request, mission, resource, sequence)
        self.requests[key] = sent
        for old, stamp in list(self.requests.items()):
            if now - stamp >= HOLDER_TTL:
                self.requests.pop(old, None)
                self.grants.pop(old, None)

    def granted(self, voter, voter_boot, request, mission, resource, sequence):
        key = (request, mission, resource, sequence)
        if key in self.requests and voter in MEMBERS and voter_boot:
            self.grants.setdefault(key, {})[voter] = voter_boot

    def valid(self, request, mission, resource, now, live_boots):
        for key, sent in self.requests.items():
            if key[:3] != (request, mission, resource) or not 0 <= now - sent < HOLDER_TTL:
                continue
            votes = self.grants.get(key, {})
            if sum(live_boots.get(n) == boot for n, boot in votes.items()) >= 2:
                return True
        return False


def lost_peer_radius(radius, maximum_speed, age, position_sigma=0.05):
    if not all(math.isfinite(x) and x >= 0 for x in (radius, maximum_speed, age, position_sigma)):
        raise ValueError('invalid uncertainty')
    return radius + 0.10 + maximum_speed * age + 3.0 * position_sigma
