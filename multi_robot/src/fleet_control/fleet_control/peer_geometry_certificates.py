"""Two endpoint proofs retain a certified frame; they cannot create an edge."""
import copy
from collections import OrderedDict
from fleet_interfaces.msg import MapContinuity
from .peer_protocol import ReplayWindow, digest
from .peer_common import pose_list


GEOMETRY_PROOF_TTL = 15.  # Geometry evidence only; motion still expires in .5 s.


def certificate_digest(message):
    return digest([message.session_id, message.constraint_id,
        message.source_robot, message.source_boot, message.source_epoch, message.source_revision,
        message.target_robot, message.target_boot, message.target_epoch, message.target_revision,
        pose_list(message.transform), message.score, message.overlap,
        message.translation_sigma, message.rotation_sigma])


def endpoints(message):
    return {message.source_robot: (message.source_boot, message.source_epoch, message.source_revision),
            message.target_robot: (message.target_boot, message.target_epoch, message.target_revision)}


def family(message):
    return tuple(sorted((n, *p[:2]) for n, p in endpoints(message).items()))


def version(message):
    return tuple((n, p[2]) for n, p in sorted(endpoints(message).items()))


class GeometryCertificates:
    def __init__(self, session, robot, boot, epoch):
        self.session, self.robot, self.boot, self.epoch = session, robot, boot, epoch
        self.certificates, self.references = OrderedDict(), {}
        self.proofs, self.own_results = {}, {}
        self.replay, self.sequence = ReplayWindow(), 0
        self.receive_counts, self.last_receive = {}, {}

    def remember(self, message, reference=None):
        if (message.session_id != self.session or len(endpoints(message)) != 2
                or set(message.witnesses) != set(endpoints(message))):
            return
        token = certificate_digest(message)
        self.certificates.setdefault(token, copy.deepcopy(message))
        own = endpoints(message).get(self.robot)
        if (reference is not None and own == (self.boot, self.epoch, reference.revision)
                and (reference.robot, reference.boot, reference.epoch)
                    == (self.robot, self.boot, self.epoch)):
            self.references[token] = reference
        # Preserve an initial certified view even when later maps contain
        # temporary observations that disappear again. Keep one initial and
        # at most three recent certificates per identity pair, within the
        # existing global limit. Touch the whole family for global LRU eviction.
        keys = [key for key, base in self.certificates.items() if family(base) == family(message)]
        anchor = min(keys, key=lambda key: version(self.certificates[key]))
        keep = {anchor, *sorted(keys, key=lambda key: version(self.certificates[key]))[-3:]}
        for key in keys:
            if key not in keep:
                self.discard(key)
        for key in keys:
            if key in self.certificates:
                self.certificates.move_to_end(key)
        while len(self.certificates) > 64:
            self.discard(next(iter(self.certificates)))

    def discard(self, token):
        self.certificates.pop(token, None)
        self.references.pop(token, None)
        self.proofs.pop(token, None)
        self.own_results.pop(token, None)

    def active_references(self, members, pending=None):
        groups = {}
        for token, message in self.certificates.items():
            identities = endpoints(message)
            if (token in self.references
                    and (pending is None or set(identities) & pending)
                    and all(members.get(n) == p[:2] for n, p in identities.items())):
                groups.setdefault(family(message), []).append(token)
        result = []
        for keys in groups.values():
            keys.sort(key=lambda token: version(self.certificates[token]))
            for token in dict.fromkeys([keys[-1], keys[0]]):
                result.append((token, self.references[token]))
        return result

    def completed(self, results):
        for token, reference, revision, validated in results:
            if (token in self.references and self.references[token].revision == reference
                    and revision >= reference):
                self.own_results[token] = (reference, revision, validated)

    def outgoing(self, now):
        for token, (reference, revision, validated) in list(self.own_results.items()):
            if not 0 <= now-validated/1e9 <= GEOMETRY_PROOF_TTL:
                del self.own_results[token]
                continue
            self.sequence += 1
            yield MapContinuity(session_id=self.session, robot_id=self.robot, boot_id=self.boot,
                map_epoch=self.epoch, sequence=self.sequence, constraint_digest=token,
                reference_revision=reference, revision=revision, validated_steady_ns=validated)

    def receive(self, sender, message, members, versions, now):
        def finish(reason):
            self.receive_counts[reason] = self.receive_counts.get(reason, 0)+1
            self.last_receive = dict(reason=reason, sender=sender,
                certificate=message.constraint_digest, revision=message.revision, steady_time=now)

        base = self.certificates.get(message.constraint_digest)
        if base is None:
            return finish('missing_certificate')
        if message.session_id != self.session or message.robot_id != sender:
            return finish('session_or_sender')
        if not 0 <= now-message.validated_steady_ns/1e9 <= GEOMETRY_PROOF_TTL:
            return finish('expired_or_future')
        identities = endpoints(base)
        identity = identities.get(sender)
        if (identity is None or members.get(sender) != (message.boot_id, message.map_epoch)
                or identity != (message.boot_id, message.map_epoch, message.reference_revision)):
            return finish('identity_or_reference')
        if not message.reference_revision <= message.revision <= versions.get(sender, 0):
            return finish('map_version_not_available')
        if (sender == self.robot and self.own_results.get(message.constraint_digest)
                != (message.reference_revision, message.revision, message.validated_steady_ns)):
            return finish('missing_local_computation')
        if not self.replay.accept(sender, message.boot_id, message.sequence, message.map_epoch):
            return finish('replay')
        self.proofs.setdefault(message.constraint_digest, {})[sender] = copy.deepcopy(message)
        finish('accepted')

    def diagnostics(self, members, now):
        rows = []
        for token, message in list(self.certificates.items())[-6:]:
            identities = endpoints(message)
            proofs = self.proofs.get(token, {})
            rows.append(dict(certificate=token, local_reference=token in self.references,
                endpoints={n: dict(boot_id=p[0], epoch=p[1], reference_revision=p[2],
                    current_identity=members.get(n),
                    proof_revision=proofs[n].revision if n in proofs else None,
                    proof_age=now-proofs[n].validated_steady_ns/1e9 if n in proofs else None)
                    for n, p in identities.items()}))
        return dict(certificates=len(self.certificates), local_references=len(self.references),
            computed_proofs=len(self.own_results), receive_counts=dict(self.receive_counts),
            last_receive=dict(self.last_receive), recent_certificates=rows)

    def confirmed_revision(self, members, now):
        revision = 0
        for token, proofs in self.proofs.items():
            if token not in self.references:
                continue
            identities = endpoints(self.certificates[token])
            if (set(identities) <= set(proofs)
                    and all(members.get(n) == p[:2] for n, p in identities.items())
                    and all(0 <= now-proofs[n].validated_steady_ns/1e9 <= GEOMETRY_PROOF_TTL
                            for n in identities)):
                revision = max(revision, proofs[self.robot].revision)
        return revision

    def relayed_bases(self):
        for token in self.own_results:
            if token in self.certificates:
                yield self.certificates[token]
