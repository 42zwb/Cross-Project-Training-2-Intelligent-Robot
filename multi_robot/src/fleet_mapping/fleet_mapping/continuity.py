"""Retained local-frame geometry, without fitting or changing a transform."""
import math
import time
import numpy as np
from scipy import ndimage
from .registration import Snapshot, distinctive_support, registration_cancelled
from .peer_echoes import filter_peer_echoes


def retained_geometry(reference, current, diagnostics=None):
    def reject(reason, **metrics):
        if diagnostics is not None:
            diagnostics.update(reason=reason, **metrics)
        return None

    if ((reference.robot, reference.boot, reference.epoch)
            != (current.robot, current.boot, current.epoch)
            or current.revision < reference.revision or registration_cancelled()):
        return reject('identity_revision_or_cancelled')
    # Compare in the existing local coordinates. There is no optimized SE(2),
    # scale, mirror or newly selected alias in this continuity check.
    points = reference.occupied(edges=True)
    if len(points) < 40:
        return reject('insufficient_reference_edges', edges=len(points))
    values, cells, valid = current.samples(points)
    seen = valid & (values >= 0)
    if seen.mean() < .90:
        return reject('insufficient_observed_overlap', observed=float(seen.mean()))
    distance = ndimage.distance_transform_edt(current.data < 65)*current.resolution
    if registration_cancelled():
        return reject('cancelled')
    distances = distance[cells[seen, 1], cells[seen, 0]]
    supported = points[seen][distances < .13]
    if (len(supported) < 40 or np.linalg.eigvalsh(np.cov(supported.T))[0] < .02
            or not distinctive_support(supported, current.resolution)):
        return reject('insufficient_distinctive_support', supported=len(supported))
    retained = max(40, int(math.ceil(len(distances)*.92)))
    rms = float(np.sqrt(np.mean(np.sort(distances)[:retained]**2)))
    score = float(np.exp(-(distances/.13)**2).mean())
    conflict = float(np.mean((distances > .20) & (values[seen] <= 25)))
    metrics = dict(score=score, rms=rms, observed=float(seen.mean()), conflict=conflict)
    if score < .85 or rms > .05 or conflict > .08:
        return reject('geometry_threshold', **metrics)
    if diagnostics is not None:
        diagnostics.update(reason='accepted', **metrics)
    return metrics


def retained_geometry_job(current, references, graph, samples, now, radius, raw_sources,
                          diagnostics=False):
    """Run the bounded retained-geometry work in the registration process."""
    def view(snapshot):
        return Snapshot(snapshot.robot, snapshot.boot, snapshot.epoch, snapshot.revision,
            filter_peer_echoes(snapshot, graph, samples, now, radius, raw_sources),
            snapshot.resolution, snapshot.origin)
    current_view, results, checks = view(current), [], []
    for token, reference in references:
        if registration_cancelled():
            return dict(proofs=[], checks=checks, cancelled=True) if diagnostics else []
        check = dict(certificate=token, reference_revision=reference.revision,
                     revision=current.revision)
        if retained_geometry(view(reference), current_view, check) is not None:
            results.append((token, reference.revision, current.revision, time.monotonic_ns()))
        checks.append(check)
    return dict(proofs=results, checks=checks, cancelled=False) if diagnostics else results
