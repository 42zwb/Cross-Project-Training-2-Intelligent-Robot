"""Unknown-origin grid registration. Only observed local grids enter matching."""
from dataclasses import dataclass
import math
import hashlib
import cv2
import numpy as np
from scipy import ndimage
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.signal import fftconvolve
from .grid import fuse

MIN_MATCH_SCORE = .80
MAX_MATCH_RMS = .06
MAX_CYCLE_TRANSLATION_ERROR = .15
MAX_CYCLE_YAW_ERROR = math.radians(3)
_worker_stop_event = None


def initialize_registration_worker(stop_event):
    global _worker_stop_event
    _worker_stop_event = stop_event


def registration_cancelled():
    return _worker_stop_event is not None and _worker_stop_event.is_set()


def angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def compose(a, b):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c*b[0] - s*b[1], a[1] + s*b[0] + c*b[1], angle(a[2]+b[2]))


def inverse(a):
    c, s = math.cos(a[2]), math.sin(a[2])
    return (-c*a[0]-s*a[1], s*a[0]-c*a[1], -a[2])


def transform_points(pose, points):
    c, s = math.cos(pose[2]), math.sin(pose[2])
    return np.asarray(points) @ np.array([[c, s], [-s, c]]) + pose[:2]


@dataclass
class Snapshot:
    robot: str
    boot: str
    epoch: int
    revision: int
    data: np.ndarray
    resolution: float
    origin: tuple

    def __post_init__(self):
        self.data = np.asarray(self.data, dtype=np.int16)
        if (self.data.ndim != 2 or not 0 < self.data.size <= 4_000_000
                or not 0.025 <= self.resolution <= 0.1
                or len(self.origin) != 3 or not np.isfinite(self.origin).all()
                or np.min(self.data) < -1 or np.max(self.data) > 100):
            raise ValueError('invalid snapshot')

    def points(self, pixels):
        return transform_points(self.origin, (np.asarray(pixels)+0.5)*self.resolution)

    def occupied(self, limit=5000, edges=False):
        occupied = self.data >= 65
        if edges:
            occupied &= ~ndimage.binary_erosion(occupied)
        y, x = np.where(occupied)
        step = max(1, int(math.ceil(len(x)/limit)))
        return self.points(np.column_stack([x[::step], y[::step]]))

    def samples(self, points):
        local = transform_points(inverse(self.origin), points)/self.resolution
        cell = np.floor(local).astype(int)
        valid = ((cell[:, 0] >= 0) & (cell[:, 1] >= 0)
                 & (cell[:, 0] < self.data.shape[1]) & (cell[:, 1] < self.data.shape[0]))
        values = np.full(len(points), -1, dtype=np.int16)
        values[valid] = self.data[cell[valid, 1], cell[valid, 0]]
        return values, cell, valid


@dataclass
class Match:
    transform: tuple
    score: float
    overlap: float
    rms: float
    inliers: int
    sigma: float
    yaw_sigma: float


def _features(snapshot):
    image = np.full(snapshot.data.shape, 127, np.uint8)
    known = snapshot.data >= 0
    image[known] = np.rint(255-snapshot.data[known]*2.55).astype(np.uint8)
    mask = ndimage.binary_erosion(known, iterations=2).astype(np.uint8)*255
    orb = cv2.ORB_create(nfeatures=4000, edgeThreshold=8, patchSize=21, fastThreshold=7)
    keys, descriptors = orb.detectAndCompute(image, mask)
    return keys, descriptors


def distinctive_support(points, resolution=.05):
    """Two bare wall faces do not distinguish one unknown room corner.

    Reject support explained by at most two lines, even if their intersection
    numerically constrains SE(2). Raster noise, robot returns and the unequal
    lengths of the visible wall fragments must not manufacture uniqueness.
    Curved or additional extended geometry is allowed; no world axes are used.
    """
    points = np.asarray(points)[::max(1, len(points)//2000)]
    if len(points) < 40:
        return False
    pixels = np.rint((points-points.min(axis=0))/resolution).astype(np.int32)+2
    shape = (int(pixels[:, 1].max())+3, int(pixels[:, 0].max())+3)
    if math.prod(shape) > 4_000_000:
        return False
    image = np.zeros(shape, np.uint8)
    image[pixels[:, 1], pixels[:, 0]] = 255
    lines = cv2.HoughLinesP(image, 1, np.pi/360, threshold=15,
                            minLineLength=max(12, int(.6/resolution)), maxLineGap=5)
    if lines is None:
        return True  # Curved support is not a repeated pair of wall lines.
    remaining = np.ones(len(points), bool)
    for _ in range(2):
        best = np.zeros(len(points), bool)
        for x1, y1, x2, y2 in lines[:, 0]:
            dx, dy = float(x2-x1), float(y2-y1)
            distances = np.abs(dx*(pixels[:, 1]-y1)-dy*(pixels[:, 0]-x1))/math.hypot(dx, dy)
            covered = remaining & (distances*resolution < .15)
            if covered.sum() > best.sum():
                best = covered
        remaining &= ~best
    return bool(remaining.sum() >= max(30, .08*len(points))
                and np.linalg.norm(np.ptp(points[remaining], axis=0)) >= .8)


def assess(a, b, pose):
    scores, rms_values, overlaps, conflicts = [], [], [], []
    for source, target, transform in ((a, b, pose), (b, a, inverse(pose))):
        points = transform_points(transform, target.occupied())
        if len(points) < 20:
            return None
        values, cells, valid = source.samples(points)
        seen = valid & (values >= 0)
        if seen.sum() < 20:
            return None
        distance = ndimage.distance_transform_edt(source.data < 65)*source.resolution
        distances = distance[cells[seen, 1], cells[seen, 0]]
        supported = points[seen][distances < .13]
        # Removing uncertain evidence must not let a tiny matching fragment
        # certify two otherwise unrelated maps. Require actual obstacle
        # support for at least 20% of the target's occupied samples as well.
        if (len(supported) < max(20, .20*len(points))
                or np.linalg.eigvalsh(np.cov(supported.T))[0] < .02):
            return None  # A single observed line cannot determine SE(2).
        if not distinctive_support(supported, source.resolution):
            return None
        # Fusion classifies 26..64 as uncertain, not free. A target obstacle
        # projected far into that band supplies no geometric contradiction.
        # Keep nearby obstacle geometry for precision and ALL definite free
        # contradictions, including those far from any observed obstacle.
        comparable = (values[seen] <= 25) | (distances < .20)
        selected_values = values[seen][comparable]
        distances = distances[comparable]
        scores.append(float(np.exp(-(distances/0.13)**2).mean()))
        # Raw SLAM snapshots include transient unlocalized robots and a few
        # isolated occupied pixels. A bounded 8% trim makes residual robust;
        # their occupied/free conflicts still count below without trimming.
        retained = max(20, int(math.ceil(len(distances)*.92)))
        residual = np.sort(np.minimum(distances, .4))[:retained]
        rms_values.append(float(np.sqrt(np.mean(residual**2))))
        conflicts.append(float(np.mean((distances > 0.20) & (selected_values <= 25))))
        yy, xx = np.where(target.data >= 0)
        stride = max(1, len(xx)//5000)
        observed = transform_points(transform, target.points(np.column_stack([xx[::stride], yy[::stride]])))
        other, _, _ = source.samples(observed)
        overlaps.append(float(np.mean(other >= 0)))
    if min(overlaps) < 0.20 or max(conflicts) > 0.08:
        return None
    return min(scores), min(overlaps), max(rms_values)


def _line_directions(snapshot):
    """Observed wall directions seed rigid hypotheses, never a world axis."""
    image = (snapshot.data >= 65).astype(np.uint8)*255
    lines = cv2.HoughLinesP(image, 1, np.pi/360, threshold=16,
                            minLineLength=max(15, int(.8/snapshot.resolution)), maxLineGap=5)
    if lines is None:
        return []
    bins = {}
    for x1, y1, x2, y2 in lines[:, 0]:
        theta = (math.atan2(float(y2-y1), float(x2-x1))+snapshot.origin[2]) % math.pi
        bucket = int(round(theta/math.radians(3))) % 60
        bins[bucket] = bins.get(bucket, 0)+math.hypot(float(x2-x1), float(y2-y1))
    selected = []
    for key in sorted(bins, key=bins.get, reverse=True):
        theta = key*math.radians(3)
        if all(abs(angle(2*(theta-old))) > math.radians(12) for old in selected):
            selected.append(theta)
        if len(selected) == 3:
            break
    return selected


def _geometry_hypotheses(a, b):
    """Rescue texture-poor ORB matches using observed-wall correlation.

    Every candidate still receives the same bidirectional overlap, conflict,
    rigid refinement and ambiguity checks. Multiple peaks are retained, so a
    repeated corridor or square does not become a forced alignment.
    """
    directions_a, directions_b = _line_directions(a), _line_directions(b)
    yaws = []
    for aa in directions_a:
        for bb in directions_b:
            for extra in (0., math.pi):
                yaw = angle(aa-bb+extra)
                if all(abs(angle(yaw-old)) > math.radians(2) for old in yaws):
                    yaws.append(yaw)
    if not yaws:
        return []
    res = max(a.resolution, b.resolution, .10)
    aa, ao = dynamic_fusion([(a.data, a.resolution, *a.origin, (0., 0., 0.))], res, padding=.1)
    distance = ndimage.distance_transform_edt(aa < 65)*res
    field = np.exp(-(distance/.15)**2).astype(np.float32)
    field[(aa >= 0) & (aa <= 25) & (distance > .25)] = -.20
    field[aa < 0] = 0.
    results = []
    for yaw in yaws:
        if registration_cancelled():
            return []
        bb, bo = dynamic_fusion([(b.data, b.resolution, *b.origin, (0., 0., yaw))], res, padding=.1)
        occupied = (bb >= 65).astype(np.float32)
        if occupied.sum() < 20:
            continue
        correlation = fftconvolve(field, occupied[::-1, ::-1], mode='full')
        for _ in range(6):
            iy, ix = np.unravel_index(np.argmax(correlation), correlation.shape)
            strength = float(correlation[iy, ix])
            if strength < 15:
                break
            shift = (ix-(bb.shape[1]-1), iy-(bb.shape[0]-1))
            pose = (ao[0]+shift[0]*res-bo[0], ao[1]+shift[1]*res-bo[1], yaw)
            results.append((strength, pose))
            gap = int(math.ceil(.65/res))
            correlation[max(0, iy-gap):iy+gap+1, max(0, ix-gap):ix+gap+1] = -np.inf
    # Keep peaks from every observed orientation. A global top-N cap can remove
    # an alternative rotation entirely and falsely make a repeated corner look
    # unique. The number remains bounded by 3*3*2 orientations and six peaks.
    return [pose for _, pose in sorted(results, reverse=True)]


def _refine_edges(source_edges, target_edges, initial, iterations=12):
    tree, pose = cKDTree(source_edges), initial
    for _ in range(iterations):
        if registration_cancelled():
            return pose
        transformed = transform_points(pose, target_edges)
        distances, neighbors = tree.query(transformed)
        good_edges = distances < .20
        if good_edges.sum() < 20:
            break
        fit = least_squares(lambda p: (transform_points(p, target_edges[good_edges])
                          - source_edges[neighbors[good_edges]]).ravel(),
                            pose, loss='huber', f_scale=.04, max_nfev=15)
        pose = (float(fit.x[0]), float(fit.x[1]), angle(float(fit.x[2])))
    return pose


def _refine_edges_bidirectional(source_edges, target_edges, initial, iterations=12):
    """Refine both observed surfaces with the same rigid SE(2) transform.

    One-way nearest-neighbor fits can bias the pose toward the more complete
    surface. Assessing both directions afterwards does not remove that bias.
    Only already plausible basins reach this bounded refinement; their original
    candidates remain available for the ambiguity comparison.
    """
    source_tree, target_tree = cKDTree(source_edges), cKDTree(target_edges)
    pose = initial
    for _ in range(iterations):
        if registration_cancelled():
            return pose
        forward, forward_neighbors = source_tree.query(transform_points(pose, target_edges))
        backward, backward_neighbors = target_tree.query(transform_points(inverse(pose), source_edges))
        forward_mask, backward_mask = forward < .20, backward < .20
        if min(forward_mask.sum(), backward_mask.sum()) < 20:
            break

        def residual(p):
            return np.concatenate([
                (transform_points(p, target_edges[forward_mask])
                 - source_edges[forward_neighbors[forward_mask]]).ravel(),
                (transform_points(inverse(p), source_edges[backward_mask])
                 - target_edges[backward_neighbors[backward_mask]]).ravel()])

        fit = least_squares(residual, pose, loss='huber', f_scale=.04, max_nfev=15)
        pose = (float(fit.x[0]), float(fit.x[1]), angle(float(fit.x[2])))
    return pose


_symmetry_cache = {}


def intrinsic_ambiguity(snapshot):
    """A repeated room is not made unique by two differently noisy rasters.

    Look for a substantial alternative placement of the observed map against
    itself, including free-space conflicts. No canvas/world edge is a wall.
    """
    classes = np.where(snapshot.data < 0, 0, np.where(snapshot.data <= 25, 1,
                       np.where(snapshot.data < 65, 2, 3))).astype(np.uint8)
    key = (snapshot.resolution, classes.shape, hashlib.sha256(classes.tobytes()).digest())
    if key in _symmetry_cache:
        return _symmetry_cache[key]
    edges = snapshot.occupied(3000, edges=True)
    result = False
    if len(edges) >= 40:
        centre = np.mean(edges, axis=0)[None, :]
        for initial in _geometry_hypotheses(snapshot, snapshot):
            if registration_cancelled():
                return False
            if (np.linalg.norm(transform_points(initial, centre)-centre) < .8
                    and abs(angle(initial[2])) < math.radians(20)):
                continue
            pose = _refine_edges(edges, edges, initial, 6)
            if (np.linalg.norm(transform_points(pose, centre)-centre) < .8
                    and abs(angle(pose[2])) < math.radians(20)):
                continue
            quality = assess(snapshot, snapshot, pose)
            if quality and quality[0] >= .85 and quality[1] >= .65 and quality[2] <= .10:
                result = True
                break
    if len(_symmetry_cache) >= 12:
        _symmetry_cache.pop(next(iter(_symmetry_cache)))
    _symmetry_cache[key] = result
    return result


def register(a, b, diagnostics=None):
    """Return source-from-target or None; ambiguity is never forced to a pose."""
    if diagnostics is not None:
        diagnostics.clear()
        diagnostics.update(feature_matches=0, hypotheses=0,
                           assessments={'unsupported': 0, 'below_floor': 0, 'plausible': 0},
                           best_evaluated=[], candidates=[])

    def finish(reason, match=None):
        if diagnostics is not None:
            diagnostics['reason'] = reason
        return match

    def evaluate(pose):
        stats = assess(a, b, pose)
        if diagnostics is not None:
            category = ('unsupported' if stats is None else 'below_floor'
                        if stats[0] < .68 or stats[2] > .12 else 'plausible')
            diagnostics['assessments'][category] += 1
            if stats is not None:
                best = diagnostics['best_evaluated']
                best.append(dict(transform=[float(v) for v in pose],
                                 score=stats[0], overlap=stats[1], rms=stats[2]))
                best.sort(key=lambda item: (item['score'], -item['rms']), reverse=True)
                del best[4:]
        return stats

    cv2.setNumThreads(1)
    if registration_cancelled():
        return finish('cancelled')
    if not distinctive_support(a.occupied(edges=True), a.resolution) or not distinctive_support(b.occupied(edges=True), b.resolution):
        return finish('insufficient_distinctive_geometry')
    if intrinsic_ambiguity(a) or intrinsic_ambiguity(b):
        return finish('intrinsic_ambiguity')
    if registration_cancelled():
        return finish('cancelled')
    ka, da = _features(a)
    kb, db = _features(b)
    pairs = cv2.BFMatcher(cv2.NORM_HAMMING).knnMatch(db, da, k=2) if da is not None and db is not None else []
    good = [m for pair in pairs if len(pair) == 2 for m, n in [pair]
            if m.distance < 0.75*n.distance]
    if diagnostics is not None:
        diagnostics['feature_matches'] = len(good)
    pa = a.points([ka[m.trainIdx].pt for m in good]) if good else np.empty((0, 2))
    pb = b.points([kb[m.queryIdx].pt for m in good]) if good else np.empty((0, 2))
    # Fit SE(2) itself, not a similarity transform whose spurious fitted scale
    # can hide a valid rotation or distort the map. Keep separate RANSAC basins.
    rng = np.random.default_rng(173)
    hypotheses = []
    for i, j in rng.integers(0, max(1, len(pa)), size=(2500 if len(pa) >= 12 else 0, 2)):
        if registration_cancelled():
            return finish('cancelled')
        va, vb = pa[j]-pa[i], pb[j]-pb[i]
        if min(np.linalg.norm(va), np.linalg.norm(vb)) < 1.0:
            continue
        th = angle(math.atan2(va[1], va[0])-math.atan2(vb[1], vb[0]))
        shift = pa[i]-transform_points((0, 0, th), pb[i:i+1])[0]
        initial = (shift[0], shift[1], th)
        indices = np.where(np.linalg.norm(transform_points(initial, pb)-pa, axis=1) < 0.15)[0]
        if len(indices) < 12 or np.linalg.norm(np.ptp(pb[indices], axis=0)) < 1.5:
            continue
        similar = next((k for k, (_, p, _) in enumerate(hypotheses)
                        if math.dist(p[:2], initial[:2]) < .3
                        and abs(angle(p[2]-th)) < math.radians(8)), None)
        candidate = (len(indices), initial, indices)
        if similar is not None:
            if len(indices) > hypotheses[similar][0]:
                hypotheses[similar] = candidate
        else:
            hypotheses.append(candidate)
        hypotheses.sort(key=lambda x: x[0], reverse=True)
        del hypotheses[8:]
    hypotheses.extend((0, pose, np.array([], dtype=int)) for pose in _geometry_hypotheses(a, b))
    if diagnostics is not None:
        diagnostics['hypotheses'] = len(hypotheses)
    candidates = []
    source_edges = a.occupied(5000, edges=True)
    target_edges = b.occupied(5000, edges=True)
    if min(len(source_edges), len(target_edges)) < 20:
        return finish('insufficient_edges')
    centre = np.mean(target_edges, axis=0)[None, :]
    for _, initial, indices in hypotheses:
        if registration_cancelled():
            return finish('cancelled')
        pose = initial
        if len(indices):
            result = least_squares(lambda p: (transform_points(p, pb[indices])-pa[indices]).ravel(),
                                   initial, loss='huber', f_scale=0.04, max_nfev=50)
            pose = (float(result.x[0]), float(result.x[1]), angle(float(result.x[2])))
        pose = _refine_edges(source_edges, target_edges, pose)
        stats = evaluate(pose)
        if stats is None or stats[0] < .68 or stats[2] > .12:
            continue
        # Never claim zero covariance just because pixels happen to coincide.
        sigma = max(0.025, min(0.10, stats[2]))
        match = Match(pose, *stats, len(indices), sigma, math.radians(1.0))
        # Compare basins at the observed map centre, not at a possibly distant
        # local-frame origin. Small ICP perturbations are not a second solution.
        similar = next((i for i, p in enumerate(candidates)
                        if np.linalg.norm(transform_points(pose, centre)
                                          - transform_points(p.transform, centre)) < .5
                        and abs(angle(pose[2]-p.transform[2])) < math.radians(10)), None)
        if similar is None:
            candidates.append(match)
        elif match.score > candidates[similar].score:
            candidates[similar] = match
    # Improve each plausible basin using the bidirectional objective. Retain
    # its original evidence even if joint refinement loses support, so a rival
    # cannot disappear just because a precision-improvement step failed.
    for original in tuple(candidates):
        if registration_cancelled():
            return finish('cancelled')
        pose = _refine_edges_bidirectional(source_edges, target_edges, original.transform)
        stats = evaluate(pose)
        if stats is None or stats[0] < .68 or stats[2] > .12:
            continue
        match = Match(pose, *stats, original.inliers, max(.025, min(.10, stats[2])), math.radians(1.0))
        similar = next((i for i, p in enumerate(candidates)
                        if np.linalg.norm(transform_points(pose, centre)
                                          - transform_points(p.transform, centre)) < .5
                        and abs(angle(pose[2]-p.transform[2])) < math.radians(10)), None)
        if similar is None:
            candidates.append(match)
        elif match.score > candidates[similar].score:
            candidates[similar] = match
    if not candidates:
        return finish('no_plausible_candidate')
    candidates.sort(key=lambda m: (m.score, m.overlap, m.inliers), reverse=True)
    if diagnostics is not None:
        diagnostics['candidates'] = [dict(transform=[float(v) for v in m.transform],
            score=m.score, overlap=m.overlap, rms=m.rms) for m in candidates[:4]]
    if len(candidates) > 1 and candidates[0].score-candidates[1].score < 0.08:
        return finish('ambiguous_candidates')
    # Raising the acceptance gate must never erase a plausible rival basin.
    # Keep the original broad evidence floor for uniqueness, then require the
    # winning transform itself to satisfy the stricter precision gate.
    if candidates[0].score < MIN_MATCH_SCORE:
        return finish('low_score')
    if candidates[0].rms > MAX_MATCH_RMS:
        return finish('high_residual')
    return finish('accepted', candidates[0])


def registration_job(a, b):
    """Bounded worker diagnostics travel with the exact input map pair."""
    diagnostics = {}
    return {'match': register(a, b, diagnostics), 'diagnostics': diagnostics}


class ConstraintGraph:
    """Replicated, boot-qualified SE(2) graph. No root process is required."""
    def __init__(self):
        self.edges = {}
        self.sigmas = {}
        self.anchor = None
        self.refinement_cache = None

    def _tree_poses(self, root):
        result = {tuple(root): (0., 0., 0.)}
        pending = True
        while pending:
            pending = False
            for (a, b), (pose, _) in sorted(self.edges.items()):
                if a in result and b not in result:
                    result[b] = compose(result[a], pose)
                    pending = True
                elif b in result and a not in result:
                    result[a] = compose(result[b], inverse(pose))
                    pending = True
        return result

    @staticmethod
    def _refine_component(tree, edges):
        anchor = min(tree)
        nodes = sorted(key for key in tree if key != anchor)
        initial = np.concatenate([tree[key] for key in nodes])

        def unpack(values):
            poses = {anchor: (0., 0., 0.)}
            poses.update((key, tuple(values[3*i:3*i+3])) for i, key in enumerate(nodes))
            return poses

        def errors(poses):
            for a, b, observed, sigma, yaw_sigma in edges:
                predicted = compose(inverse(poses[a]), poses[b])
                yield (predicted[0]-observed[0], predicted[1]-observed[1],
                       angle(predicted[2]-observed[2]), sigma, yaw_sigma)

        def residual(values):
            result = []
            for dx, dy, dyaw, sigma, yaw_sigma in errors(unpack(values)):
                result.extend((dx/max(sigma, .025), dy/max(sigma, .025),
                               dyaw/max(yaw_sigma, math.radians(.5))))
            return result

        try:
            fit = least_squares(residual, initial, loss='soft_l1', max_nfev=30)
        except (ValueError, FloatingPointError):
            return None
        if not fit.success or not np.isfinite(fit.x).all():
            return None
        result = unpack(fit.x)
        if any(math.hypot(dx, dy) > MAX_CYCLE_TRANSLATION_ERROR
               or abs(dyaw) > MAX_CYCLE_YAW_ERROR for dx, dy, dyaw, _, _ in errors(result)):
            return None
        return result

    def poses(self, root=None):
        root = root or self.anchor
        if root is None:
            return {}
        root = tuple(root)
        tree = self._tree_poses(root)
        edges = tuple((a, b, measured, *self.sigmas.get((a, b), (.05, math.radians(1))))
                      for (a, b), (measured, _) in sorted(self.edges.items()) if a in tree and b in tree)
        # A chain already satisfies its measurements. Only a genuine cycle
        # contributes redundant evidence for refining the relative origins.
        if len({frozenset((a, b)) for a, b, *_ in edges}) < len(tree):
            return tree
        if self.refinement_cache is None or self.refinement_cache[0] != edges:
            canonical = min(tree)
            self.refinement_cache = edges, self._refine_component(self._tree_poses(canonical), edges)
        solution = self.refinement_cache[1]
        if solution is None:
            return tree
        from_canonical = inverse(solution[root])
        result = {key: compose(from_canonical, value) for key, value in solution.items()}
        result[root] = (0., 0., 0.)
        return result

    def uncertainties(self, root=None):
        root = root or self.anchor
        if root is None:
            return {}
        result = {tuple(root): (0., 0.)}
        pending = True
        while pending:
            pending = False
            for (a, b), (pose, _) in sorted(self.edges.items()):
                sigma, yaw_sigma = self.sigmas.get((a, b), (.05, math.radians(1)))
                for source, target in ((a, b), (b, a)):
                    if source in result and target not in result:
                        old, old_yaw = result[source]
                        result[target] = (math.sqrt(old*old+sigma*sigma+(math.hypot(*pose[:2])*old_yaw)**2),
                                          math.hypot(old_yaw, yaw_sigma))
                        pending = True
        return result

    def add(self, source, target, transform, score=1.0, sigma=.05, yaw_sigma=math.radians(1)):
        source, target = tuple(source), tuple(target)
        if (source == target or not np.isfinite(transform).all() or len(transform) != 3
                or not math.isfinite(sigma) or not math.isfinite(yaw_sigma) or min(sigma, yaw_sigma) < 0):
            return False
        # Keep the original cycle admission fence. A joint fit must not hide
        # a new measurement that disagrees with the already accepted path.
        existing = self._tree_poses(source)
        if target in existing:
            predicted = existing[target]
            if (math.dist(predicted[:2], transform[:2]) > MAX_CYCLE_TRANSLATION_ERROR
                    or abs(angle(predicted[2]-transform[2])) > MAX_CYCLE_YAW_ERROR):
                return False
        old = self.edges.get((source, target))
        if old and old[1] > score + 0.1:
            return False
        self.edges[(source, target)] = (tuple(transform), score)
        self.sigmas[(source, target)] = sigma, yaw_sigma
        if self.anchor is None:
            self.anchor = min(source, target)
        return True


def anchor_key(poses, robot, boot):
    matches = [key for key in poses if key[:2] == (robot, boot)]
    return matches[0] if len(matches) == 1 else None


def dynamic_fusion(layers, resolution=0.05, padding=0.75, max_cells=1_000_000):
    """Fuse into an observed-extent canvas; no world-coordinate arena is used."""
    if not layers:
        return None
    corners = []
    for data, res, ox, oy, oyaw, pose in layers:
        yy, xx = np.where(np.asarray(data) >= 0)
        if not len(xx):
            continue
        bounds = [[xx.min()*res, yy.min()*res], [(xx.max()+1)*res, yy.min()*res],
                  [(xx.max()+1)*res, (yy.max()+1)*res], [xx.min()*res, (yy.max()+1)*res]]
        corners.extend(transform_points(compose(pose, (ox, oy, oyaw)), bounds))
    if not corners:
        return None
    lo = np.floor((np.min(corners, axis=0)-padding)/resolution)*resolution
    hi = np.ceil((np.max(corners, axis=0)+padding)/resolution)*resolution
    width, height = np.rint((hi-lo)/resolution).astype(int)
    if width <= 0 or height <= 0 or width*height > max_cells:
        raise ValueError('fused canvas exceeds configured resource limit')
    return fuse(layers, (height, width), resolution, tuple(lo)), tuple(lo)


def voronoi_mask(shape, resolution, origin, seeds, owner, margin=0.35):
    yy, xx = np.indices(shape)
    x, y = origin[0]+(xx+0.5)*resolution, origin[1]+(yy+0.5)*resolution
    keys = sorted(seeds)
    distances = np.stack([(x-seeds[n][0])**2+(y-seeds[n][1])**2 for n in keys])
    allowed = np.argmin(distances, axis=0) == keys.index(owner)
    return ndimage.distance_transform_edt(allowed)*resolution >= margin
