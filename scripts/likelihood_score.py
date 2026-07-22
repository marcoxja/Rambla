"""M7 Phase 2.5 Task 4 diagnostic (part 2/2): offline likelihood-field
scoring of captured snapshots (scripts/likelihood_capture.py's output).

Answers the question Task 4 exists to settle: for a scan AMCL used to
confidently converge to the WRONG pose, does the ground-truth pose score
better under AMCL's own measurement model (a sharpness/tuning problem --
Task 5's sigma_hit/z_hit lever) or comparably/worse (genuine perceptual
aliasing -- no AMCL parameter tuning fixes that)?

Deliberately reimplements nav2_amcl's actual LikelihoodFieldModel scoring
(not a generic/textbook likelihood field), verified against nav2_amcl's own
source (sensors/laser/laser.cpp): for each of up to `max_beams` beams
(subsampled by the same fixed stride nav2_amcl uses), skip NaN/max-range
readings entirely, project the beam endpoint from the CANDIDATE pose into
map cells, look up the precomputed distance-to-nearest-occupied-cell field
(clamped at laser_likelihood_max_dist, matching amcl.yaml), and accumulate
    pz = z_hit * exp(-(d*d) / (2*sigma_hit**2)) + z_rand / range_max
    score += pz ** 3
across beams -- the cubing is a real, deliberate nav2_amcl idiosyncrasy
(decorrelates beams / limits any one beam's influence), not a bug to
"clean up" here. Reading z_short/z_max/lambda_short from amcl.yaml would be
wrong: those belong to the alternate `likelihood_field_prob`/beam models,
not `laser_model_type: likelihood_field`, which is what this deployment
actually runs -- they play no part in the real score and are intentionally
not used below.

Ground-truth pose comes from each snapshot's Gazebo world-frame pose plus
the confirmed pure-translation map<->world offset from
M7_LOCALIZATION_FINDINGS.md section 3 (+3.93, -2.92; four independent wall
features agreed to ~5cm) -- no rotation term, matching that section's own
methodology.

Usage (map.yaml/map.pgm/amcl.yaml paths default to the real rambla-vm
layout; run ON rambla-vm where scipy/numpy are already installed -- see
rambla_likelihood_score in scripts/vm.sh):
    python3 likelihood_score.py --snapshots snapshots.json
"""
import argparse
import json
import math
import re

import numpy as np
from scipy.ndimage import distance_transform_edt

# Confirmed 2026-07-19 (M7_LOCALIZATION_FINDINGS.md section 3): map_frame =
# world_frame + this offset, pure translation, no rotation term.
MAP_MINUS_WORLD_OFFSET = (3.93, -2.92)

OCCUPIED, FREE, UNKNOWN = 1, 0, 2


def read_map_yaml(path):
    fields = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or ':' not in line:
                continue
            key, _, value = line.partition(':')
            fields[key.strip()] = value.strip()
    return fields


def read_pgm_raster(image_path):
    """Same hand-rolled raw-P5 reader as scripts/validate_map.py (kept
    dependency-free of that script so this file can be piped over SSH
    stdin standalone, matching that file's own precedent of not importing
    across these one-off scripts)."""
    with open(image_path, 'rb') as f:
        data = f.read()
    if not data.startswith(b'P5'):
        raise ValueError(f'unsupported map image format: {data[:2]!r} (expected raw P5 PGM)')
    idx = 2
    tokens = []
    while len(tokens) < 3:
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        if idx < len(data) and data[idx:idx + 1] == b'#':
            while idx < len(data) and data[idx:idx + 1] != b'\n':
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        tokens.append(data[start:idx])
    idx += 1
    width, height, maxval = (int(t) for t in tokens)
    pixels = data[idx:idx + width * height]
    if len(pixels) != width * height:
        raise RuntimeError(f'{image_path}: raster truncated')
    return width, height, maxval, pixels


def classify_pixels(width, height, maxval, pixels, negate, free_thresh, occupied_thresh):
    arr = np.frombuffer(pixels, dtype=np.uint8).astype(np.float64).reshape(height, width)
    normalized = (arr / maxval) if negate else ((maxval - arr) / maxval)
    classes = np.full((height, width), UNKNOWN, dtype=np.uint8)
    classes[normalized > occupied_thresh] = OCCUPIED
    classes[normalized < free_thresh] = FREE
    return classes


def read_amcl_params(path):
    """amcl.yaml is a flat `key: value` block under `amcl: ros__parameters:`
    -- hand-parsed the same dependency-free way as the map.yaml reader
    above, since only a handful of scalar keys are needed."""
    wanted = {
        'z_hit', 'z_rand', 'sigma_hit', 'laser_likelihood_max_dist', 'max_beams',
    }
    params = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if ':' not in line or line.startswith('#'):
                continue
            key, _, value = line.partition(':')
            key = key.strip()
            if key in wanted:
                params[key] = float(value.split('#')[0].strip())
    missing = wanted - params.keys()
    if missing:
        raise RuntimeError(f'{path}: missing expected amcl params {missing}')
    params['max_beams'] = int(params['max_beams'])
    return params


class LikelihoodField:
    def __init__(self, map_yaml_path, max_occ_dist):
        fields = read_map_yaml(map_yaml_path)
        self.resolution = float(fields['resolution'])
        origin = [float(v) for v in fields['origin'].strip('[]').split(',')]
        self.origin_x, self.origin_y = origin[0], origin[1]
        negate = int(float(fields.get('negate', '0'))) != 0
        free_thresh = float(fields.get('free_thresh', '0.196'))
        occupied_thresh = float(fields.get('occupied_thresh', '0.65'))

        import os
        image_path = os.path.join(
            os.path.dirname(map_yaml_path), fields['image'])
        width, height, maxval, pixels = read_pgm_raster(image_path)
        self.width, self.height = width, height
        classes = classify_pixels(
            width, height, maxval, pixels, negate, free_thresh, occupied_thresh)

        occupied_mask = classes == OCCUPIED
        # distance_transform_edt gives, for each False cell, the pixel
        # distance to the nearest True cell -- invert the mask so it's
        # "distance to nearest occupied cell", matching AMCL's own
        # convertMap()/map_cspace precomputed occ_dist field.
        dist_px = distance_transform_edt(~occupied_mask)
        self.occ_dist = np.minimum(dist_px * self.resolution, max_occ_dist)

    def world_to_image_rc(self, wx, wy):
        # image row 0 is the TOP of the raw PGM raster; map.yaml's origin is
        # the world coordinate of the BOTTOM-left cell (map_server's
        # OccupancyGrid convention) -- flip row so this stays consistent
        # with how classify_pixels/occ_dist were built directly from the
        # raw (unflipped) PGM bytes.
        col = int(math.floor((wx - self.origin_x) / self.resolution))
        occ_row = int(math.floor((wy - self.origin_y) / self.resolution))
        row = self.height - 1 - occ_row
        return row, col

    def distance_at(self, wx, wy):
        row, col = self.world_to_image_rc(wx, wy)
        if 0 <= row < self.height and 0 <= col < self.width:
            return self.occ_dist[row, col]
        return None  # out of map bounds


def score_pose(field, params, pose, scan):
    """Faithful reimplementation of nav2_amcl's LikelihoodFieldModel scoring
    (see module docstring) for one candidate (x, y, yaw) pose against one
    scan. Returns (score, beams_used)."""
    x, y, yaw = pose
    ranges = scan['ranges']
    n = len(ranges)
    max_beams = params['max_beams']
    range_max = scan['range_max']
    sigma_hit = params['sigma_hit']
    z_hit = params['z_hit']
    z_rand = params['z_rand']

    step = 1 if max_beams < 2 else max(1, (n - 1) // (max_beams - 1))

    score = 0.0
    beams_used = 0
    for i in range(0, n, step):
        r = ranges[i]
        if r is None or math.isnan(r) or math.isinf(r) or r >= range_max:
            continue
        angle = scan['angle_min'] + i * scan['angle_increment']
        bx = x + r * math.cos(yaw + angle)
        by = y + r * math.sin(yaw + angle)
        d = field.distance_at(bx, by)
        if d is None:
            d = params['laser_likelihood_max_dist']
        pz = z_hit * math.exp(-(d * d) / (2 * sigma_hit * sigma_hit))
        pz += z_rand / range_max
        score += pz ** 3
        beams_used += 1
    return score, beams_used


def analyze_snapshot(field, params, snapshot):
    gt = snapshot['ground_truth_world']
    amcl = snapshot['amcl_pose']
    scan = snapshot['scan']
    if gt is None or amcl is None or scan is None:
        return {'skipped': True, 'reason': 'missing ground_truth/amcl_pose/scan'}

    true_pose_map = (
        gt[0] + MAP_MINUS_WORLD_OFFSET[0],
        gt[1] + MAP_MINUS_WORLD_OFFSET[1],
        gt[2],
    )
    amcl_pose_map = tuple(amcl)

    true_score, true_beams = score_pose(field, params, true_pose_map, scan)
    amcl_score, amcl_beams = score_pose(field, params, amcl_pose_map, scan)

    err_m = math.hypot(
        true_pose_map[0] - amcl_pose_map[0], true_pose_map[1] - amcl_pose_map[1])
    correct = err_m < 0.5

    if amcl_score <= 1e-300:
        ratio = float('inf') if true_score > 0 else 1.0
    else:
        ratio = true_score / amcl_score

    verdict = (
        'AMCL_AT_TRUTH (control)' if correct else
        'TRUTH_SCORES_HIGHER (tuning/sampling issue)' if ratio > 2.0 else
        'COMPARABLE_OR_WRONG_HIGHER (content aliasing)'
    )

    return {
        'localization_status': snapshot['localization_status'],
        'true_pose_map': [round(v, 3) for v in true_pose_map],
        'amcl_pose_map': [round(v, 3) for v in amcl_pose_map],
        'error_m': round(err_m, 3),
        'amcl_was_correct': correct,
        'true_score': true_score,
        'amcl_score': amcl_score,
        'true_beams_used': true_beams,
        'amcl_beams_used': amcl_beams,
        'true_over_amcl_ratio': ratio if math.isfinite(ratio) else None,
        'verdict': verdict,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--snapshots', required=True, help='path to likelihood_capture.py JSON output')
    parser.add_argument(
        '--map-yaml', default='/home/rambla_vm/ros2_ws/maps/active/map.yaml')
    parser.add_argument(
        '--amcl-yaml',
        default='/home/rambla_vm/ros2_ws/install/rambla_localization/share/rambla_localization/config/amcl.yaml')
    parser.add_argument('--report-path', default=None)
    args = parser.parse_args()

    with open(args.snapshots) as f:
        data = json.load(f)

    params = read_amcl_params(args.amcl_yaml)
    field = LikelihoodField(args.map_yaml, params['laser_likelihood_max_dist'])

    results = []
    for i, snap in enumerate(data['snapshots']):
        r = analyze_snapshot(field, params, snap)
        r['snapshot_index'] = i
        results.append(r)
        print(f"--- snapshot {i} ---")
        for k, v in r.items():
            print(f'  {k}: {v}')
        print()

    n_wrong = sum(1 for r in results if not r.get('skipped') and not r.get('amcl_was_correct'))
    n_truth_higher = sum(
        1 for r in results
        if not r.get('skipped') and not r.get('amcl_was_correct')
        and 'TRUTH_SCORES_HIGHER' in r.get('verdict', ''))
    print('==== summary ====')
    print(f'  {len(results)} snapshots, {n_wrong} confidently-wrong, '
          f'{n_truth_higher}/{n_wrong} of those score truth higher (>2x)')

    report_path = args.report_path or 'likelihood_diagnostic_report.json'
    with open(report_path, 'w') as f:
        json.dump({'params': params, 'results': results}, f, indent=2)
    print(f'  report: {report_path}')


if __name__ == '__main__':
    main()
