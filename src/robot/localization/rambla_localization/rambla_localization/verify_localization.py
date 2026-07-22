"""M5 localization verification harness: deterministic, scripted scenarios
that emit a small event log + a verification_report.json verdict, instead of
a human or an AI watching raw per-tick telemetry live.

Two real bugs slipped through the earlier manual verification precisely
because the stack said nothing about its own reasoning: AMCL was launched
against a world whose footprint didn't match the active map (no warning
anywhere), and localization_probe silently exhausted its bounded rotation
attempts while stuck in GLOBAL (no signal anywhere). This harness turns both
into named, explicit failures:
  - run_preflight() refuses to start a scenario if the active map's
    footprint doesn't roughly match --world's known extents.
  - wait_for_state()'s stall watchdog fails fast with "STALLED: probe
    exhausted" instead of waiting out the full convergence timeout while
    /probe_status silently reports EXHAUSTED.

Usage (on rambla-vm, sim + localize.launch.py already running):
    ros2 run rambla_localization verify_localization --scenario cold_start --world house
    ros2 run rambla_localization verify_localization --scenario kidnap --world house
    ros2 run rambla_localization verify_localization --scenario ekf_drift --world house

Ground truth and the kidnap teleport both go through Gazebo Transport (`gz
topic`/`gz service`, not ROS2) since Gazebo's absolute world-frame pose has
no ROS2 topic equivalent here. The accuracy checks use delta displacement
(change in AMCL pose vs change in ground truth over a commanded motion)
rather than comparing absolute positions directly, because the map frame's
origin is wherever the M3 recording bag started, not the Gazebo world
origin - a fixed frame offset that cancels out in a delta but would look
like a large "error" in an absolute comparison.
"""
import argparse
import json
import math
import os
import re
import struct
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String

from rambla_localization.localization_state import (
    CONVERGING,
    DEGRADED,
    GLOBAL,
    LOST,
    USABLE,
)
from rambla_localization.probe_behavior import EXHAUSTED

REQUIRED_TOPICS = [
    '/localization_status',
    '/localization_metrics',
    '/amcl_pose',
    '/particle_cloud',
    '/probe_status',
    '/odometry/filtered',
]

# World footprints (x_span_m, y_span_m) - derived from grepping wall <pose>
# entries in house.sdf during M5 verification. Kept here rather than
# imported cross-package: rambla_localization has no dependency on
# rambla_sim's world files, and this is only used for a coarse
# footprint-size sanity check, not exact geometry.
WORLD_BOUNDS_M = {
    'house': (16.0, 12.0),
}
MAP_WORLD_SIZE_TOLERANCE = 0.3  # fractional; either axis differing by more flags a mismatch

# Default kidnap targets: a point far from the world's usual spawn, clear of
# the wall footprint. Override with --kidnap-x/-y/-z/-yaw if the model ends
# up embedded in an obstacle for a given world's interior layout.
KIDNAP_DEFAULTS = {
    'house': (4.5, -3.5, 0.05, 0.0),
}


class EventLog:
    """Small, human-and-AI-readable event log: one line per meaningful
    event (state transition, checkpoint, watchdog trip) - not a per-tick
    telemetry dump. Doubles as the ordered 'events' list in the JSON
    report."""

    def __init__(self):
        self._t0 = time.time()
        self.events = []

    def log(self, msg, **fields):
        t = round(time.time() - self._t0, 1)
        entry = {'t': t, 'msg': msg}
        entry.update(fields)
        self.events.append(entry)
        extra = ' '.join(f'{k}={v}' for k, v in fields.items())
        line = f'[{t:7.1f}s] {msg}'
        if extra:
            line += f' ({extra})'
        print(line, flush=True)


class Watcher(Node):
    """Subscribes to every signal the harness needs, tracks last-seen time
    per required topic (for the preflight liveness check), and exposes the
    latest decoded value of each - no raw-history buffering."""

    def __init__(self):
        super().__init__('verify_localization')
        self.last_seen = {t: None for t in REQUIRED_TOPICS}
        self.status = None
        self.metrics = None
        self.probe = None
        self.amcl_xy = None
        self.odom_xy = None

        self.create_subscription(
            String, '/localization_status', self._on_status, 10)
        self.create_subscription(
            String, '/localization_metrics', self._on_metrics, 10)
        self.create_subscription(
            String, '/probe_status', self._on_probe, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)
        self.create_subscription(
            Odometry, '/odometry/filtered', self._on_odom, 10)
        self.create_subscription(
            ParticleCloud, '/particle_cloud', self._on_particles,
            qos_profile_sensor_data)
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel_raw', 10)

    def _seen(self, topic):
        self.last_seen[topic] = time.time()

    def _on_status(self, msg):
        self._seen('/localization_status')
        self.status = msg.data

    def _on_metrics(self, msg):
        self._seen('/localization_metrics')
        try:
            self.metrics = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_probe(self, msg):
        self._seen('/probe_status')
        try:
            self.probe = json.loads(msg.data)
        except json.JSONDecodeError:
            pass

    def _on_amcl(self, msg):
        self._seen('/amcl_pose')
        p = msg.pose.pose.position
        self.amcl_xy = (p.x, p.y)

    def _on_odom(self, msg):
        self._seen('/odometry/filtered')
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)

    def _on_particles(self, _msg):
        self._seen('/particle_cloud')

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=max(min(0.2, end - time.time()), 0.0))


# --- Gazebo Transport helpers (ground truth + kidnap teleport) -------------

def gz_ground_truth(model):
    try:
        out = subprocess.run(
            ['gz', 'topic', '-e', '-t', f'/model/{model}/pose', '-n', '1'],
            capture_output=True, text=True, timeout=5,
        ).stdout
    except Exception:
        return None
    pos = re.search(
        r'position\s*\{\s*x:\s*(-?[\d.eE+-]+)\s*y:\s*(-?[\d.eE+-]+)', out)
    ori = re.search(
        r'orientation\s*\{[^}]*z:\s*(-?[\d.eE+-]+)\s*w:\s*(-?[\d.eE+-]+)', out)
    if not pos or not ori:
        return None
    gx, gy = float(pos.group(1)), float(pos.group(2))
    gz_, gw = float(ori.group(1)), float(ori.group(2))
    gyaw = 2 * math.atan2(gz_, gw)
    return gx, gy, gyaw


def gz_teleport(world, model, x, y, z, yaw):
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)
    req = (
        f'name: "{model}" position: {{x: {x} y: {y} z: {z}}} '
        f'orientation: {{z: {qz} w: {qw}}}'
    )
    result = subprocess.run(
        ['gz', 'service', '-s', f'/world/{world}/set_pose',
         '--reqtype', 'gz.msgs.Pose', '--reptype', 'gz.msgs.Boolean',
         '--timeout', '2000', '--req', req],
        capture_output=True, text=True, timeout=5,
    )
    return 'true' in result.stdout.lower()


# --- Map / world footprint preflight ---------------------------------------

def read_map_yaml(path):
    # map_saver_cli's output is flat key: value YAML (no nesting) - same
    # hand-parse convention as scripts/validate_map.py, so this stays
    # dependency-free (no pyyaml requirement on rambla-vm).
    fields = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or ':' not in line:
                continue
            key, _, value = line.partition(':')
            fields[key.strip()] = value.strip()
    return fields


def read_image_dimensions(image_path):
    with open(image_path, 'rb') as f:
        header = f.read(8)
        if header.startswith(b'\x89PNG'):
            f.seek(16)
            w, h = struct.unpack('>II', f.read(8))
            return w, h
        f.seek(0)
        data = f.read(512)
    magic = data[:2]
    if magic not in (b'P5', b'P2'):
        raise ValueError(f'unsupported map image format: {magic!r}')
    tokens = []
    idx = 2
    while len(tokens) < 2 and idx < len(data):
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        if idx < len(data) and data[idx:idx + 1] == b'#':
            while idx < len(data) and data[idx:idx + 1] != b'\n':
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        if idx > start:
            tokens.append(data[start:idx])
    return int(tokens[0]), int(tokens[1])


def compute_map_bounds(map_yaml_path):
    fields = read_map_yaml(map_yaml_path)
    resolution = float(fields['resolution'])
    image_name = fields['image']
    image_path = os.path.join(os.path.dirname(map_yaml_path), image_name)
    width_px, height_px = read_image_dimensions(image_path)
    origin = [float(v) for v in fields['origin'].strip('[]').split(',')]
    return {
        'resolution': resolution,
        'width_m': width_px * resolution,
        'height_m': height_px * resolution,
        'origin_x': origin[0],
        'origin_y': origin[1],
    }


def check_map_world_match(map_bounds, world):
    """One-sided check: a SLAM-produced map only covers the floor area the
    mapping run actually explored, so it is normal and expected for the map
    footprint to be SMALLER than the world's full wall-to-wall extent - a
    two-sided tolerance would flag every legitimate map as a mismatch. The
    real, previously-missed failure mode is the opposite: a map raster
    LARGER than the room it's launched against (M5 verification found
    map_v001, recorded in `house`, launched against the much smaller
    `apartment_world` - AMCL could structurally never converge). So this
    only refuses when the map exceeds the world's footprint, with a small
    margin for wall-thickness/measurement slack."""
    world_span = WORLD_BOUNDS_M.get(world)
    if world_span is None:
        return None, f'no known bounds for world={world!r} - skipping size check'
    world_w, world_h = world_span
    map_w, map_h = map_bounds['width_m'], map_bounds['height_m']
    margin = 1.0 + MAP_WORLD_SIZE_TOLERANCE
    mismatch = map_w > world_w * margin or map_h > world_h * margin
    detail = (
        f'map footprint {map_w:.1f}x{map_h:.1f}m vs world={world!r} '
        f'footprint {world_w:.1f}x{world_h:.1f}m'
    )
    return (not mismatch), detail


def run_preflight(node, log, args):
    log.log('preflight: waiting for required topics', topics=REQUIRED_TOPICS)
    deadline = time.time() + args.topic_timeout_s
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if all(node.last_seen[t] is not None for t in REQUIRED_TOPICS):
            break
    missing = [t for t in REQUIRED_TOPICS if node.last_seen[t] is None]
    if missing:
        log.log('preflight FAILED: topics silent', topics=missing)
        return False
    log.log('preflight: all required topics live')

    try:
        map_bounds = compute_map_bounds(args.map_yaml)
    except Exception as exc:
        log.log('preflight FAILED: could not read map.yaml', error=str(exc))
        return False

    ok, detail = check_map_world_match(map_bounds, args.world)
    if ok is False:
        log.log('preflight FAILED: map/world footprint mismatch', detail=detail)
        return False
    log.log(
        'preflight: map/world footprint OK' if ok else
        'preflight: map/world size check skipped', detail=detail)
    return True


# --- Shared wait / measurement helpers --------------------------------------

def wait_for_state(node, log, target_states, timeout_s,
                    stall_check=False, stall_timeout_s=60.0):
    """Spins until node.status is in target_states or times out. If
    stall_check, also fails fast with a named reason - not a bare timeout -
    once the probe reports EXHAUSTED while stuck in GLOBAL/CONVERGING for
    stall_timeout_s: this is the previously-missed 'robot stopped spinning'
    failure mode, now explicit instead of silent."""
    deadline = time.time() + timeout_s
    stalled_since = None
    last_logged = None
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.status != last_logged:
            last_logged = node.status
            log.log('state', state=node.status)
        if node.status in target_states:
            return True
        if stall_check and node.status in (GLOBAL, CONVERGING):
            probe_mode = (node.probe or {}).get('mode')
            if probe_mode == EXHAUSTED:
                if stalled_since is None:
                    stalled_since = time.time()
                elif time.time() - stalled_since >= stall_timeout_s:
                    log.log(
                        'STALLED: probe exhausted, cannot converge from '
                        'stationary',
                        attempts_used=(node.probe or {}).get('attempts_used'))
                    return False
            else:
                stalled_since = None
    log.log(
        'TIMEOUT waiting for target state',
        target=sorted(target_states), last=node.status)
    return False


def measure_delta_accuracy(node, log, model, translate_duration_s=3.0,
                            linear_mps=0.15):
    """AMCL accuracy via delta displacement: a fixed map-frame/world-frame
    offset (M3 bag start vs Gazebo world origin) cancels out in a CHANGE
    comparison but would look like a large, spurious error in an absolute
    amcl_pose-vs-ground-truth comparison."""
    node.spin_for(0.1)
    start_gt = gz_ground_truth(model)
    start_amcl = node.amcl_xy
    if start_gt is None or start_amcl is None:
        log.log('accuracy check SKIPPED: missing baseline ground truth/amcl pose')
        return None

    end_time = time.time() + translate_duration_s
    while time.time() < end_time:
        node.publish_cmd(linear_mps, 0.0)
        rclpy.spin_once(node, timeout_sec=0.1)
    node.publish_cmd(0.0, 0.0)
    node.spin_for(1.0)  # let AMCL settle after motion

    end_gt = gz_ground_truth(model)
    end_amcl = node.amcl_xy
    if end_gt is None or end_amcl is None:
        log.log('accuracy check SKIPPED: missing ending ground truth/amcl pose')
        return None

    gt_delta = math.hypot(end_gt[0] - start_gt[0], end_gt[1] - start_gt[1])
    amcl_delta = math.hypot(
        end_amcl[0] - start_amcl[0], end_amcl[1] - start_amcl[1])
    delta_error_m = abs(gt_delta - amcl_delta)
    result = {
        'gt_delta_m': round(gt_delta, 3),
        'amcl_delta_m': round(amcl_delta, 3),
        'delta_error_m': round(delta_error_m, 3),
    }
    log.log('accuracy check', **result)
    return result


# --- Scenarios ---------------------------------------------------------------

def scenario_cold_start(node, log, args):
    """Covers M5_PLAN.md verification items #1 (arbitrary start pose) and
    #2 (convergence to USABLE)."""
    report = {'preflight_ok': run_preflight(node, log, args)}
    if not report['preflight_ok']:
        report['pass'] = False
        return report

    t_start = time.time()
    converged = wait_for_state(
        node, log, {USABLE}, args.converge_timeout_s,
        stall_check=True, stall_timeout_s=args.stall_timeout_s)
    report['converged'] = converged
    report['time_to_converge_s'] = round(time.time() - t_start, 1)

    accuracy = measure_delta_accuracy(node, log, args.model) if converged else None
    report['accuracy'] = accuracy
    report['pass'] = bool(
        converged
        and (accuracy is None or accuracy['delta_error_m'] < args.accuracy_threshold_m)
    )
    return report


def pick_kidnap_target(node, log, args):
    """Choose a kidnap teleport target in Gazebo WORLD frame. Defaults to
    the active map's centroid - a point known to fall inside the explored,
    well-constrained region - transformed from map frame to world frame via
    the CURRENT live offset between amcl_pose and ground truth (the two
    frames differ by a fixed translation: the map frame's origin is
    wherever the M3 mapping bag started, not the Gazebo world origin, and
    that offset can be measured directly right before the teleport). A
    hardcoded guess risks landing outside the actually-explored area and
    localization never reconverging for a reason unrelated to the
    kidnap-recovery mechanism itself - exactly what an earlier hardcoded
    'mirror of spawn' default hit during M5 verification (see
    KIDNAP_DEFAULTS, now only a last-resort fallback)."""
    if args.kidnap_x is not None and args.kidnap_y is not None:
        return (
            args.kidnap_x, args.kidnap_y,
            args.kidnap_z if args.kidnap_z is not None else 0.05,
            args.kidnap_yaw if args.kidnap_yaw is not None else 0.0,
        )

    gt = gz_ground_truth(args.model)
    amcl = node.amcl_xy
    try:
        map_bounds = compute_map_bounds(args.map_yaml)
    except Exception:
        map_bounds = None

    if gt and amcl and map_bounds:
        offset_x = gt[0] - amcl[0]
        offset_y = gt[1] - amcl[1]
        map_center_x = map_bounds['origin_x'] + map_bounds['width_m'] / 2.0
        map_center_y = map_bounds['origin_y'] + map_bounds['height_m'] / 2.0
        target_x = map_center_x + offset_x
        target_y = map_center_y + offset_y
        log.log(
            'kidnap: target from map centroid + live frame offset',
            map_center=(round(map_center_x, 2), round(map_center_y, 2)),
            offset=(round(offset_x, 2), round(offset_y, 2)),
            target=(round(target_x, 2), round(target_y, 2)))
        return (
            target_x, target_y,
            args.kidnap_z if args.kidnap_z is not None else 0.05,
            args.kidnap_yaw if args.kidnap_yaw is not None else 0.0,
        )

    default_pose = KIDNAP_DEFAULTS.get(args.world, (0.0, 0.0, 0.05, 0.0))
    log.log(
        'kidnap: could not compute map-grounded target, falling back to '
        'static default', default=default_pose)
    return default_pose


def scenario_kidnap(node, log, args):
    """Covers M5_PLAN.md verification item #4 (kidnapped-robot recovery):
    reach USABLE, teleport the model via Gazebo Transport, and confirm the
    monitor detects the jump, issues a reinit, the probe re-arms, and the
    system re-converges without any operator action."""
    report = {'preflight_ok': run_preflight(node, log, args)}
    if not report['preflight_ok']:
        report['pass'] = False
        return report

    converged = wait_for_state(
        node, log, {USABLE}, args.converge_timeout_s,
        stall_check=True, stall_timeout_s=args.stall_timeout_s)
    report['initial_converged'] = converged
    if not converged:
        report['pass'] = False
        return report

    reinit_count_before = (node.metrics or {}).get('reinit_count', 0)
    attempts_before = (node.probe or {}).get('attempts_used', 0)
    kx, ky, kz, kyaw = pick_kidnap_target(node, log, args)
    log.log('kidnap: teleporting model', x=kx, y=ky, z=kz, yaw=kyaw)
    teleported = gz_teleport(args.world, args.model, kx, ky, kz, kyaw)
    report['teleport_ack'] = teleported
    if not teleported:
        log.log('kidnap FAILED: teleport service call not acknowledged')
        report['pass'] = False
        return report

    left_usable = wait_for_state(
        node, log, {DEGRADED, LOST, GLOBAL, CONVERGING},
        args.detect_timeout_s, stall_check=False)
    report['left_usable'] = left_usable

    recovered = wait_for_state(
        node, log, {USABLE}, args.recover_timeout_s,
        stall_check=True, stall_timeout_s=args.stall_timeout_s)
    report['recovered'] = recovered

    reinit_count_after = (node.metrics or {}).get('reinit_count', 0)
    report['reinit_count_before'] = reinit_count_before
    report['reinit_count_after'] = reinit_count_after
    report['reinit_issued'] = reinit_count_after > reinit_count_before
    report['probe_attempts_before'] = attempts_before
    report['probe_attempts_after_recovery'] = (node.probe or {}).get('attempts_used')

    report['post_recovery_accuracy'] = (
        measure_delta_accuracy(node, log, args.model) if recovered else None)

    report['pass'] = bool(left_usable and recovered and report['reinit_issued'])
    return report


def scenario_ekf_drift(node, log, args):
    """Covers M5_PLAN.md verification item #3 (EKF-drift claim). Scope
    note: this measures EKF-vs-AMCL divergence WHILE AMCL keeps correcting
    (validates the bounded-error claim with the full stack active) - it
    does not disable AMCL mid-run to measure raw odometry drift in
    isolation, which is out of scope for this harness."""
    report = {'preflight_ok': run_preflight(node, log, args)}
    if not report['preflight_ok']:
        report['pass'] = False
        return report

    converged = wait_for_state(
        node, log, {USABLE}, args.converge_timeout_s,
        stall_check=True, stall_timeout_s=args.stall_timeout_s)
    report['converged'] = converged
    if not converged:
        report['pass'] = False
        return report

    log.log(
        'ekf_drift: starting scripted motion + divergence sampling',
        duration_s=args.ekf_drift_duration_s)

    node.spin_for(0.2)
    start_gt = gz_ground_truth(args.model)
    start_odom = node.odom_xy
    start_amcl = node.amcl_xy
    if not (start_gt and start_odom and start_amcl):
        log.log('ekf_drift FAILED: missing baseline reading')
        report['pass'] = False
        return report

    samples = []
    t0 = time.time()
    t_end = t0 + args.ekf_drift_duration_s
    next_sample = t0
    while time.time() < t_end:
        phase = int((time.time() - t0) // 3.0) % 4
        linear = 0.15 if phase in (0, 2) else 0.0
        angular = 0.6 if phase in (1, 3) else 0.0
        node.publish_cmd(linear, angular)
        rclpy.spin_once(node, timeout_sec=0.2)
        if time.time() >= next_sample:
            next_sample = time.time() + 5.0
            gt = gz_ground_truth(args.model)
            if gt and node.odom_xy and node.amcl_xy:
                gt_delta = math.hypot(gt[0] - start_gt[0], gt[1] - start_gt[1])
                odom_delta = math.hypot(
                    node.odom_xy[0] - start_odom[0], node.odom_xy[1] - start_odom[1])
                amcl_delta = math.hypot(
                    node.amcl_xy[0] - start_amcl[0], node.amcl_xy[1] - start_amcl[1])
                sample = {
                    't': round(time.time() - t0, 1),
                    'odom_err_m': round(abs(odom_delta - gt_delta), 3),
                    'amcl_err_m': round(abs(amcl_delta - gt_delta), 3),
                }
                samples.append(sample)
                log.log('ekf_drift sample', **sample)
    node.publish_cmd(0.0, 0.0)

    report['samples'] = samples
    if len(samples) >= 2:
        odom_growth = samples[-1]['odom_err_m'] - samples[0]['odom_err_m']
        amcl_growth = samples[-1]['amcl_err_m'] - samples[0]['amcl_err_m']
        report['odom_err_growth_m'] = round(odom_growth, 3)
        report['amcl_err_growth_m'] = round(amcl_growth, 3)
        report['amcl_final_err_m'] = samples[-1]['amcl_err_m']
        report['pass'] = bool(
            samples[-1]['amcl_err_m'] < args.accuracy_threshold_m
            and amcl_growth <= odom_growth
        )
    else:
        report['pass'] = False
        log.log('ekf_drift FAILED: not enough samples collected')
    return report


SCENARIOS = {
    'cold_start': scenario_cold_start,
    'kidnap': scenario_kidnap,
    'ekf_drift': scenario_ekf_drift,
}


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scenario', required=True, choices=sorted(SCENARIOS))
    parser.add_argument('--world', default='house')
    parser.add_argument('--model', default='rambla')
    parser.add_argument(
        '--map-yaml', default=os.path.expanduser('~/ros2_ws/maps/active/map.yaml'))
    parser.add_argument('--report-path', default=None)
    parser.add_argument('--topic-timeout-s', type=float, default=15.0)
    parser.add_argument('--converge-timeout-s', type=float, default=180.0)
    parser.add_argument('--stall-timeout-s', type=float, default=60.0)
    parser.add_argument('--detect-timeout-s', type=float, default=20.0)
    parser.add_argument('--recover-timeout-s', type=float, default=90.0)
    parser.add_argument('--ekf-drift-duration-s', type=float, default=60.0)
    parser.add_argument('--accuracy-threshold-m', type=float, default=0.5)
    parser.add_argument('--kidnap-x', type=float, default=None)
    parser.add_argument('--kidnap-y', type=float, default=None)
    parser.add_argument('--kidnap-z', type=float, default=None)
    parser.add_argument('--kidnap-yaw', type=float, default=None)
    return parser.parse_args()


def main():
    args = parse_args()
    # Kidnap target resolution (map centroid + live frame offset, or an
    # explicit --kidnap-x/-y override) happens in pick_kidnap_target() at
    # scenario run time, once amcl_pose/ground truth are actually available.
    if args.report_path is None:
        ts = time.strftime('%Y%m%d_%H%M%S')
        args.report_path = f'verification_report_{args.scenario}_{ts}.json'

    rclpy.init()
    node = Watcher()
    log = EventLog()
    log.log('verify_localization starting', scenario=args.scenario, world=args.world)

    full_report = {
        'scenario': args.scenario,
        'world': args.world,
        'model': args.model,
        'started_at': time.strftime('%Y-%m-%dT%H:%M:%S'),
    }
    try:
        result = SCENARIOS[args.scenario](node, log, args)
    except Exception as exc:
        log.log('scenario raised an exception', error=str(exc))
        result = {'pass': False, 'error': str(exc)}
    finally:
        node.destroy_node()
        rclpy.shutdown()

    full_report.update(result)
    full_report['events'] = log.events
    with open(args.report_path, 'w') as f:
        json.dump(full_report, f, indent=2)

    verdict = 'PASS' if full_report.get('pass') else 'FAIL'
    print()
    print(f'==== {args.scenario} : {verdict} ====')
    for key, value in result.items():
        print(f'  {key}: {value}')
    print(f'  report: {args.report_path}')

    sys.exit(0 if full_report.get('pass') else 1)


if __name__ == '__main__':
    main()
