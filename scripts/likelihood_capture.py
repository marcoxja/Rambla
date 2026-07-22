"""M7 Phase 2.5 Task 4 diagnostic (part 1/2): capture (scan, AMCL pose,
ground truth) snapshots from the live sim for offline likelihood scoring.

Not a permanent verification scenario (unlike verify_localization.py) --
a one-off diagnostic tool, piped over SSH stdin like scripts/validate_map.py
and scripts/sample_resources.py (no colcon build/install/deploy step needed).

Assumes localize:=true is already running against map_v002 (rambla_sim_start
already up) -- does not start or restart anything.

Trigger: rather than waiting for /localization_status == USABLE (which the
whole point of this diagnostic is that it usually never reaches), this
snapshots the moment localization_probe's mode reports EXHAUSTED while
status is GLOBAL or CONVERGING -- exactly the "confident/still" moment the
M7_LOCALIZATION_FINDINGS.md six-snapshot trial sampled by hand. Since M7
Phase 2.5 Task 1's stationary_stale_timeout_s=60s now holds a CONVERGING
filter still for ~60s instead of destructively reiniting it in ~11s, there
is a wide, safe window to capture in after the trigger fires.

Captures up to --count snapshots (default 4) within --max-wait-s (default
600s), across as many natural GLOBAL/CONVERGING/reinit cycles as it takes --
no teleporting, no forced restarts, just passive observation of the sim's
existing behavior.

Usage (from the Mac, over SSH -- see rambla_likelihood_capture in
scripts/vm.sh):
    ssh rambla-vm "source /opt/ros/jazzy/setup.bash && \
        source ~/ros2_ws/install/setup.bash && python3 -" \
        < scripts/likelihood_capture.py --count 4 --max-wait-s 600 \
        > snapshots.json
"""
import argparse
import json
import math
import re
import subprocess
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

EXHAUSTED = 'EXHAUSTED'
PRE_USABLE_STATES = {'GLOBAL', 'CONVERGING'}


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


class SnapshotWatcher(Node):
    def __init__(self, model):
        super().__init__('likelihood_capture')
        self.model = model
        self.status = None
        self.probe_mode = None
        self.scan = None
        self.amcl_pose = None
        # dedupe: one snapshot per continuous run of the trigger condition,
        # not per distinct (status, mode) value -- status/mode cycle through
        # the same small set of values every ~31s, so comparing against the
        # last-seen VALUE (rather than tracking edge-armed state) would fire
        # once and then never again for the life of the process.
        self._armed = True
        self._last_capture_t = None

        self.create_subscription(String, '/localization_status', self._on_status, 10)
        self.create_subscription(String, '/probe_status', self._on_probe, 10)
        self.create_subscription(
            LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, 10)

    def _on_status(self, msg):
        self.status = msg.data

    def _on_probe(self, msg):
        try:
            self.probe_mode = json.loads(msg.data).get('mode')
        except json.JSONDecodeError:
            pass

    def _on_scan(self, msg):
        self.scan = msg

    def _on_amcl(self, msg):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = 2 * math.atan2(o.z, o.w)
        self.amcl_pose = (p.x, p.y, yaw)

    def ready_to_trigger(self):
        holding = self.status in PRE_USABLE_STATES and self.probe_mode == EXHAUSTED
        if not holding:
            self._armed = True
            return False
        if self._armed:
            # Debounce: /probe_status and /localization_status can flap
            # (EXHAUSTED -> briefly something else -> EXHAUSTED again)
            # within the same real hold, which would otherwise re-arm and
            # fire a near-instant duplicate capture of the same event.
            now = time.time()
            if self._last_capture_t is not None and now - self._last_capture_t < 5.0:
                return False
            self._armed = False
            self._last_capture_t = now
            return True
        return False

    def snapshot(self):
        gt = gz_ground_truth(self.model)
        scan = self.scan
        return {
            't_wall': time.time(),
            'localization_status': self.status,
            'amcl_pose': self.amcl_pose,
            'ground_truth_world': gt,
            'scan': {
                'angle_min': scan.angle_min,
                'angle_max': scan.angle_max,
                'angle_increment': scan.angle_increment,
                'range_min': scan.range_min,
                'range_max': scan.range_max,
                'ranges': list(scan.ranges),
            } if scan is not None else None,
        }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', default='rambla')
    parser.add_argument('--count', type=int, default=4)
    parser.add_argument('--max-wait-s', type=float, default=600.0)
    args = parser.parse_args()

    rclpy.init()
    node = SnapshotWatcher(args.model)
    snapshots = []
    deadline = time.time() + args.max_wait_s
    print(f'waiting for up to {args.count} EXHAUSTED-while-pre-USABLE '
          f'triggers (budget {args.max_wait_s:.0f}s)...', file=sys.stderr)
    try:
        while time.time() < deadline and len(snapshots) < args.count:
            rclpy.spin_once(node, timeout_sec=0.2)
            if node.ready_to_trigger() and node.scan is not None and node.amcl_pose is not None:
                snap = node.snapshot()
                snapshots.append(snap)
                print(
                    f'captured snapshot {len(snapshots)}/{args.count}: '
                    f'status={snap["localization_status"]} '
                    f'amcl_pose={snap["amcl_pose"]} '
                    f'ground_truth={snap["ground_truth_world"]}',
                    file=sys.stderr)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if len(snapshots) < args.count:
        print(
            f'WARNING: only captured {len(snapshots)}/{args.count} before '
            f'the time budget ran out', file=sys.stderr)

    json.dump({'model': args.model, 'snapshots': snapshots}, sys.stdout, indent=2)


if __name__ == '__main__':
    main()
