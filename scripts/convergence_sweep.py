"""M7 Phase 2.5 Task 5: formal, reusable convergence-cycle monitor for
isolated AMCL parameter sweeps.

Task 3's mistake was bundling three amcl.yaml changes into one live trial
and losing attribution for which one did what. Task 5 explicitly requires
testing each dimension alone and head-to-head -- this script is the shared
measurement tool for that, so every candidate config (baseline,
recovery_alpha re-enabled, sigma_hit/z_hit sharpened, ...) is measured the
same way and results are directly comparable, instead of hand-reading logs
fresh each time (as Task 0's throwaway monitor and the post-Task-3 "formal
head-to-head" both did ad hoc).

Not a pass/fail scenario like verify_localization.py -- this observes the
sim passively for a fixed wall-clock budget (no restart, no teleport) and
reports how many independent GLOBAL-reinit cycles occurred and how many of
them reached CONVERGING / USABLE. Each natural reinit cycle is an
independent trial of "can this config disambiguate the multi-room map from
a fresh uniform spread", so one long run yields many trials cheaply (the
post-Task-3 test got 8-10 cycles out of a single 300s run) -- no need to
restart the sim between trials.

Usage (from the Mac, over SSH -- see rambla_convergence_sweep in
scripts/vm.sh):
    ssh rambla-vm "source /opt/ros/jazzy/setup.bash && \
        source ~/ros2_ws/install/setup.bash && python3 -" \
        < scripts/convergence_sweep.py --duration-s 900 --label baseline \
        > sweep_baseline.json
"""
import argparse
import json
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from std_msgs.msg import String

GLOBAL = 'GLOBAL'
CONVERGING = 'CONVERGING'
USABLE = 'USABLE'


class SweepWatcher(Node):
    def __init__(self):
        super().__init__('convergence_sweep')
        self.status = None
        self.reinit_count = None
        self.events = []  # (t, status)
        self._t0 = time.time()

        self.create_subscription(String, '/localization_status', self._on_status, 10)
        self.create_subscription(String, '/localization_metrics', self._on_metrics, 10)
        # Subscribed only to confirm the topic is live -- not used for
        # cycle accounting (that's status-transition-driven).
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', lambda _msg: None, 10)

    def _on_status(self, msg):
        if msg.data != self.status:
            self.status = msg.data
            self.events.append((round(time.time() - self._t0, 2), msg.data))

    def _on_metrics(self, msg):
        try:
            self.reinit_count = json.loads(msg.data).get('reinit_count')
        except json.JSONDecodeError:
            pass


def derive_cycles(events):
    """A cycle starts at each transition INTO GLOBAL (a fresh uniform
    spread after reinit) and ends at the next such transition (or the end
    of the run). Report per-cycle whether it ever reached CONVERGING/
    USABLE and how long it lasted."""
    cycles = []
    current = None
    for t, status in events:
        if status == GLOBAL:
            if current is not None:
                current['end_t'] = t
                current['duration_s'] = round(t - current['start_t'], 1)
                cycles.append(current)
            current = {
                'start_t': t, 'reached_converging': False, 'reached_usable': False,
            }
        elif current is not None:
            if status == CONVERGING:
                current['reached_converging'] = True
            elif status == USABLE:
                current['reached_converging'] = True  # USABLE implies it passed through/superseded CONVERGING
                current['reached_usable'] = True
    if current is not None:
        current.setdefault('end_t', None)
        current.setdefault('duration_s', None)
        cycles.append(current)
    return cycles


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--duration-s', type=float, default=900.0)
    parser.add_argument('--label', default='unlabeled', help='config label, e.g. "recovery_alpha_on" -- purely for the report, not functional')
    args = parser.parse_args()

    rclpy.init()
    node = SweepWatcher()
    deadline = time.time() + args.duration_s
    print(f'sweep "{args.label}": watching for {args.duration_s:.0f}s...', file=sys.stderr)
    try:
        while time.time() < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    cycles = derive_cycles(node.events)
    # Drop the final cycle if the run ended mid-cycle (no clean end_t) --
    # counting a truncated cycle as a normal trial would bias the sample.
    complete_cycles = [c for c in cycles if c['end_t'] is not None]
    converging_count = sum(1 for c in complete_cycles if c['reached_converging'])
    usable_count = sum(1 for c in complete_cycles if c['reached_usable'])

    report = {
        'label': args.label,
        'duration_s': args.duration_s,
        'final_reinit_count': node.reinit_count,
        'cycle_count': len(complete_cycles),
        'converging_count': converging_count,
        'usable_count': usable_count,
        'converging_rate': round(converging_count / len(complete_cycles), 3) if complete_cycles else None,
        'usable_rate': round(usable_count / len(complete_cycles), 3) if complete_cycles else None,
        'cycles': complete_cycles,
        'raw_events': node.events,
    }
    print(
        f'==== sweep "{args.label}" ==== '
        f'{len(complete_cycles)} cycles, '
        f'{converging_count} reached CONVERGING, {usable_count} reached USABLE',
        file=sys.stderr)
    json.dump(report, sys.stdout, indent=2)


if __name__ == '__main__':
    main()
