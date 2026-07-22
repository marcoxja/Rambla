"""M7 navigation verification harness: deterministic, scripted scenarios that
emit a small event log + a verification_report.json verdict, the same
"no live human/AI reading of raw telemetry" contract verify_localization.py
established for M5 (see its module docstring). Reuses that harness's helpers
(EventLog, gz_teleport, compute_map_bounds, KIDNAP_DEFAULTS) directly rather
than reimplementing them - M7_PLAN.md's own instruction ("reused by M7's
harness, not reinvented").

Usage (on rambla-vm, sim launched with localize:=true navigate:=true):
    ros2 run rambla_navigation verify_navigation --scenario nav_bringup --world house
    ros2 run rambla_navigation verify_navigation --scenario gating --world house

Only nav_bringup (Phase 1) and gating (Phase 2) are implemented here -
reach_goal/obstacle_avoid (Phase 3) and unreachable_goal/recovery (Phase 4)
land with their own phases, per M7_PLAN.md's phased verification split.
"""
import argparse
import json
import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from rclpy.node import Node
from std_msgs.msg import String

from rambla_localization.localization_state import USABLE
from rambla_localization.verify_localization import (
    EventLog,
    KIDNAP_DEFAULTS,
    compute_map_bounds,
    gz_teleport,
)

REQUIRED_TOPICS = ['/localization_status', '/behavior_mode', '/cmd_vel_raw']
# Existence-in-the-graph checks, not subscribe-and-wait-for-a-message: the
# global costmap is mostly static (always_send_full_costmap: false) and, once
# stable, may not republish again for the rest of the run, so a late
# subscriber can't reliably wait for a fresh sample here even with
# TRANSIENT_LOCAL durability (confirmed live, 2026-07-16 - the stack was
# separately proven working end-to-end via an actual nav run that planned
# around walls and reached its goal moments earlier). Registration with the
# right type is sufficient evidence the node wired up correctly at bringup;
# actual costmap-driven behavior is what Phase 3's reach_goal/obstacle_avoid
# scenarios prove, not this one.
REGISTERED_TOPICS = [
    '/cmd_vel_nav', '/global_costmap/costmap_raw', '/local_costmap/costmap_raw']
NAV2_LIFECYCLE_NODE_NAMES = [
    'controller_server', 'planner_server', 'behavior_server', 'bt_navigator']

CMD_VEL_ZERO_TOLERANCE = 0.01


class Watcher(Node):
    """Subscribes to every signal the harness needs, tracks last-seen time
    per required/costmap topic (liveness), and exposes the latest decoded
    value of each - same shape as verify_localization.py's Watcher."""

    def __init__(self):
        super().__init__('verify_navigation')
        self.last_seen = {t: None for t in REQUIRED_TOPICS}
        self.status = None
        self.mode = None
        self.cmd_vel_raw = (0.0, 0.0)

        self.create_subscription(
            String, '/localization_status', self._on_status, 10)
        self.create_subscription(String, '/behavior_mode', self._on_mode, 10)
        self.create_subscription(
            Twist, '/cmd_vel_raw', self._on_cmd_vel_raw, 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

    def _seen(self, topic):
        self.last_seen[topic] = time.time()

    def _on_status(self, msg):
        self._seen('/localization_status')
        self.status = msg.data

    def _on_mode(self, msg):
        self._seen('/behavior_mode')
        self.mode = msg.data

    def _on_cmd_vel_raw(self, msg):
        self._seen('/cmd_vel_raw')
        self.cmd_vel_raw = (msg.linear.x, msg.angular.z)

    def registered_topic_names(self):
        return {name for name, _types in self.get_topic_names_and_types()}

    def publish_goal(self, x, y, yaw=0.0):
        msg = PoseStamped()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self._goal_pub.publish(msg)

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(
                self, timeout_sec=max(min(0.2, end - time.time()), 0.0))


def get_lifecycle_state_label(node, node_name, timeout_s):
    """Queries <node_name>/get_state directly - a real lifecycle query, not
    a guess from stdout - matching the nav_bringup scenario's own
    requirement ('query lifecycle state, not stdout')."""
    client = node.create_client(GetState, f'/{node_name}/get_state')
    if not client.wait_for_service(timeout_sec=timeout_s):
        return None
    future = client.call_async(GetState.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    result = future.result()
    if result is None:
        return None
    return result.current_state.id == LifecycleState.PRIMARY_STATE_ACTIVE


# --- Scenarios ---------------------------------------------------------------

def scenario_nav_bringup(node, log, args):
    """Covers Phase 1's deferred verification: every Nav2 lifecycle node
    reaches ACTIVE, /behavior_mode is actually publishing, and /cmd_vel_nav
    + both costmaps are registered in the graph with the right type (see
    REGISTERED_TOPICS' comment for why this is existence, not a received
    message, for the costmap topics)."""
    report = {}
    lifecycle_active = {}
    for name in NAV2_LIFECYCLE_NODE_NAMES:
        active = get_lifecycle_state_label(node, name, args.topic_timeout_s)
        lifecycle_active[name] = active
        log.log('lifecycle state', node=name, active=active)
    report['lifecycle_active'] = lifecycle_active
    all_active = all(lifecycle_active.values())

    log.log('nav_bringup: waiting for /behavior_mode')
    deadline = time.time() + args.topic_timeout_s
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.last_seen['/behavior_mode'] is not None:
            break
    behavior_mode_live = node.last_seen['/behavior_mode'] is not None
    report['behavior_mode_live'] = behavior_mode_live
    if not behavior_mode_live:
        log.log('nav_bringup: /behavior_mode still silent')

    registered = node.registered_topic_names()
    missing_registered = [t for t in REGISTERED_TOPICS if t not in registered]
    report['missing_registered_topics'] = missing_registered
    if missing_registered:
        log.log('nav_bringup: topics not registered', topics=missing_registered)

    report['pass'] = bool(
        all_active and behavior_mode_live and not missing_registered)
    return report


def scenario_gating(node, log, args):
    """Covers Phase 2's own verification: (a) a goal published before USABLE
    is held (never dispatched, /behavior_mode never reaches NAV) until
    USABLE; (b) once NAV is active and driving, a forced kidnap
    (DEGRADED/LOST) makes the supervisor cancel the goal, leave NAV, and
    hold /cmd_vel_raw at zero."""
    report = {}

    try:
        map_bounds = compute_map_bounds(args.map_yaml)
        goal_x = map_bounds['origin_x'] + map_bounds['width_m'] / 2.0
        goal_y = map_bounds['origin_y'] + map_bounds['height_m'] / 2.0
    except Exception as exc:
        log.log('gating FAILED: could not read map.yaml', error=str(exc))
        report['pass'] = False
        return report

    log.log('gating: waiting for required topics', topics=REQUIRED_TOPICS)
    deadline = time.time() + args.topic_timeout_s
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if all(node.last_seen[t] is not None for t in REQUIRED_TOPICS):
            break
    missing = [t for t in REQUIRED_TOPICS if node.last_seen[t] is None]
    if missing:
        log.log('gating FAILED: topics silent', topics=missing)
        report['pass'] = False
        return report

    node.spin_for(0.5)
    log.log('gating: publishing goal pre-USABLE',
            x=round(goal_x, 2), y=round(goal_y, 2), status=node.status)
    node.publish_goal(goal_x, goal_y)

    # Part (a): the goal must be held - /behavior_mode must not reach NAV
    # before /localization_status reaches USABLE.
    held = True
    dispatched = False
    conv_deadline = time.time() + args.converge_timeout_s
    while time.time() < conv_deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.mode == 'NAV':
            dispatched = True
            if node.status != USABLE:
                held = False
            break
    report['goal_held_pre_usable'] = held
    report['nav_dispatched'] = dispatched
    if not held:
        log.log(
            'gating FAILED: NAV entered before USABLE',
            status=node.status, mode=node.mode)
        report['pass'] = False
        return report
    if not dispatched:
        log.log(
            'gating FAILED: never reached NAV within converge_timeout_s',
            last_status=node.status)
        report['pass'] = False
        return report
    log.log('gating: NAV dispatched after USABLE, goal correctly held pre-USABLE')

    # Give the robot a moment to actually start driving before disrupting it.
    node.spin_for(2.0)

    kx, ky, kz, kyaw = KIDNAP_DEFAULTS.get(args.world, (0.0, 0.0, 0.05, 0.0))
    log.log('gating: teleporting to force DEGRADED/LOST mid-goal',
            x=kx, y=ky)
    teleported = gz_teleport(args.world, args.model, kx, ky, kz, kyaw)
    report['teleport_ack'] = teleported

    # Part (b): the supervisor must leave NAV and hold cmd_vel_raw at zero.
    left_nav = False
    detect_deadline = time.time() + args.detect_timeout_s
    while time.time() < detect_deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.mode != 'NAV':
            left_nav = True
            break
    report['left_nav_on_degrade'] = left_nav

    held_zero = False
    if left_nav:
        node.spin_for(0.5)  # let a couple more /cmd_vel_raw ticks land
        held_zero = (
            abs(node.cmd_vel_raw[0]) < CMD_VEL_ZERO_TOLERANCE
            and abs(node.cmd_vel_raw[1]) < CMD_VEL_ZERO_TOLERANCE
        )
    report['held_zero_after_cancel'] = held_zero
    log.log(
        'gating: post-kidnap check', left_nav=left_nav, held_zero=held_zero,
        mode=node.mode, cmd_vel_raw=node.cmd_vel_raw)

    report['pass'] = bool(held and dispatched and left_nav and held_zero)
    return report


SCENARIOS = {
    'nav_bringup': scenario_nav_bringup,
    'gating': scenario_gating,
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
    parser.add_argument('--converge-timeout-s', type=float, default=280.0)
    parser.add_argument('--detect-timeout-s', type=float, default=20.0)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.report_path is None:
        ts = time.strftime('%Y%m%d_%H%M%S')
        args.report_path = f'verification_report_{args.scenario}_{ts}.json'

    rclpy.init()
    node = Watcher()
    log = EventLog()
    log.log('verify_navigation starting', scenario=args.scenario, world=args.world)

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
