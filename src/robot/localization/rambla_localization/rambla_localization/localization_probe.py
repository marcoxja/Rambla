"""rclpy wrapper around ProbeBehavior: a small, bounded "nudge" behavior for
AMCL's cold-start ambiguity - a single stationary scan often can't
disambiguate multiple pose hypotheses in a symmetric room; AMCL needs
motion to accumulate distinguishing bearings between landmarks. Active only
while /localization_status reads GLOBAL/CONVERGING and the robot has been
genuinely stationary (real /odometry/filtered velocity, not just no
teleop - /control_authority flipping back to AUTO only means no teleop
command for 0.4s, the robot may still be physically coasting) for a short
grace period. Publishes a slow, bounded in-place rotation to /cmd_vel_raw -
the same autonomous-intent channel rambla_traversal already uses, with no
changes to rambla_safety's arbitration (INT-005). Gives up after a bounded
number of rotation attempts rather than searching indefinitely; a fresh
localization_monitor reinit (edge into GLOBAL) re-arms it.

All state-machine logic (grace period, rotation/observe timing, bounded
attempts) lives in the new, rclpy-independent probe_behavior.ProbeBehavior -
same shape as this package's own localization_state.LocalizationMonitor and
rambla_safety's ControlAuthority. This file only wires ROS
subscriptions/timer to it and publishes its output.

Subscribes to /control_authority (published by rambla_safety's SafetyNode)
and skips publishing entirely while it reads MANUAL - belt-and-suspenders
on top of rambla_safety's own discard-while-MANUAL behavior, exactly like
rambla_traversal's traversal_node.py. Not calling tick() at all while
MANUAL also freezes probe_behavior's internal phase timers, matching
traversal_node's "don't touch behavior state while paused" pattern.
"""
import json

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import String

from rambla_localization.probe_behavior import (
    DEFAULT_ANGULAR_VEL_RAD_S,
    DEFAULT_ATTEMPT_DURATION_S,
    DEFAULT_GRACE_PERIOD_S,
    DEFAULT_LINEAR_VEL_MPS,
    DEFAULT_MAX_ATTEMPTS,
    DEFAULT_OBSERVE_DURATION_S,
    DEFAULT_STATIONARY_ANGULAR_THRESHOLD_RADPS,
    DEFAULT_STATIONARY_LINEAR_THRESHOLD_MPS,
    DEFAULT_TRANSLATE_DURATION_S,
    EXHAUSTED,
    ProbeBehavior,
)

CMD_VEL_RAW_TOPIC = '/cmd_vel_raw'
CONTROL_AUTHORITY_TOPIC = '/control_authority'
LOCALIZATION_STATUS_TOPIC = '/localization_status'
PROBE_STATUS_TOPIC = '/probe_status'
ODOM_TOPIC = '/odometry/filtered'
MANUAL_MODE = 'MANUAL'

PROBE_RATE_HZ = 10.0


class LocalizationProbeNode(Node):

    def __init__(self):
        super().__init__('localization_probe')

        self.declare_parameter('grace_period_s', DEFAULT_GRACE_PERIOD_S)
        self.declare_parameter('angular_vel_rad_s', DEFAULT_ANGULAR_VEL_RAD_S)
        self.declare_parameter('attempt_duration_s', DEFAULT_ATTEMPT_DURATION_S)
        self.declare_parameter('observe_duration_s', DEFAULT_OBSERVE_DURATION_S)
        self.declare_parameter('max_attempts', DEFAULT_MAX_ATTEMPTS)
        # M7 Phase 1: additive, default-preserving - a localize-only run
        # (no nav bringup) is unaffected. Under the M7 nav bringup this is
        # remapped to /cmd_vel_probe so behavior_supervisor, not this node,
        # owns /cmd_vel_raw (see navigate.launch.py / apartment_world.
        # launch.py's probe_cmd_topic plumbing).
        self.declare_parameter('cmd_vel_topic', CMD_VEL_RAW_TOPIC)
        self.declare_parameter(
            'stationary_linear_threshold_mps',
            DEFAULT_STATIONARY_LINEAR_THRESHOLD_MPS)
        self.declare_parameter(
            'stationary_angular_threshold_radps',
            DEFAULT_STATIONARY_ANGULAR_THRESHOLD_RADPS)
        self.declare_parameter('linear_vel_mps', DEFAULT_LINEAR_VEL_MPS)
        self.declare_parameter(
            'translate_duration_s', DEFAULT_TRANSLATE_DURATION_S)

        self._max_attempts = self.get_parameter('max_attempts').value
        self._behavior = ProbeBehavior(
            clock=lambda: self.get_clock().now().nanoseconds / 1e9,
            grace_period_s=self.get_parameter('grace_period_s').value,
            angular_vel_rad_s=self.get_parameter('angular_vel_rad_s').value,
            attempt_duration_s=self.get_parameter('attempt_duration_s').value,
            observe_duration_s=self.get_parameter('observe_duration_s').value,
            max_attempts=self._max_attempts,
            stationary_linear_threshold_mps=self.get_parameter(
                'stationary_linear_threshold_mps').value,
            stationary_angular_threshold_radps=self.get_parameter(
                'stationary_angular_threshold_radps').value,
            linear_vel_mps=self.get_parameter('linear_vel_mps').value,
            translate_duration_s=self.get_parameter(
                'translate_duration_s').value,
        )

        self._authority_mode = 'AUTO'
        self._last_logged_mode = None

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self._probe_status_pub = self.create_publisher(
            String, PROBE_STATUS_TOPIC, 10)
        self.create_subscription(
            String, LOCALIZATION_STATUS_TOPIC, self._on_status, 10)
        self.create_subscription(
            String, CONTROL_AUTHORITY_TOPIC, self._on_control_authority, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)
        self.create_timer(1.0 / PROBE_RATE_HZ, self._tick)

    def _on_status(self, msg):
        self._behavior.on_localization_status(msg.data)

    def _on_control_authority(self, msg):
        self._authority_mode = msg.data

    def _on_odom(self, msg):
        twist = msg.twist.twist
        self._behavior.on_velocity(twist.linear.x, twist.angular.z)

    def _tick(self):
        if self._authority_mode == MANUAL_MODE:
            return
        linear, angular = self._behavior.tick()
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)

        mode = self._behavior.mode
        if mode != self._last_logged_mode:
            self._last_logged_mode = mode
            if mode == EXHAUSTED:
                self.get_logger().warn(
                    f'probe exhausted after {self._behavior.attempts_used} '
                    f'attempts - holding; will re-arm on next GLOBAL edge')
            else:
                self.get_logger().info(f'probe mode -> {mode}')

        self._probe_status_pub.publish(String(data=json.dumps({
            'mode': mode,
            'attempts_used': self._behavior.attempts_used,
            'max_attempts': self._max_attempts,
        })))


def main():
    rclpy.init()
    node = LocalizationProbeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
