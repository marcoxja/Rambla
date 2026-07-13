"""rclpy wrapper around SafetyMonitor + ControlAuthority: subscribes to
/scan and both bumper contact topics, and to both motion-intent sources -
/cmd_vel_raw (autonomous, e.g. rambla_traversal) and /cmd_vel_teleop
(manual, rambla_control_panel) - then republishes the arbitrated,
safety-filtered result to /cmd_vel at a steady rate.

This node is the sole final publisher to /cmd_vel, for both AUTO and
MANUAL. Resolves P1-3 (see .claude/internal-docs/audits/2026-07-08-
accepted-stabilization-findings.md Sec.5): previously rambla_control_panel
published straight to /cmd_vel, bypassing this node's safety filter
entirely, with nothing preventing it and the autonomous path from both
writing to /cmd_vel at once. Now exactly one source is selected per tick
(see control_authority.py for the AUTO/MANUAL arbitration rules) and that
source's command always passes through SafetyMonitor before publish - so
local safety remains authoritative regardless of who's driving.

/control_authority (std_msgs/String, "AUTO"/"MANUAL") is published
alongside /cmd_vel so a top-level autonomous-behavior node (e.g.
rambla_traversal) can pause/yield while MANUAL is engaged, without needing
its own mode-arbitration logic - see control_authority.py's module
docstring.

Republishing on a timer (not directly from callbacks) decouples the
publish rate from sensor/command jitter - same pattern as
rambla_control_panel's ControlPanelNode._publish_cmd_vel.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

from rambla_safety.control_authority import ControlAuthority
from rambla_safety.safety_monitor import (
    DEFAULT_FRONT_ARC_DEG,
    DEFAULT_MIN_VALID_RANGE_M,
    DEFAULT_STOP_DISTANCE_M,
    SafetyMonitor,
)

CMD_VEL_RATE_HZ = 20.0

SCAN_TOPIC = '/scan'
CMD_VEL_RAW_TOPIC = '/cmd_vel_raw'
CMD_VEL_TELEOP_TOPIC = '/cmd_vel_teleop'
CMD_VEL_TOPIC = '/cmd_vel'
CONTROL_AUTHORITY_TOPIC = '/control_authority'


class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter('stop_distance_m', DEFAULT_STOP_DISTANCE_M)
        self.declare_parameter('front_arc_deg', DEFAULT_FRONT_ARC_DEG)
        self.declare_parameter('min_valid_range_m', DEFAULT_MIN_VALID_RANGE_M)
        self._monitor = SafetyMonitor(
            stop_distance_m=self.get_parameter('stop_distance_m').value,
            front_arc_deg=self.get_parameter('front_arc_deg').value,
            min_valid_range_m=self.get_parameter('min_valid_range_m').value,
            clock=lambda: self.get_clock().now().nanoseconds / 1e9)
        self._authority = ControlAuthority(
            clock=lambda: self.get_clock().now().nanoseconds / 1e9)

        self._cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self._authority_pub = self.create_publisher(
            String, CONTROL_AUTHORITY_TOPIC, 10)
        self.create_timer(1.0 / CMD_VEL_RATE_HZ, self._publish_cmd_vel)

        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, 10)
        self.create_subscription(
            Contacts, '/bumper_left/contact', self._on_bumper_left, 10)
        self.create_subscription(
            Contacts, '/bumper_right/contact', self._on_bumper_right, 10)
        self.create_subscription(
            Twist, CMD_VEL_RAW_TOPIC, self._on_cmd_vel_raw, 10)
        self.create_subscription(
            Twist, CMD_VEL_TELEOP_TOPIC, self._on_cmd_vel_teleop, 10)

    def _on_scan(self, msg):
        self._monitor.on_scan(msg.ranges, msg.angle_min, msg.angle_increment)

    def _on_bumper_left(self, msg):
        self._monitor.on_bumper_left(bool(msg.contacts))

    def _on_bumper_right(self, msg):
        self._monitor.on_bumper_right(bool(msg.contacts))

    def _on_cmd_vel_raw(self, msg):
        self._authority.on_auto_cmd(msg.linear.x, msg.angular.z)

    def _on_cmd_vel_teleop(self, msg):
        self._authority.on_manual_cmd(msg.linear.x, msg.angular.z)

    def _publish_cmd_vel(self):
        mode, (linear, angular) = self._authority.tick()
        linear, angular = self._monitor.filter_cmd(linear, angular)
        out = Twist()
        out.linear.x = linear
        out.angular.z = angular
        self._cmd_vel_pub.publish(out)
        self._authority_pub.publish(String(data=mode))


def main():
    rclpy.init()
    node = SafetyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
