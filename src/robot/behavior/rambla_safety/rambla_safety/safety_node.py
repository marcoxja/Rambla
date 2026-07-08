"""rclpy wrapper around SafetyMonitor: subscribes to /scan_fixed and both
bumper contact topics, subscribes to /cmd_vel_raw (whatever an upstream
autonomous behavior wants to do), and republishes the safety-filtered
result to /cmd_vel at a steady rate - this node is the sole final
publisher to /cmd_vel for autonomous behaviors. Manual teleop
(rambla_control_panel) publishes straight to /cmd_vel and bypasses this
node entirely; the two are not intended to run at the same time.

Republishing on a timer (not directly from callbacks) decouples the
publish rate from sensor/command jitter - same pattern as
rambla_control_panel's ControlPanelNode._publish_cmd_vel.
"""
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from ros_gz_interfaces.msg import Contacts
from sensor_msgs.msg import LaserScan

from rambla_safety.safety_monitor import (
    DEFAULT_FRONT_ARC_DEG,
    DEFAULT_MIN_VALID_RANGE_M,
    DEFAULT_STOP_DISTANCE_M,
    SafetyMonitor,
)

CMD_VEL_RATE_HZ = 20.0

SCAN_TOPIC = '/scan_fixed'
CMD_VEL_RAW_TOPIC = '/cmd_vel_raw'
CMD_VEL_TOPIC = '/cmd_vel'


class SafetyNode(Node):

    def __init__(self):
        super().__init__('safety_node')

        self.declare_parameter('stop_distance_m', DEFAULT_STOP_DISTANCE_M)
        self.declare_parameter('front_arc_deg', DEFAULT_FRONT_ARC_DEG)
        self.declare_parameter('min_valid_range_m', DEFAULT_MIN_VALID_RANGE_M)
        self._monitor = SafetyMonitor(
            stop_distance_m=self.get_parameter('stop_distance_m').value,
            front_arc_deg=self.get_parameter('front_arc_deg').value,
            min_valid_range_m=self.get_parameter('min_valid_range_m').value)
        self._latest_cmd = (0.0, 0.0)

        self._cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_timer(1.0 / CMD_VEL_RATE_HZ, self._publish_cmd_vel)

        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, 10)
        self.create_subscription(
            Contacts, '/bumper_left/contact', self._on_bumper_left, 10)
        self.create_subscription(
            Contacts, '/bumper_right/contact', self._on_bumper_right, 10)
        self.create_subscription(
            Twist, CMD_VEL_RAW_TOPIC, self._on_cmd_vel_raw, 10)

    def _on_scan(self, msg):
        self._monitor.on_scan(msg.ranges, msg.angle_min, msg.angle_increment)

    def _on_bumper_left(self, msg):
        self._monitor.on_bumper_left(bool(msg.contacts))

    def _on_bumper_right(self, msg):
        self._monitor.on_bumper_right(bool(msg.contacts))

    def _on_cmd_vel_raw(self, msg):
        self._latest_cmd = (msg.linear.x, msg.angular.z)

    def _publish_cmd_vel(self):
        linear, angular = self._monitor.filter_cmd(*self._latest_cmd)
        out = Twist()
        out.linear.x = linear
        out.angular.z = angular
        self._cmd_vel_pub.publish(out)


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
