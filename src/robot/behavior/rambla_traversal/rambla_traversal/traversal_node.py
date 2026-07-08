"""rclpy wrapper around WanderBehavior: subscribes to /scan_fixed, publishes
reactive drive commands to /cmd_vel_raw (never /cmd_vel directly - always
passes through rambla_safety), and stops itself after a fixed time budget
so a mapping-recording session has a clean, reproducible end. Tracks
distance traveled via /odometry/filtered purely for the run-summary log,
not as a stopping condition (time budget is simpler and sufficient given
the apartment world's scale - see wander_behavior.py's module docstring).
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from rambla_traversal.wander_behavior import WanderBehavior

CMD_RATE_HZ = 10.0
SCAN_TOPIC = '/scan_fixed'
ODOM_TOPIC = '/odometry/filtered'
CMD_VEL_RAW_TOPIC = '/cmd_vel_raw'

# Matches the SLAM recording plan's "~30-60s, crosses one doorway" target
# (real-map-plan-2026-07-06.md Phase 1) - long enough to reliably reach and
# cross at least one doorway in the current apartment world at
# DEFAULT_FORWARD_SPEED, short enough to keep the resulting bag small.
DEFAULT_TRAVERSAL_DURATION_S = 45.0


class TraversalNode(Node):

    def __init__(self):
        super().__init__('traversal_node')

        self.declare_parameter('traversal_duration_s', DEFAULT_TRAVERSAL_DURATION_S)
        self._duration_s = self.get_parameter('traversal_duration_s').value

        self._behavior = WanderBehavior()
        self._latest_scan = None
        self._start_time = None
        self._finished = False
        self._last_position = None
        self._distance_traveled_m = 0.0

        self._cmd_pub = self.create_publisher(Twist, CMD_VEL_RAW_TOPIC, 10)
        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)
        self.create_timer(1.0 / CMD_RATE_HZ, self._tick)

        self.get_logger().info(
            f'Traversal starting: {self._duration_s:.0f}s budget')

    def _on_scan(self, msg):
        self._latest_scan = msg

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        if self._last_position is not None:
            dx = p.x - self._last_position[0]
            dy = p.y - self._last_position[1]
            self._distance_traveled_m += math.hypot(dx, dy)
        self._last_position = (p.x, p.y)

    def _tick(self):
        if self._finished:
            return

        now = self.get_clock().now()
        if self._start_time is None:
            self._start_time = now

        elapsed_s = (now - self._start_time).nanoseconds / 1e9
        if elapsed_s >= self._duration_s:
            self._finished = True
            self._cmd_pub.publish(Twist())
            self.get_logger().info(
                f'Traversal finished: {elapsed_s:.1f}s elapsed, '
                f'{self._distance_traveled_m:.2f}m traveled')
            return

        if self._latest_scan is None:
            return

        linear, angular = self._behavior.next_command(
            self._latest_scan.ranges,
            self._latest_scan.angle_min,
            self._latest_scan.angle_increment)
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = TraversalNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
