"""rclpy wrapper around LocalizationMonitor: subscribes to /particle_cloud
(spread) and /amcl_pose (covariance), publishes the derived confidence
state to /localization_status (std_msgs/String, mirroring rambla_safety's
/control_authority enum-string pattern), and calls nav2_amcl's
/reinitialize_global_localization service both on startup (once amcl is
up) and automatically whenever LOST persists - the concrete "recover
after being moved" behavior M5_PLAN.md's Phase 4 calls for, no operator
action required.

Startup activation is detected via std_srvs/Empty service availability on
/reinitialize_global_localization rather than polling map_server/amcl's
own lifecycle GetState services directly - the service handle only
becomes callable once amcl has stood up its ROS interfaces under
localize.launch.py's lifecycle_manager (autostart:=true), which is
sufficient here without adding a second lifecycle_msgs dependency and a
two-node GetState poll.

All state-machine logic (thresholds, transitions, recovery timing) lives
in localization_state.LocalizationMonitor - this file only computes the
two numeric readings (particle spread, pose covariance magnitude) from
ROS messages and drives that class's tick/request_reinit calls off ROS
subscriptions/timers.
"""
import json
import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.msg import ParticleCloud
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from std_srvs.srv import Empty

from rambla_localization.localization_state import LocalizationMonitor

PARTICLE_CLOUD_TOPIC = '/particle_cloud'
AMCL_POSE_TOPIC = '/amcl_pose'
ODOM_TOPIC = '/odometry/filtered'
LOCALIZATION_STATUS_TOPIC = '/localization_status'
LOCALIZATION_METRICS_TOPIC = '/localization_metrics'
REINITIALIZE_SERVICE = '/reinitialize_global_localization'

MONITOR_RATE_HZ = 5.0
STARTUP_POLL_RATE_HZ = 1.0


class LocalizationMonitorNode(Node):

    def __init__(self):
        super().__init__('localization_monitor')

        self._monitor = LocalizationMonitor(
            clock=lambda: self.get_clock().now().nanoseconds / 1e9)

        self._reinit_client = self.create_client(Empty, REINITIALIZE_SERVICE)
        self._activated = False
        self._reinit_count = 0
        self._last_logged_state = None
        self._spread_m = None
        self._cov_xy_m = None
        self._cov_yaw_rad = None

        self._status_pub = self.create_publisher(
            String, LOCALIZATION_STATUS_TOPIC, 10)
        self._metrics_pub = self.create_publisher(
            String, LOCALIZATION_METRICS_TOPIC, 10)

        # nav2_amcl publishes /particle_cloud BEST_EFFORT (confirmed live via
        # `ros2 topic info -v` against this 1.3.12 build) - a default
        # (RELIABLE) subscription here is QoS-incompatible and silently
        # drops every message, which was live-verified to starve this node
        # of all particle-spread data (state machine never leaves GLOBAL,
        # times out to LOST, reinits forever). qos_profile_sensor_data
        # matches nav2's own convention for this topic.
        self.create_subscription(
            ParticleCloud, PARTICLE_CLOUD_TOPIC, self._on_particle_cloud,
            qos_profile_sensor_data)
        self.create_subscription(
            PoseWithCovarianceStamped, AMCL_POSE_TOPIC, self._on_amcl_pose, 10)
        # Real velocity, not e.g. /control_authority - same source and
        # rationale as localization_probe.py's own /odometry/filtered
        # subscription. Lets the monitor tell "quiet because AMCL is
        # motion-gated and the robot isn't moving" apart from "quiet
        # because something is actually wrong" (see localization_state.py's
        # stationary_stale_timeout_s).
        self.create_subscription(
            Odometry, ODOM_TOPIC, self._on_odom, 10)

        # Polls (not blocking wait_for_service) so the node stays
        # responsive to rclpy.spin() the whole time - cancels itself once
        # activation has happened, see _poll_startup_activation.
        self._startup_timer = self.create_timer(
            1.0 / STARTUP_POLL_RATE_HZ, self._poll_startup_activation)
        self.create_timer(1.0 / MONITOR_RATE_HZ, self._tick)

    def _poll_startup_activation(self):
        if self._activated:
            return
        if not self._reinit_client.service_is_ready():
            return
        self._activated = True
        self._startup_timer.cancel()
        self._call_reinitialize('startup')

    def _call_reinitialize(self, reason):
        # Fire-and-forget, matching AMCL's own service semantics (Empty
        # request/response) - the state machine doesn't wait for the
        # response before optimistically tracking GLOBAL again.
        self._reinit_client.call_async(Empty.Request())
        self._monitor.request_reinit()
        self._reinit_count += 1
        self.get_logger().info(
            f'reinitialize_global_localization issued '
            f'(#{self._reinit_count}, reason={reason})')

    def _on_particle_cloud(self, msg):
        spread_m = self._compute_spread_m(msg.particles)
        if spread_m is not None:
            self._spread_m = spread_m
            self._monitor.on_particle_cloud(spread_m)

    def _on_amcl_pose(self, msg):
        cov = msg.pose.covariance
        cov_xy_m = math.sqrt(max(cov[0], 0.0) + max(cov[7], 0.0))
        cov_yaw_rad = math.sqrt(max(cov[35], 0.0))
        self._cov_xy_m = cov_xy_m
        self._cov_yaw_rad = cov_yaw_rad
        self._monitor.on_amcl_pose(cov_xy_m, cov_yaw_rad)

    def _on_odom(self, msg):
        twist = msg.twist.twist
        self._monitor.on_velocity(twist.linear.x, twist.angular.z)

    def _tick(self):
        state, should_reinit = self._monitor.tick()
        if should_reinit and self._reinit_client.service_is_ready():
            self._call_reinitialize('sustained-LOST')
            state = self._monitor.state
        self._status_pub.publish(String(data=state))

        if state != self._last_logged_state:
            self._last_logged_state = state
            self.get_logger().info(
                f'localization state -> {state} '
                f'spread={self._spread_m} cov_xy={self._cov_xy_m} '
                f'cov_yaw={self._cov_yaw_rad} '
                f'reinit_count={self._reinit_count}')

        self._metrics_pub.publish(String(data=json.dumps({
            'state': state,
            'spread_m': self._spread_m,
            'cov_xy_m': self._cov_xy_m,
            'cov_yaw_rad': self._cov_yaw_rad,
            'reinit_count': self._reinit_count,
            'stamp': self.get_clock().now().nanoseconds / 1e9,
        })))

    @staticmethod
    def _compute_spread_m(particles):
        """Weighted std of particle xy position from the weighted mean -
        a single scalar radius: small once AMCL has converged to one
        hypothesis, large while multiple/uniform hypotheses remain."""
        if not particles:
            return None
        total_weight = sum(p.weight for p in particles)
        if total_weight <= 0.0:
            xs = [p.pose.position.x for p in particles]
            ys = [p.pose.position.y for p in particles]
            mean_x = sum(xs) / len(xs)
            mean_y = sum(ys) / len(ys)
            var = sum(
                (x - mean_x) ** 2 + (y - mean_y) ** 2
                for x, y in zip(xs, ys)
            ) / len(xs)
            return math.sqrt(var)
        mean_x = sum(
            p.weight * p.pose.position.x for p in particles) / total_weight
        mean_y = sum(
            p.weight * p.pose.position.y for p in particles) / total_weight
        var = sum(
            p.weight * (
                (p.pose.position.x - mean_x) ** 2
                + (p.pose.position.y - mean_y) ** 2)
            for p in particles
        ) / total_weight
        return math.sqrt(var)


def main():
    rclpy.init()
    node = LocalizationMonitorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
