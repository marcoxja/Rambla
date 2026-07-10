"""Injects placeholder covariance onto /odom and /imu/data for the EKF.

gz-sim Harmonic's OdometryPublisher and IMU sensor systems publish all-zero
covariance, which starves robot_localization's ekf_node of any real
per-sensor trust signal and causes /odometry/filtered to diverge unboundedly
under real motion (see robot/pre-slam-audit.md). This node subscribes to raw
/odom and /imu/data, stamps in hand-picked, static, non-zero covariance
(ODOM_TWIST_COVARIANCE / IMU_*_COVARIANCE below), and republishes on
/odom_fixed and /imu/data_fixed, which ekf.yaml's odom0/imu0 consume. These
are placeholder values sized to what rambla_localization/config/ekf.yaml
actually trusts (odom0_config: vx/vy/vyaw; imu0_config: orientation/angular
velocity/ax,ay) - not a real sensor noise model, consistent with
DESIGN_SPEC.md PHY-004's "placeholder until real hardware" approach. Revisit
if/when gz-sim populates real covariance, or once real hardware provides it.

/scan, /camera/image_raw, and /camera/camera_info are NOT touched here
anymore: gz Harmonic 8.11's <gz_frame_id> (LiDAR/IMU) and
<optical_frame_id> (camera) tags in plugins.xacro now set clean, unprefixed
frame_ids at the source (confirmed empirically against the running sim -
see plugins.xacro's inline comments), so those raw topics no longer need a
downstream frame_id-rewriting republish. This node's only remaining
frame_id rewrite is on /odom: the Gazebo OdometryPublisher plugin is a
plugin, not a sensor, so <gz_frame_id> doesn't apply to it - the odom
frame_id/child_frame_id assignment below stays here for that reason (and
is essentially free since this node already republishes /odom for
covariance).
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

# Diagonal-only placeholder covariance, row-major 6x6 (x,y,z,rot_x,rot_y,rot_z
# twice-nested per ROS convention). Off-diagonal entries MUST stay 0 (a
# covariance matrix with large off-diagonal cross-terms is not positive
# semi-definite and corrupts the filter - confirmed via a live /diagnostics
# warning: "position (34)... extremely large" - 34 is an OFF-diagonal
# row5/col4 cross-term, not a diagonal one). Only the diagonal entries
# ekf.yaml actually trusts (odom0_config: vx, vy, vyaw) get a small,
# plausible value; other diagonal entries are set large so an untrusted
# field can't accidentally look authoritative if config ever changes.
_UNTRUSTED = 1e6
ODOM_TWIST_COVARIANCE = [0.0] * 36
for _i in range(6):
    ODOM_TWIST_COVARIANCE[_i * 6 + _i] = _UNTRUSTED
ODOM_TWIST_COVARIANCE[0 * 6 + 0] = 0.01   # vx
ODOM_TWIST_COVARIANCE[1 * 6 + 1] = 0.01   # vy
ODOM_TWIST_COVARIANCE[5 * 6 + 5] = 0.02   # vyaw

# IMU covariances are flat 3x3 diagonals (x,y,z / roll,pitch,yaw). All three
# fields below are trusted per ekf.yaml's imu0_config (orientation and
# angular velocity fully; linear acceleration x/y tentatively, z untrusted).
IMU_ORIENTATION_COVARIANCE = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
IMU_ANGULAR_VELOCITY_COVARIANCE = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
IMU_LINEAR_ACCELERATION_COVARIANCE = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, _UNTRUSTED]


class CovarianceInjector(Node):

    def __init__(self):
        super().__init__('covariance_injector')

        self.odom_pub = self.create_publisher(Odometry, '/odom_fixed', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._on_odom, 10)

        self.imu_pub = self.create_publisher(Imu, '/imu/data_fixed', 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self._on_imu, 10)

    def _on_odom(self, msg):
        # Gazebo's OdometryPublisher plugin can't take <gz_frame_id> (it's
        # sensor-level only), so this rewrite stays here - see module
        # docstring.
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.twist.covariance = ODOM_TWIST_COVARIANCE
        self.odom_pub.publish(msg)

    def _on_imu(self, msg):
        msg.orientation_covariance = IMU_ORIENTATION_COVARIANCE
        msg.angular_velocity_covariance = IMU_ANGULAR_VELOCITY_COVARIANCE
        msg.linear_acceleration_covariance = IMU_LINEAR_ACCELERATION_COVARIANCE
        self.imu_pub.publish(msg)


def main():
    rclpy.init()
    node = CovarianceInjector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        # rclpy's own SIGINT handler can already have called shutdown() by
        # the time this runs (confirmed via a live launch_testing SIGINT
        # teardown, which surfaced this for the first time - this node was
        # previously only ever stopped by killing the whole launch, not
        # individually SIGINT'd) - guard against the resulting
        # "rcl_shutdown already called" RCLError on a redundant call.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
