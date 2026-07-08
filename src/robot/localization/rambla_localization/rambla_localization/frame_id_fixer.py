"""Rewrites frame_id/child_frame_id on /odom, /imu/data, /scan, and /camera/*.

gz-sim Harmonic's OdometryPublisher, IMU, gpu_lidar, and camera sensor
systems publish these messages with the model name prepended (e.g.
"rambla/odom", "rambla/imu_link/rambla_imu",
"rambla/base_scan/laser_scan_sensor") even when the SDF's <odom_frame>,
<robot_base_frame>, and sensor <frame_id> are set to the plain names used
by robot_state_publisher's URDF-derived TF tree (confirmed against gz-sim
8.11.0's own source and runtime behavior - not a config or bridge-level
fix). Since robot_localization's ekf_node matches incoming frame_id against
its configured world_frame/odom_frame and does a TF lookup for IMU data, the
prefix mismatch causes it to silently discard every measurement - the same
mismatch would break any future TF-based consumer of /scan or /camera/*
(e.g. SLAM) the same way. This node strips the prefix on all four so
downstream consumers see the plain frame names that match TF.

/odom and /imu/data are also given hand-picked, static, non-zero covariance
here. gz-sim's OdometryPublisher and IMU sensor systems publish all-zero
covariance, which starves robot_localization's ekf_node of any real
per-sensor trust signal and causes /odometry/filtered to diverge unboundedly
under real motion (see robot/pre-slam-audit.md). These are placeholder
values sized to what rambla_localization/config/ekf.yaml actually trusts
(odom0_config: vx/vy/vyaw; imu0_config: orientation/angular velocity/ax,ay) -
not a real sensor noise model, consistent with DESIGN_SPEC.md PHY-004's
"placeholder until real hardware" approach. Revisit if/when gz-sim populates
real covariance, or once real hardware provides it.
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image, Imu, LaserScan

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


class FrameIdFixer(Node):

    def __init__(self):
        super().__init__('frame_id_fixer')

        self.odom_pub = self.create_publisher(Odometry, '/odom_fixed', 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._on_odom, 10)

        self.imu_pub = self.create_publisher(Imu, '/imu/data_fixed', 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self._on_imu, 10)

        self.scan_pub = self.create_publisher(LaserScan, '/scan_fixed', 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self._on_scan, 10)

        self.image_pub = self.create_publisher(Image, '/camera/image_raw_fixed', 10)
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self._on_image, 10)

        self.camera_info_pub = self.create_publisher(
            CameraInfo, '/camera/camera_info_fixed', 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera_info', self._on_camera_info, 10)

    def _on_odom(self, msg):
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_footprint'
        msg.twist.covariance = ODOM_TWIST_COVARIANCE
        self.odom_pub.publish(msg)

    def _on_imu(self, msg):
        msg.header.frame_id = 'imu_link'
        msg.orientation_covariance = IMU_ORIENTATION_COVARIANCE
        msg.angular_velocity_covariance = IMU_ANGULAR_VELOCITY_COVARIANCE
        msg.linear_acceleration_covariance = IMU_LINEAR_ACCELERATION_COVARIANCE
        self.imu_pub.publish(msg)

    def _on_scan(self, msg):
        msg.header.frame_id = 'base_scan'
        self.scan_pub.publish(msg)

    def _on_image(self, msg):
        msg.header.frame_id = 'camera_link_optical'
        self.image_pub.publish(msg)

    def _on_camera_info(self, msg):
        msg.header.frame_id = 'camera_link_optical'
        self.camera_info_pub.publish(msg)


def main():
    rclpy.init()
    node = FrameIdFixer()
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
