"""Minimal post-launch smoke check for apartment_world.launch.py.

Asserts liveness + frame_id correctness on the topics the pre-SLAM audit
(robot/pre-slam-audit.md) flagged as needing an automated check: this is the
kind of check that would have caught two real regressions that previously
required a human to notice by hand - ekf_node silently discarding all input
(no /odometry/filtered), and /joint_states going silent. It also asserts the
INT-006 topic rates frozen in src/shared/contracts/robot-interface.md. It is
deliberately not a general test framework: liveness + frame_id + rate on a
handful of topics, nothing more.
"""
import unittest

import launch_testing
import launch_testing.actions
import rclpy
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from rclpy.node import Node
from sensor_msgs.msg import JointState, LaserScan
from tf2_ros import Buffer, TransformListener

STARTUP_TIMEOUT = 30.0
RATE_WINDOW_S = 4.0
# Generous floor, not precise metrology - rambla-vm is CPU-contended and sim
# rates sag under load. The goal is catching a broken/silent contract (a
# topic gone quiet or wired to the wrong rate), not measuring jitter.
MIN_RATE_FRACTION = 0.4


def generate_test_description():
    sim_pkg_share = get_package_share_directory('rambla_sim')
    apartment_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            f'{sim_pkg_share}/launch/apartment_world.launch.py'
        )
    )

    return LaunchDescription([
        apartment_world,
        launch_testing.actions.ReadyToTest(),
    ])


class TestSmokeTopics(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = Node('test_smoke_topics')

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for_message(self, topic, msg_type, timeout=STARTUP_TIMEOUT):
        received = []
        sub = self.node.create_subscription(
            msg_type, topic, lambda msg: received.append(msg), 10)
        end_time = self.node.get_clock().now().nanoseconds / 1e9 + timeout
        while not received and self.node.get_clock().now().nanoseconds / 1e9 < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.5)
        self.node.destroy_subscription(sub)
        return received[0] if received else None

    def _measured_rate(self, topic, msg_type, window=RATE_WINDOW_S, timeout=STARTUP_TIMEOUT):
        """Confirm liveness, then count messages over `window` wall-clock
        seconds on a fresh subscription. Returns observed Hz, or None if the
        topic never published at all.
        """
        if self._wait_for_message(topic, msg_type, timeout=timeout) is None:
            return None
        count = 0

        def _on_msg(_msg):
            nonlocal count
            count += 1

        sub = self.node.create_subscription(msg_type, topic, _on_msg, 20)
        end_time = self.node.get_clock().now().nanoseconds / 1e9 + window
        while self.node.get_clock().now().nanoseconds / 1e9 < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        return count / window

    def test_odometry_filtered_publishes(self):
        from nav_msgs.msg import Odometry
        msg = self._wait_for_message('/odometry/filtered', Odometry)
        self.assertIsNotNone(
            msg, 'ekf_node produced no /odometry/filtered - see '
            'robot/localization/CLAUDE.md (this previously happened silently '
            'due to a frame_id mismatch)')

    def test_odometry_filtered_rate(self):
        from nav_msgs.msg import Odometry
        hz = self._measured_rate('/odometry/filtered', Odometry)
        self.assertIsNotNone(hz, '/odometry/filtered never published')
        min_hz = 30.0 * MIN_RATE_FRACTION
        self.assertGreater(
            hz, min_hz,
            f'/odometry/filtered observed {hz:.1f} Hz, below the '
            f'{min_hz:.1f} Hz floor for the 30 Hz contract rate in '
            'src/shared/contracts/robot-interface.md - ekf_filter_node may '
            'be stalling under load')

    def test_tf_odom_to_base_footprint(self):
        buffer = Buffer()
        listener = TransformListener(buffer, self.node)
        end_time = self.node.get_clock().now().nanoseconds / 1e9 + STARTUP_TIMEOUT
        found = False
        while not found and self.node.get_clock().now().nanoseconds / 1e9 < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.5)
            found = buffer.can_transform('odom', 'base_footprint', rclpy.time.Time())
        self.node.destroy_subscription(listener.tf_sub)
        self.assertTrue(found, 'odom -> base_footprint transform never appeared on /tf')

    def test_scan_frame_id(self):
        msg = self._wait_for_message('/scan', LaserScan)
        self.assertIsNotNone(msg, '/scan never published')
        self.assertEqual(
            msg.header.frame_id, 'base_scan',
            'raw /scan has the wrong frame_id - check <gz_frame_id> on the '
            'LiDAR sensor block in plugins.xacro; SLAM will silently fail '
            'TF lookups against this topic')

    def test_scan_rate(self):
        hz = self._measured_rate('/scan', LaserScan)
        self.assertIsNotNone(hz, '/scan never published')
        min_hz = 5.0 * MIN_RATE_FRACTION
        self.assertGreater(
            hz, min_hz,
            f'/scan observed {hz:.1f} Hz, below the {min_hz:.1f} Hz floor '
            'for the 5 Hz contract rate in '
            'src/shared/contracts/robot-interface.md - check the LiDAR '
            '<update_rate> in plugins.xacro or the bridge table in '
            'apartment_world.launch.py')

    def test_imu_frame_id(self):
        from sensor_msgs.msg import Imu
        msg = self._wait_for_message('/imu/data', Imu)
        self.assertIsNotNone(msg, '/imu/data never published')
        self.assertEqual(
            msg.header.frame_id, 'imu_link',
            'raw /imu/data has the wrong frame_id - check <gz_frame_id> on '
            'the IMU sensor block in plugins.xacro')

    def test_camera_image_rate(self):
        from sensor_msgs.msg import Image
        hz = self._measured_rate('/camera/image_raw', Image)
        self.assertIsNotNone(hz, '/camera/image_raw never published')
        min_hz = 15.0 * MIN_RATE_FRACTION
        self.assertGreater(
            hz, min_hz,
            f'/camera/image_raw observed {hz:.1f} Hz, below the '
            f'{min_hz:.1f} Hz floor for the 15 Hz contract rate in '
            'src/shared/contracts/robot-interface.md')

    def test_camera_info_rate(self):
        from sensor_msgs.msg import CameraInfo
        hz = self._measured_rate('/camera/camera_info', CameraInfo)
        self.assertIsNotNone(hz, '/camera/camera_info never published')
        min_hz = 15.0 * MIN_RATE_FRACTION
        self.assertGreater(
            hz, min_hz,
            f'/camera/camera_info observed {hz:.1f} Hz, below the '
            f'{min_hz:.1f} Hz floor for the 15 Hz contract rate in '
            'src/shared/contracts/robot-interface.md')

    def test_joint_states_publishes(self):
        msg = self._wait_for_message('/joint_states', JointState)
        self.assertIsNotNone(msg, '/joint_states never published')


@launch_testing.post_shutdown_test()
class TestProcessExit(unittest.TestCase):

    def test_exit_codes(self, proc_info):
        # gz sim doesn't reliably exit within launch_testing's SIGINT grace
        # period and gets escalated to SIGTERM (-15) - confirmed benign
        # (Gazebo itself shuts down fine standalone; this is a
        # launch_testing-teardown timing quirk, not a crash). rclpy nodes
        # (covariance_injector, ekf_node, etc.) exit -2 (SIGINT) on a clean
        # shutdown under launch_testing's teardown - also expected, not a
        # crash. Both allowed here in addition to the default 0.
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, -15])
