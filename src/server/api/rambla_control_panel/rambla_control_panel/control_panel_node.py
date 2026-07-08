"""rclpy side of the control panel: publishes /cmd_vel from browser joystick
input (with a deadman watchdog so a dropped connection can't leave the robot
driving forever - see deadman.py), and republishes sensor/camera/diagnostic
data for server.py's websocket and MJPEG endpoints to consume.

Runs rclpy.spin on a background thread since rclpy is not asyncio-native;
FastAPI handlers call this node's methods directly (publishers are
thread-safe) and this node pushes inbound data to asyncio via the
loop/queue handed to it by server.py.
"""
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import Image, Imu, LaserScan

from rambla_control_panel.deadman import DeadmanTimer

CMD_VEL_RATE_HZ = 20.0
SENSOR_PUBLISH_RATE_HZ = 8.0
DIAGNOSTICS_RATE_HZ = 1.0

# Frame-corrected topics from rambla_localization's frame_id_fixer, not the
# raw gz-sim topics - see frame_id_fixer.py for why the raw ones have wrong
# frame_ids. /odometry/filtered is the EKF's fused output (ekf.yaml), a
# better "ground truth" position readout than raw /odom_fixed.
SCAN_TOPIC = '/scan_fixed'
IMU_TOPIC = '/imu/data_fixed'
ODOM_TOPIC = '/odometry/filtered'
CAMERA_TOPIC = '/camera/image_raw_fixed'

CAMERA_JPEG_QUALITY = 70


class ControlPanelNode(Node):

    def __init__(self):
        super().__init__('control_panel_node')

        self._cv_bridge = CvBridge()
        self._deadman = DeadmanTimer(clock=time.monotonic)
        self._latest_jpeg = None
        self._latest_jpeg_lock = threading.Lock()

        # Pluggable sinks set by server.py once its event loop exists - kept
        # as plain callables (not a hard dependency on FastAPI/asyncio here)
        # so this node stays testable/runnable standalone.
        self.on_scan = None
        self.on_imu = None
        self.on_odom = None
        self.on_diagnostics = None

        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_timer(1.0 / CMD_VEL_RATE_HZ, self._publish_cmd_vel)

        self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, 10)
        self.create_subscription(Imu, IMU_TOPIC, self._on_imu, 10)
        self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)
        self.create_subscription(Image, CAMERA_TOPIC, self._on_image, 10)

        # Callbacks above only overwrite this - the timer below is the only
        # thing that ever reads it and calls the sinks, so a sensor
        # publishing far faster than SENSOR_PUBLISH_RATE_HZ (IMU is ~95Hz)
        # can't spawn a broadcast task per message. Previously each callback
        # called its sink directly, which meant one asyncio.create_task per
        # incoming message with no ceiling - see bugs/ soak-test writeup.
        self._latest_sensor_data = {}
        self.create_timer(1.0 / SENSOR_PUBLISH_RATE_HZ, self._publish_sensor_snapshot)

        self.create_timer(1.0 / DIAGNOSTICS_RATE_HZ, self._publish_diagnostics)
        self._topic_last_seen = {}
        self._liveness_subs = {}
        self.create_timer(1.0 / DIAGNOSTICS_RATE_HZ, self._refresh_liveness_subscriptions)

    # --- inbound from browser (called from FastAPI's event loop/thread) ---

    def submit_drive_command(self, linear, angular):
        self._deadman.on_command(linear, angular)

    def submit_disconnect(self):
        self._deadman.on_disconnect()

    def latest_camera_jpeg(self):
        with self._latest_jpeg_lock:
            return self._latest_jpeg

    # --- outbound to /cmd_vel, at a steady rate regardless of browser
    #     event jitter - see deadman.py's module docstring ---

    def _publish_cmd_vel(self):
        linear, angular = self._deadman.command()
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_vel_pub.publish(msg)

    # --- sensor subscriptions -> websocket sinks ---

    def _on_scan(self, msg):
        self._topic_last_seen[SCAN_TOPIC] = time.monotonic()
        self._latest_sensor_data['scan'] = {
            'angle_min': msg.angle_min,
            'angle_max': msg.angle_max,
            'angle_increment': msg.angle_increment,
            'range_min': msg.range_min,
            'range_max': msg.range_max,
            'ranges': list(msg.ranges),
        }

    def _on_imu(self, msg):
        self._topic_last_seen[IMU_TOPIC] = time.monotonic()
        self._latest_sensor_data['imu'] = {
            'orientation': {
                'x': msg.orientation.x, 'y': msg.orientation.y,
                'z': msg.orientation.z, 'w': msg.orientation.w,
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x, 'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z,
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x, 'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z,
            },
        }

    def _on_odom(self, msg):
        self._topic_last_seen[ODOM_TOPIC] = time.monotonic()
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        v = msg.twist.twist
        self._latest_sensor_data['odom'] = {
            'position': {'x': p.x, 'y': p.y, 'z': p.z},
            'orientation': {'x': o.x, 'y': o.y, 'z': o.z, 'w': o.w},
            'linear_velocity': {'x': v.linear.x, 'y': v.linear.y},
            'angular_velocity_z': v.angular.z,
        }

    def _publish_sensor_snapshot(self):
        # Single fixed-rate tick broadcasts whatever's latest per sensor,
        # regardless of how many messages actually arrived since the last
        # tick - see _on_scan/_on_imu/_on_odom, which only overwrite.
        if self.on_scan and 'scan' in self._latest_sensor_data:
            self.on_scan(self._latest_sensor_data['scan'])
        if self.on_imu and 'imu' in self._latest_sensor_data:
            self.on_imu(self._latest_sensor_data['imu'])
        if self.on_odom and 'odom' in self._latest_sensor_data:
            self.on_odom(self._latest_sensor_data['odom'])

    def _on_image(self, msg):
        self._topic_last_seen[CAMERA_TOPIC] = time.monotonic()
        frame = self._cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        ok, jpeg = cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, CAMERA_JPEG_QUALITY])
        if ok:
            with self._latest_jpeg_lock:
                self._latest_jpeg = jpeg.tobytes()

    # --- lightweight diagnostics: node/topic liveness, no
    #     diagnostic_aggregator infra - see plan for why this is enough
    #     for a first slice ---

    def _refresh_liveness_subscriptions(self):
        # The four topics this node already type-subscribes to (SCAN_TOPIC
        # etc.) update _topic_last_seen directly in their callbacks above.
        # Every OTHER topic in the graph (/scan, /odom, /joint_states, /tf,
        # ...) needs its own liveness probe or the Nodes tab shows "never"
        # for topics that are actually alive - this generic (type-erased)
        # subscription just timestamps arrival, it doesn't need to decode
        # the message.
        tracked_directly = {SCAN_TOPIC, IMU_TOPIC, ODOM_TOPIC, CAMERA_TOPIC}
        live_topics = {name for name, _types in self.get_topic_names_and_types()}

        # Drop subscriptions for topics that have left the graph (e.g. a
        # node bounced and briefly changed its topic set) - without this,
        # _liveness_subs/_topic_last_seen grow one entry per topic name
        # ever seen, for the process lifetime, with no reclamation.
        gone = set(self._liveness_subs) - live_topics
        for topic_name in gone:
            self.destroy_subscription(self._liveness_subs.pop(topic_name))
            self._topic_last_seen.pop(topic_name, None)

        for topic_name, types in self.get_topic_names_and_types():
            if topic_name in tracked_directly or topic_name in self._liveness_subs:
                continue
            if not types:
                continue
            try:
                msg_class = get_message(types[0])
                sub = self.create_subscription(
                    msg_class,
                    topic_name,
                    lambda _msg, t=topic_name: self._topic_last_seen.__setitem__(
                        t, time.monotonic()),
                    10)
            except Exception:
                # /tf, /tf_static, /parameter_events, etc. use QoS profiles
                # or message types this best-effort liveness probe doesn't
                # need to handle specially - just skip, the topic still
                # shows up in the Nodes tab, only its "last seen" age is
                # left as None ("never") rather than tracked.
                continue
            self._liveness_subs[topic_name] = sub

    def _publish_diagnostics(self):
        if not self.on_diagnostics:
            return
        now = time.monotonic()
        node_names = [
            f'{ns.rstrip("/")}/{name}' if ns != '/' else f'/{name}'
            for name, ns in self.get_node_names_and_namespaces()
        ]
        topics = []
        for topic_name, _types in self.get_topic_names_and_types():
            last_seen = self._topic_last_seen.get(topic_name)
            topics.append({
                'topic': topic_name,
                'age_s': (now - last_seen) if last_seen is not None else None,
            })
        self.on_diagnostics({'nodes': sorted(node_names), 'topics': topics})


def spin_in_background(node):
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    return thread


def main():
    # Entry point: brings up rclpy on a background thread and runs the
    # FastAPI/uvicorn server (which wires this node's on_* sinks) on the
    # main thread - see server.py:run().
    from rambla_control_panel.server import run

    rclpy.init()
    node = ControlPanelNode()
    spin_in_background(node)
    try:
        run(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
