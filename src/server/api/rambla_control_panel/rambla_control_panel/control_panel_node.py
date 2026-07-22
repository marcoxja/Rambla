"""rclpy side of the gateway (M4, CTL-009): publishes manual drive intent to
/cmd_vel_teleop from relay-forwarded joystick input (with a deadman watchdog
so a dropped connection can't leave the robot driving forever - see
deadman.py), republishes sensor/diagnostics data, and forwards the
already-compressed camera stream on demand. gateway_client.py owns the
outbound WSS connections to the hosted relay (Cloudflare Worker + per-robot
Durable Object - see src/shared/contracts/relay-protocol.md) and wires this
node's on_* sinks / camera-subscribe requests; this module has no socket
code of its own so it stays testable/runnable standalone.

/cmd_vel_teleop is *intent*, not the actuator-facing command: rambla_safety's
SafetyNode is the sole final /cmd_vel publisher and arbitrates AUTO
(/cmd_vel_raw, from rambla_traversal) vs MANUAL (/cmd_vel_teleop) authority -
see rambla_safety/control_authority.py. This node only publishes while the
deadman is actively receiving fresh commands (is_active), plus exactly one
final zero on the tick it goes inactive, so SafetyNode's own manual-liveness
timeout sees real silence promptly instead of a forever-repeating stream of
zeros that would keep MANUAL authority engaged after disconnect.

Runs rclpy.spin on a background thread since rclpy is not asyncio-native;
gateway_client's asyncio loop calls this node's methods directly (publishers
are thread-safe, and camera-subscription changes are debounced through a
polled flag rather than called cross-thread - see _sync_camera_subscription)
and this node pushes inbound data to asyncio via the sinks gateway_client
wires up per connection.
"""
import threading
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message
from sensor_msgs.msg import CompressedImage, Imu, LaserScan
from std_msgs.msg import String

from rambla_control_panel.deadman import DeadmanTimer

CMD_VEL_RATE_HZ = 20.0
SENSOR_PUBLISH_RATE_HZ = 8.0
DIAGNOSTICS_RATE_HZ = 1.0
CAMERA_SUBSCRIPTION_POLL_RATE_HZ = 5.0
TELEMETRY_SUBSCRIPTION_POLL_RATE_HZ = 5.0

# /scan is the raw gz-sim topic - plugins.xacro's <gz_frame_id> tags already
# give it a plain frame_id at the source, so no downstream frame-ID
# republish is needed. /imu/data_fixed is rambla_localization's
# covariance_injector republish (still needed for non-zero covariance - see
# covariance_injector.py). /odometry/filtered is the EKF's fused output
# (ekf.yaml), a better "ground truth" position readout than raw /odom.
# /amcl_pose (M5 Phase 7) is nav2_amcl's map-frame pose+covariance -
# PoseWithCovarianceStamped, default (RELIABLE) QoS, same as
# localization_monitor's own subscription. /localization_status is the
# monitor's confidence state machine (std_msgs/String) - folded into the
# sensor_pose snapshot below rather than forwarded as its own sink, since it
# has no meaning to a viewer except as an attribute of the pose it qualifies.
# /camera/image_raw/compressed is the JPEG side-channel from M3
# (camera_compressor) - forwarded verbatim, never decoded/re-encoded here
# (this is what removes cv_bridge/cv2 from the gateway's idle-CPU cost -
# see robot/resource-budget/CLAUDE.md's measurement log).
SCAN_TOPIC = '/scan'
IMU_TOPIC = '/imu/data_fixed'
ODOM_TOPIC = '/odometry/filtered'
AMCL_POSE_TOPIC = '/amcl_pose'
LOCALIZATION_STATUS_TOPIC = '/localization_status'
CAMERA_COMPRESSED_TOPIC = '/camera/image_raw/compressed'
CONTROL_AUTHORITY_TOPIC = '/control_authority'
CMD_VEL_TELEOP_TOPIC = '/cmd_vel_teleop'


class ControlPanelNode(Node):

    def __init__(self):
        super().__init__('control_panel_node')

        self._deadman = DeadmanTimer(clock=time.monotonic)
        self._teleop_was_active = False

        # Pluggable sinks set by gateway_client.py once its asyncio loop/ws
        # connections exist - kept as plain callables (not a hard dependency
        # on websockets/asyncio here) so this node stays testable/runnable
        # standalone.
        self.on_scan = None
        self.on_imu = None
        self.on_odom = None
        self.on_pose = None
        self.on_diagnostics = None
        self.on_control_authority = None
        self.on_camera_frame = None

        self._cmd_vel_pub = self.create_publisher(Twist, CMD_VEL_TELEOP_TOPIC, 10)
        self.create_timer(1.0 / CMD_VEL_RATE_HZ, self._publish_cmd_vel)

        # Sensor subscriptions are on-demand, same shape as the camera below:
        # ungated, this measured at ~80% mean CPU always-on regardless of
        # viewers -- IMU alone publishes at ~95Hz and every tick was
        # processed and forwarded even with zero browsers attached, which is
        # exactly the always-on cost the camera path was already fixed to
        # avoid (see robot/resource-budget/CLAUDE.md's measurement log).
        # gateway_client flips _telemetry_wanted from the
        # asyncio thread on telemetry_subscribe/telemetry_unsubscribe; the
        # poll timer below does the actual create_subscription/
        # destroy_subscription on the rclpy spin thread, same reasoning as
        # _sync_camera_subscription.
        self._scan_sub = None
        self._imu_sub = None
        self._odom_sub = None
        self._pose_sub = None
        self._localization_status_sub = None
        self._latest_localization_status = None
        self._telemetry_wanted = False
        self.create_timer(
            1.0 / TELEMETRY_SUBSCRIPTION_POLL_RATE_HZ, self._sync_telemetry_subscriptions)
        self._last_control_authority_mode = None
        self.create_subscription(
            String, CONTROL_AUTHORITY_TOPIC, self._on_control_authority, 10)

        # Camera subscription is on-demand: created only
        # while >=1 browser viewer is attached, destroyed when the last one
        # leaves. gateway_client flips _camera_wanted from the asyncio
        # thread on video_subscribe/video_unsubscribe (a plain bool, GIL-
        # atomic); the poll timer below is what actually calls
        # create_subscription/destroy_subscription, and it always runs on
        # the rclpy spin thread (it's a timer callback), so it never races
        # the executor's own waitset the way a cross-thread rclpy call
        # could.
        self._camera_sub = None
        self._camera_wanted = False
        self.create_timer(
            1.0 / CAMERA_SUBSCRIPTION_POLL_RATE_HZ, self._sync_camera_subscription)

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

    # --- inbound from the relay (called from gateway_client's asyncio loop) ---

    def submit_drive_command(self, linear, angular):
        self._deadman.on_command(linear, angular)

    def submit_disconnect(self):
        self._deadman.on_disconnect()

    def request_camera_subscribe(self):
        self._camera_wanted = True

    def request_camera_unsubscribe(self):
        self._camera_wanted = False

    def request_telemetry_subscribe(self):
        self._telemetry_wanted = True

    def request_telemetry_unsubscribe(self):
        self._telemetry_wanted = False

    # --- outbound to /cmd_vel_teleop, at a steady rate regardless of
    #     relay event jitter - see deadman.py's module docstring ---

    def _publish_cmd_vel(self):
        # command() must run first - it's what actually detects/applies a
        # timeout trip (is_active alone doesn't advance the clock check).
        linear, angular = self._deadman.command()
        active = self._deadman.is_active
        if not active and not self._teleop_was_active:
            # Already silent last tick and still silent - nothing new to
            # report. Publishing zeros here forever would look, to
            # SafetyNode's manual-liveness timeout, indistinguishable from
            # a live operator holding still, and MANUAL authority would
            # never release back to AUTO.
            return
        self._teleop_was_active = active
        msg = Twist()
        msg.linear.x = linear
        msg.angular.z = angular
        self._cmd_vel_pub.publish(msg)

    # --- sensor subscriptions -> relay sinks ---

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

    def _on_pose(self, msg):
        self._topic_last_seen[AMCL_POSE_TOPIC] = time.monotonic()
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        self._latest_sensor_data['pose'] = {
            'position': {'x': p.x, 'y': p.y, 'z': p.z},
            'orientation': {'x': o.x, 'y': o.y, 'z': o.z, 'w': o.w},
            'covariance': list(msg.pose.covariance),
            'status': self._latest_localization_status,
        }

    def _on_localization_status(self, msg):
        self._topic_last_seen[LOCALIZATION_STATUS_TOPIC] = time.monotonic()
        self._latest_localization_status = msg.data

    def _sync_telemetry_subscriptions(self):
        if self._telemetry_wanted and self._scan_sub is None:
            self._scan_sub = self.create_subscription(LaserScan, SCAN_TOPIC, self._on_scan, 10)
            self._imu_sub = self.create_subscription(Imu, IMU_TOPIC, self._on_imu, 10)
            self._odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self._on_odom, 10)
            self._pose_sub = self.create_subscription(
                PoseWithCovarianceStamped, AMCL_POSE_TOPIC, self._on_pose, 10)
            self._localization_status_sub = self.create_subscription(
                String, LOCALIZATION_STATUS_TOPIC, self._on_localization_status, 10)
        elif not self._telemetry_wanted and self._scan_sub is not None:
            self.destroy_subscription(self._scan_sub)
            self.destroy_subscription(self._imu_sub)
            self.destroy_subscription(self._odom_sub)
            self.destroy_subscription(self._pose_sub)
            self.destroy_subscription(self._localization_status_sub)
            self._scan_sub = None
            self._imu_sub = None
            self._odom_sub = None
            self._pose_sub = None
            self._localization_status_sub = None
            self._latest_localization_status = None
            # Drop stale values rather than re-forwarding the last snapshot
            # forever once subscriptions (and thus fresh data) stop.
            self._latest_sensor_data.clear()

    def _publish_sensor_snapshot(self):
        if not self._telemetry_wanted:
            return
        # Single fixed-rate tick broadcasts whatever's latest per sensor,
        # regardless of how many messages actually arrived since the last
        # tick - see _on_scan/_on_imu/_on_odom, which only overwrite.
        if self.on_scan and 'scan' in self._latest_sensor_data:
            self.on_scan(self._latest_sensor_data['scan'])
        if self.on_imu and 'imu' in self._latest_sensor_data:
            self.on_imu(self._latest_sensor_data['imu'])
        if self.on_odom and 'odom' in self._latest_sensor_data:
            self.on_odom(self._latest_sensor_data['odom'])
        if self.on_pose and 'pose' in self._latest_sensor_data:
            self.on_pose(self._latest_sensor_data['pose'])

    def _on_control_authority(self, msg):
        self._topic_last_seen[CONTROL_AUTHORITY_TOPIC] = time.monotonic()
        # Edge-triggered, not the topic's own 20Hz rate: the relay contract
        # (relay-protocol.md) only needs to know the current mode, and
        # forwarding every tick would make the control channel far chattier
        # than the sensor/diagnostics traffic it's meant to stay light next
        # to (and fights WebSocket Hibernation between real interactions).
        if msg.data == self._last_control_authority_mode:
            return
        self._last_control_authority_mode = msg.data
        if self.on_control_authority:
            self.on_control_authority(msg.data)

    def _sync_camera_subscription(self):
        if self._camera_wanted and self._camera_sub is None:
            self._camera_sub = self.create_subscription(
                CompressedImage, CAMERA_COMPRESSED_TOPIC, self._on_camera_frame, 10)
        elif not self._camera_wanted and self._camera_sub is not None:
            self.destroy_subscription(self._camera_sub)
            self._camera_sub = None

    def _on_camera_frame(self, msg):
        self._topic_last_seen[CAMERA_COMPRESSED_TOPIC] = time.monotonic()
        if self.on_camera_frame:
            self.on_camera_frame(bytes(msg.data))

    # --- lightweight diagnostics: node/topic liveness, no
    #     diagnostic_aggregator infra - see plan for why this is enough
    #     for a first slice ---

    def _refresh_liveness_subscriptions(self):
        if not self._telemetry_wanted:
            # Same on-demand gate as the sensor subscriptions (Phase 5
            # follow-up): this probes and subscribes to every topic in the
            # graph, which is exactly as pointless to keep running with zero
            # viewers as the sensor callbacks were.
            if self._liveness_subs:
                for sub in self._liveness_subs.values():
                    self.destroy_subscription(sub)
                self._liveness_subs.clear()
                self._topic_last_seen.clear()
            return
        # The topics this node already type-subscribes to directly (SCAN_TOPIC
        # etc., plus CONTROL_AUTHORITY_TOPIC) update _topic_last_seen in their
        # own callbacks above. CAMERA_COMPRESSED_TOPIC is also excluded here
        # even though its dedicated subscription is only sometimes present
        # (_sync_camera_subscription) - a generic liveness probe would still
        # pay the same CompressedImage deserialization cost as a real viewer
        # subscription, which is exactly the always-on cost the on-demand
        # design exists to avoid. Its diagnostics "last seen" is only ever
        # fresh while a viewer is actually attached - that's intentional,
        # not a bug.
        #
        # Every OTHER topic in the graph (/odom, /joint_states, /tf, ...)
        # needs its own liveness probe or the Nodes tab shows "never" for
        # topics that are actually alive - this generic (type-erased)
        # subscription just timestamps arrival, it doesn't need to decode
        # the message.
        tracked_directly = {SCAN_TOPIC, IMU_TOPIC, ODOM_TOPIC, AMCL_POSE_TOPIC,
                            LOCALIZATION_STATUS_TOPIC, CONTROL_AUTHORITY_TOPIC,
                            CAMERA_COMPRESSED_TOPIC}
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
        if not self._telemetry_wanted:
            return
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
    # Entry point: brings up rclpy on a background thread and runs
    # gateway_client's asyncio loop (outbound WSS to the relay - see
    # src/shared/contracts/relay-protocol.md) on the main thread.
    from rambla_control_panel.gateway_client import run_gateway

    rclpy.init()
    node = ControlPanelNode()
    spin_in_background(node)
    try:
        run_gateway(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
