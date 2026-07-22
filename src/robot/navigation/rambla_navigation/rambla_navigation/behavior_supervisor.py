"""rclpy wrapper around BehaviorSupervisor: subscribes to /goal_pose
(RViz-compatible geometry_msgs/PoseStamped), /localization_status,
/control_authority, /cmd_vel_nav, and /cmd_vel_probe; drives a NavigateToPose
action client against bt_navigator; and publishes the muxed result to
/cmd_vel_raw (this node is the sole writer - see M7_PLAN.md and
control-arbitration-layers.md) plus /behavior_mode (String enum) at a
steady rate, the same pattern rambla_safety's SafetyNode uses to decouple
publish rate from sensor/command jitter.

All mode-selection/gating/dispatch-vs-hold logic lives in the new, pure
supervisor_state.BehaviorSupervisor - same shape as this repo's
LocalizationMonitor/ProbeBehavior/ControlAuthority. This file only wires ROS
subscriptions/timer/action-client to it and publishes its output.

The action goal is fired fire-and-track, not fire-and-forget: dispatch and
cancel are driven synchronously by BehaviorSupervisor.tick()'s returned
action, and the eventual result (success or failure alike) is fed back via
on_goal_finished() so the mux never gets wedged believing a finished goal is
still active - the same "drive a service/action off a pure state machine's
signal, then report the outcome back" shape localization_monitor.py uses for
/reinitialize_global_localization.
"""
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String

from rambla_navigation.supervisor_state import (
    BehaviorSupervisor,
    CANCEL_GOAL,
    DISPATCH_GOAL,
)

CMD_VEL_RAW_TOPIC = '/cmd_vel_raw'
CMD_VEL_NAV_TOPIC = '/cmd_vel_nav'
CMD_VEL_PROBE_TOPIC = '/cmd_vel_probe'
GOAL_POSE_TOPIC = '/goal_pose'
LOCALIZATION_STATUS_TOPIC = '/localization_status'
CONTROL_AUTHORITY_TOPIC = '/control_authority'
BEHAVIOR_MODE_TOPIC = '/behavior_mode'
NAVIGATE_TO_POSE_ACTION = 'navigate_to_pose'

SUPERVISOR_RATE_HZ = 20.0


class BehaviorSupervisorNode(Node):

    def __init__(self):
        super().__init__('behavior_supervisor')

        self._supervisor = BehaviorSupervisor(
            clock=lambda: self.get_clock().now().nanoseconds / 1e9)
        self._latest_goal_msg = None
        self._goal_handle = None
        self._last_logged_mode = None

        self._nav_client = ActionClient(
            self, NavigateToPose, NAVIGATE_TO_POSE_ACTION)

        self._cmd_pub = self.create_publisher(Twist, CMD_VEL_RAW_TOPIC, 10)
        self._mode_pub = self.create_publisher(
            String, BEHAVIOR_MODE_TOPIC, 10)

        self.create_subscription(
            PoseStamped, GOAL_POSE_TOPIC, self._on_goal_pose, 10)
        self.create_subscription(
            String, LOCALIZATION_STATUS_TOPIC,
            self._on_localization_status, 10)
        self.create_subscription(
            String, CONTROL_AUTHORITY_TOPIC, self._on_control_authority, 10)
        self.create_subscription(
            Twist, CMD_VEL_NAV_TOPIC, self._on_cmd_vel_nav, 10)
        self.create_subscription(
            Twist, CMD_VEL_PROBE_TOPIC, self._on_cmd_vel_probe, 10)

        self.create_timer(1.0 / SUPERVISOR_RATE_HZ, self._tick)

    def _on_goal_pose(self, msg):
        self._latest_goal_msg = msg
        self._supervisor.on_goal_pose(msg)

    def _on_localization_status(self, msg):
        self._supervisor.on_localization_status(msg.data)

    def _on_control_authority(self, msg):
        self._supervisor.on_control_authority(msg.data)

    def _on_cmd_vel_nav(self, msg):
        self._supervisor.on_cmd_vel_nav(msg.linear.x, msg.angular.z)

    def _on_cmd_vel_probe(self, msg):
        self._supervisor.on_cmd_vel_probe(msg.linear.x, msg.angular.z)

    def _tick(self):
        mode, (linear, angular), action = self._supervisor.tick()

        if action == DISPATCH_GOAL:
            self._dispatch_goal()
        elif action == CANCEL_GOAL:
            self._cancel_goal()

        out = Twist()
        out.linear.x = linear
        out.angular.z = angular
        self._cmd_pub.publish(out)
        self._mode_pub.publish(String(data=mode))

        if mode != self._last_logged_mode:
            self._last_logged_mode = mode
            self.get_logger().info(f'behavior_mode -> {mode}')

    def _dispatch_goal(self):
        if self._latest_goal_msg is None:
            return
        if not self._nav_client.server_is_ready():
            self.get_logger().warn(
                'navigate_to_pose action server not ready; '
                'dropping goal dispatch')
            self._supervisor.on_goal_finished()
            return
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self._latest_goal_msg
        send_future = self._nav_client.send_goal_async(goal_msg)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('navigate_to_pose goal rejected')
            self._supervisor.on_goal_finished()
            return
        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        self._goal_handle = None
        self._supervisor.on_goal_finished()

    def _cancel_goal(self):
        # Fire-and-forget, matching localization_monitor's reinit-service
        # call - BehaviorSupervisor already dropped the goal synchronously
        # in tick(); this just asks Nav2 to stop acting on it.
        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None


def main():
    rclpy.init()
    node = BehaviorSupervisorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
