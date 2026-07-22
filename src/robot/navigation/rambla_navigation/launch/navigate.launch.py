import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

# controller_server's velocity output goes to /cmd_vel_nav only - never
# /cmd_vel or /cmd_vel_raw directly (M7 Context: Nav2 and the supervisor
# feed /cmd_vel_raw only; rambla_safety stays the sole final /cmd_vel
# publisher). Exposed as a module-level constant (not inlined in the Node()
# call below) so test/test_nav_config.py can assert it without executing
# or introspecting a live launch graph - the same "no sim, deterministic"
# approach the rest of M7's unit tests use.
CONTROLLER_CMD_VEL_TOPIC = '/cmd_vel_nav'
CONTROLLER_REMAPPINGS = [('cmd_vel', CONTROLLER_CMD_VEL_TOPIC)]

# Nav2 lifecycle nodes this file brings up, in the order given to
# lifecycle_manager_navigation below. Also exposed as a module-level
# constant for the same testability reason as CONTROLLER_REMAPPINGS.
NAV2_LIFECYCLE_NODE_NAMES = [
    'controller_server',
    'planner_server',
    'behavior_server',
    'bt_navigator',
]


def generate_launch_description():
    nav_pkg_share = get_package_share_directory('rambla_navigation')
    nav2_config = os.path.join(nav_pkg_share, 'config', 'nav2.yaml')

    # Only brings up Nav2's planning/control/behavior/BT nodes plus the
    # supervisor - does not include localize.launch.py, apartment_world.
    # launch.py, or spawn simulation, so this same file can be included
    # unchanged by a future real-hardware bringup launch instead of a sim
    # launch (same reasoning as ekf.launch.py/localize.launch.py). Assumes
    # localize (map_server + amcl) is already up - Nav2 is a pure TF/map
    # consumer here, never a transform publisher.
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': True}],
        remappings=CONTROLLER_REMAPPINGS,
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': True}],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': True}],
    )

    # No default_nav_to_pose_bt_xml override in nav2.yaml - runs
    # nav2_bt_navigator's packaged default BT
    # (navigate_to_pose_w_replanning_and_recovery.xml), which already wraps
    # the nav sequence in a RecoveryNode driving the spin/backup/wait
    # behavior_plugins declared above (M7 scope decision: Nav2's built-in
    # recovery, no custom BT). See test_nav_config.py, which parses the
    # actual installed default to prove this rather than assuming it.
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_config, {'use_sim_time': True}],
    )

    # nav2_costmap_2d nodes are declared inside controller_server/
    # planner_server's own processes via their costmap params (global_
    # costmap/local_costmap groups in nav2.yaml) - Nav2's standard
    # composition, no separate costmap node needed here.

    # Nav2's lifecycle nodes stay UNCONFIGURED (publishing nothing) without
    # something driving them through configure/activate - same pattern as
    # localize.launch.py's lifecycle_manager_localization. autostart:=true
    # does that automatically on launch.
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': NAV2_LIFECYCLE_NODE_NAMES,
        }],
    )

    # M7 Phase 2 (not yet implemented as of Phase 1): sole /cmd_vel_raw
    # writer, muxing NAV (/cmd_vel_nav)/PROBE (/cmd_vel_probe)/MAP intent
    # and gating goal dispatch on /localization_status == USABLE.
    # Referenced here now (the executable entry point already exists per
    # Phase 0 - console_scripts entry points resolve at invocation, not at
    # launch-description-build time) so this file doesn't need a second
    # edit once behavior_supervisor.py lands; it simply won't run
    # successfully until Phase 2 is committed.
    behavior_supervisor = Node(
        package='rambla_navigation',
        executable='behavior_supervisor',
        name='behavior_supervisor',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager_navigation,
        behavior_supervisor,
    ])
