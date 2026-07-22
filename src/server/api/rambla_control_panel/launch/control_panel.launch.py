from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Standalone on purpose - not included by apartment_world.launch.py.
    # This is a dev tool that should be startable/stoppable independent of
    # the sim (and, later, runnable unchanged against real hardware), run
    # alongside it:
    #   ros2 launch rambla_sim apartment_world.launch.py &
    #   wrangler dev  # in src/server/api/rambla_relay, Phase 1 local relay
    #   ros2 launch rambla_control_panel control_panel.launch.py
    #
    # Defaults match rambla_relay/.dev.vars.example so the local
    # wrangler-dev loop works with no overrides; override robot_token if
    # your local .dev.vars differs, and relay_url when pointing at a
    # deployed Worker.
    relay_url_arg = DeclareLaunchArgument(
        'relay_url', default_value='ws://localhost:8787',
        description='Base wss:// URL of the relay (Cloudflare Worker / local wrangler dev).')
    robot_id_arg = DeclareLaunchArgument(
        'robot_id', default_value='robot-1',
        description='Robot id used for relay routing (relay-protocol.md {robot_id}).')
    robot_token_arg = DeclareLaunchArgument(
        'robot_token', default_value='dev-local-robot-token',
        description='Bearer token presented to the relay (rambla_relay ROBOT_TOKEN secret).')
    stats_file_arg = DeclareLaunchArgument(
        'stats_file', default_value='/tmp/rambla_gateway_stats.json',
        description='Path the gateway writes per-channel bandwidth counters to '
                     '(read by scripts/sample_resources.py).')

    control_panel = Node(
        package='rambla_control_panel',
        executable='control_panel',
        name='control_panel_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        relay_url_arg,
        robot_id_arg,
        robot_token_arg,
        stats_file_arg,
        SetEnvironmentVariable('RAMBLA_RELAY_URL', LaunchConfiguration('relay_url')),
        SetEnvironmentVariable('RAMBLA_ROBOT_ID', LaunchConfiguration('robot_id')),
        SetEnvironmentVariable('RAMBLA_ROBOT_TOKEN', LaunchConfiguration('robot_token')),
        SetEnvironmentVariable('RAMBLA_GATEWAY_STATS_FILE', LaunchConfiguration('stats_file')),
        control_panel,
    ])
