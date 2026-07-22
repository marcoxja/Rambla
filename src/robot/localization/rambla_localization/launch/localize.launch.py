import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rambla_localization')
    amcl_config = os.path.join(pkg_share, 'config', 'amcl.yaml')

    # Fixed local cache path (M5 Phase 2) - this launch file has no Modal/
    # batch/version awareness. rambla_map_activate (scripts/vm.sh) is the
    # only thing that ever changes what `active` points at.
    default_map_path = os.path.expanduser('~/ros2_ws/maps/active/map.yaml')
    map_yaml = LaunchConfiguration('map')

    # M7 Phase 1: additive, default-preserving - default '/cmd_vel_raw'
    # keeps a localize-only run (no nav bringup) behaving exactly as it did
    # before M7. apartment_world.launch.py remaps this to '/cmd_vel_probe'
    # only when navigate:=true, so behavior_supervisor (the M7 nav
    # supervisor) becomes the sole /cmd_vel_raw writer instead.
    probe_cmd_topic = LaunchConfiguration('probe_cmd_topic')

    # Only launches map_server + amcl (+ their lifecycle manager) - does not
    # include apartment_world.launch.py or spawn simulation, so this same
    # file can be included unchanged by a future real-hardware bringup
    # launch file instead of a sim launch, same reasoning as ekf.launch.py.
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True}],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_config, {'use_sim_time': True}],
    )

    # map_server and amcl are both nav2 lifecycle nodes - they stay
    # UNCONFIGURED (publishing nothing) without something driving them
    # through configure/activate. autostart:=true does that automatically on
    # launch, no operator action required.
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }],
    )

    # M5 Phase 4: confidence state machine + recovery trigger - AMCL's own
    # recovery_alpha_* (amcl.yaml) is a passive mitigation only, not a
    # guaranteed kidnapped-robot detector. Not a lifecycle node itself; it
    # polls /reinitialize_global_localization's own availability to detect
    # amcl activation instead of joining the lifecycle_manager above.
    localization_monitor = Node(
        package='rambla_localization',
        executable='localization_monitor',
        name='localization_monitor',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # M5 Phase 5: cautious disambiguation - a slow, bounded in-place
    # rotation while stationary and GLOBAL/CONVERGING, so AMCL has motion
    # to accumulate distinguishing bearings from. Not a lifecycle node;
    # publishes to /cmd_vel_raw like rambla_traversal, arbitrated
    # unchanged by rambla_safety (INT-005).
    localization_probe = Node(
        package='rambla_localization',
        executable='localization_probe',
        name='localization_probe',
        output='screen',
        parameters=[{'use_sim_time': True, 'cmd_vel_topic': probe_cmd_topic}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=default_map_path,
            description=(
                'Path to a map.yaml. Defaults to the Phase 2 fixed local '
                'map cache path activated by scripts/vm.sh:rambla_map_'
                'activate - not parameterized by Modal batch/version at '
                'this level.'
            ),
        ),
        DeclareLaunchArgument(
            'probe_cmd_topic', default_value='/cmd_vel_raw',
            description=(
                "localization_probe's output topic. Default '/cmd_vel_raw' "
                'preserves today\'s localize-only behavior; M7 nav bringup '
                "passes '/cmd_vel_probe' so behavior_supervisor owns "
                '/cmd_vel_raw instead.'
            ),
        ),
        map_server,
        amcl,
        lifecycle_manager,
        localization_monitor,
        localization_probe,
    ])
