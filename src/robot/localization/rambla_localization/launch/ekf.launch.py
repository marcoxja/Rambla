import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rambla_localization')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

    # Only launches ekf_node - does not include apartment_world.launch.py or
    # spawn simulation, so this same file can be included unchanged by a
    # future real-hardware bringup launch file alongside a hardware driver
    # bringup instead of a sim launch. frame_id_fixer is launched from
    # apartment_world.launch.py instead (not here): it now also covers
    # /scan and /camera/*, which are unrelated to the EKF and needed by
    # any sim consumer, not just this package - see frame_id_fixer.py.
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': True}],
    )

    return LaunchDescription([ekf_node])
