import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('rambla_safety')
    safety_config = os.path.join(pkg_share, 'config', 'safety.yaml')

    # safety.yaml's thresholds are loaded through ROS's standard
    # parameters=[...] file mechanism (same pattern as ekf.launch.py's
    # ekf_config) and read via declare_parameter/get_parameter in
    # safety_node.py.
    safety_node = Node(
        package='rambla_safety',
        executable='safety_node',
        name='safety_node',
        output='screen',
        parameters=[safety_config, {'use_sim_time': True}],
    )

    return LaunchDescription([safety_node])
