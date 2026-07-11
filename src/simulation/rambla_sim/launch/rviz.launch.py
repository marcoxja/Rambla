import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg_share = get_package_share_directory('rambla_sim')
    rviz_config = os.path.join(sim_pkg_share, 'rviz', 'rambla.rviz')

    # Standalone - run alongside apartment_world.launch.py from a second
    # (X-forwarded) session, not included by it. RViz is a GUI tool the
    # developer starts/stops independently of the sim; keeping it separate
    # is what makes it fit the SSH-based Remote-SSH workflow (see
    # simulation/CLAUDE.md) instead of requiring VM console access.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    return LaunchDescription([rviz])
