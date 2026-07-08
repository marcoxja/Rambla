from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Standalone, manually-launched (not included from apartment_world.
    # launch.py) - this is "run when you want a mapping session," not an
    # always-on part of the sim bringup, same pattern as
    # rambla_control_panel's standalone control_panel.launch.py.
    traversal_node = Node(
        package='rambla_traversal',
        executable='traversal_node',
        name='traversal_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([traversal_node])
