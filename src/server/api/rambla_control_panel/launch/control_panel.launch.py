from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Standalone on purpose - not included by apartment_world.launch.py.
    # This is a dev tool that should be startable/stoppable independent of
    # the sim (and, later, runnable unchanged against real hardware), run
    # alongside it:
    #   ros2 launch rambla_sim apartment_world.launch.py &
    #   ros2 launch rambla_control_panel control_panel.launch.py
    control_panel = Node(
        package='rambla_control_panel',
        executable='control_panel',
        name='control_panel_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([control_panel])
