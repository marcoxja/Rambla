import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    sim_pkg_share = get_package_share_directory('rambla_sim')
    description_pkg_share = get_package_share_directory('rambla_description')
    localization_pkg_share = get_package_share_directory('rambla_localization')
    safety_pkg_share = get_package_share_directory('rambla_safety')

    world_path = os.path.join(sim_pkg_share, 'worlds', 'apartment_world.sdf')
    xacro_path = os.path.join(description_pkg_share, 'urdf', 'rambla.urdf.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': f'-r -s {world_path}'}.items(),
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rambla',
            '-x', '2.0',
            '-y', '2.0',
            '-z', '0.05',
        ],
        output='screen',
    )

    # Topic name notes (each confirmed live via `gz topic -e -t <name>`
    # during verification, not assumed):
    # - DiffDrive/OdometryPublisher resolve by joint/model name, unaffected
    #   by link-lumping, and publish under their fully-qualified gz path.
    # - gpu_lidar/camera/imu sensors AND JointStatePublisher all publish
    #   under the PLAIN topic name given in their <topic> tag (e.g. `scan`,
    #   `joint_states` - not a fully-qualified `.../link/.../sensor/...` or
    #   `.../model/rambla/...` path), regardless of which link/joint
    #   they're attached to. See bugs/joint-states-topic-name-mismatch.md -
    #   the bridge previously assumed JointStatePublisher namespaced like
    #   DiffDrive/OdometryPublisher; it doesn't.
    # - The contact sensor (bumper) is now attached to its own dedicated
    #   bumper_left/bumper_right links (not base_link) - see bumper_half in
    #   rambla.urdf.xacro and bugs/bumper-contact-sensor-silent.md. Bridged
    #   below on the fully-qualified path matching that new attachment;
    #   verify the live path via `gz topic -l` after spawning and correct
    #   this if gz-sim's actual entity-tree naming differs.
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/model/rambla/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU',
            (
                '/world/apartment_world/model/rambla/link/bumper_left/sensor/'
                'bumper_left/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'
            ),
            (
                '/world/apartment_world/model/rambla/link/bumper_right/sensor/'
                'bumper_right/contact@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'
            ),
        ],
        remappings=[
            ('/model/rambla/odometry', '/odom'),
            (
                '/world/apartment_world/model/rambla/link/bumper_left/sensor/bumper_left/contact',
                '/bumper_left/contact',
            ),
            (
                '/world/apartment_world/model/rambla/link/bumper_right/sensor/bumper_right/contact',
                '/bumper_right/contact',
            ),
        ],
        output='screen',
    )

    # Rewrites frame_id/child_frame_id on /odom, /imu/data, /scan, and
    # /camera/* (gz-sim prepends the model name, e.g. "rambla/odom", which
    # doesn't match robot_state_publisher's plain URDF-derived TF tree), and
    # injects placeholder non-zero covariance on /odom and /imu/data so
    # ekf_node has a real per-sensor trust signal - see frame_id_fixer.py
    # and robot/pre-slam-audit.md. Sim-only shim: dropped (not ported) on
    # real hardware, whose driver nodes publish plain frame_ids and real
    # covariance directly.
    frame_id_fixer = Node(
        package='rambla_localization',
        executable='frame_id_fixer',
        name='frame_id_fixer',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Fuses /odom_fixed + /imu/data_fixed into /odometry/filtered and becomes
    # sole owner of the odom->base_footprint TF (see
    # rambla_localization/config/ekf.yaml and the <tf_topic> removal in
    # rambla_description/urdf/plugins.xacro).
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(localization_pkg_share, 'launch', 'ekf.launch.py')
        )
    )

    # Local collision-safety reflex (DESIGN_SPEC.md LOC-001/LOC-002/LOC-004,
    # SAF-001-003): overrides /cmd_vel on imminent collision using
    # /scan_fixed + bumper contact, no map/server dependency. Included here
    # (not left standalone like control_panel.launch.py) because it depends
    # on sim sensor topics existing, same ownership relationship as ekf.
    # Subscribes to /cmd_vel_raw (published by e.g. rambla_traversal) and
    # is the sole publisher of /cmd_vel for autonomous behaviors - see
    # rambla_safety/safety_node.py.
    safety = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(safety_pkg_share, 'launch', 'safety.launch.py')
        )
    )

    # gz_sim needs to be up before robot_state_publisher/spawn_robot/bridge/
    # frame_id_fixer/ekf/safety have anything to attach to or bridge from.
    # ROS2 launch's default parallel start has been fast enough so far that
    # this hasn't caused a failure, but nothing guarantees that on a slower
    # machine or heavier world - see robot/pre-slam-audit.md item 5. A short
    # TimerAction is the simplest guard consistent with this repo's other
    # launch files (none use the more involved RegisterEventHandler pattern).
    delayed_bringup = TimerAction(
        period=3.0,
        actions=[robot_state_publisher, spawn_robot, bridge, frame_id_fixer, ekf, safety],
    )

    return LaunchDescription([gz_sim, delayed_bringup])
