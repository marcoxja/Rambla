import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Per-world spawn defaults, used whenever the spawn_x/y/z launch args are left
# at their default (empty string). Keyed by the `world` launch-arg value,
# which is also the .sdf filename stem and the <world name> inside that file
# (see the invariant note in launch_setup below).
WORLD_SPAWN_DEFAULTS = {
    # Living room, near the original 3-room layout's entrance.
    'apartment_world': (2.0, 2.0, 0.05),
    # Living room open floor, clear of the sofa/coffee-table/TV-stand
    # (upstream house.sdf itself documents this as the robot spawn area:
    # x=-6 to -2, y=1.5 to 4.5).
    'house': (-4.0, 3.0, 0.05),
}


def launch_setup(context, *args, **kwargs):
    sim_pkg_share = get_package_share_directory('rambla_sim')
    description_pkg_share = get_package_share_directory('rambla_description')
    localization_pkg_share = get_package_share_directory('rambla_localization')
    safety_pkg_share = get_package_share_directory('rambla_safety')

    # Invariant relied on below and by the bridge's bumper contact topic
    # paths: the `world` launch-arg value, the worlds/<world>.sdf filename
    # stem, and the file's <world name="..."> must all match. Keep any new
    # world file's <world name> equal to its filename stem.
    world_name = LaunchConfiguration('world').perform(context)
    world_path = os.path.join(sim_pkg_share, 'worlds', f'{world_name}.sdf')
    xacro_path = os.path.join(description_pkg_share, 'urdf', 'rambla.urdf.xacro')
    robot_description = xacro.process_file(xacro_path).toxml()

    default_x, default_y, default_z = WORLD_SPAWN_DEFAULTS.get(
        world_name, (2.0, 2.0, 0.05)
    )
    spawn_x = LaunchConfiguration('spawn_x').perform(context) or str(default_x)
    spawn_y = LaunchConfiguration('spawn_y').perform(context) or str(default_y)
    spawn_z = LaunchConfiguration('spawn_z').perform(context) or str(default_z)

    # Headless (-s, server-only) by default - required over a plain SSH
    # session with no DISPLAY. Pass gui:=true only when a real X server is
    # reachable (e.g. DISPLAY=:0 pointed at the VM's own console via
    # startx - see simulation/CLAUDE.md) to watch the sim visually.
    gui = LaunchConfiguration('gui').perform(context)
    server_flag = '' if gui == 'true' else '-s '

    # gz-sim's model:// URI resolver (SystemPaths) only searches
    # GZ_SIM_RESOURCE_PATH, which out of the box covers /opt/ros/jazzy/share
    # but not this workspace's own install space - so package://rambla_
    # description/... mesh URIs (introduced by the M1 bumper mesh fix; the
    # first mesh this repo has used) fail to resolve ("Unable to find file
    # with URI [model://rambla_description/...]") even though the .stl is
    # correctly installed and package:// resolves fine for the URDF itself.
    # Appending rambla_description's share PARENT dir (not the package's own
    # share/rambla_description subdir) lets gz-sim's model://<pkg>/<path>
    # search land on share/<pkg>/<path>, matching how CMakeLists.txt installs
    # meshes/. Must be set before gz_sim starts.
    gz_resource_path = AppendEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        os.path.dirname(description_pkg_share),
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={'gz_args': f'-r {server_flag}{world_path}'}.items(),
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
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', spawn_z,
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
    bumper_left_topic = (
        f'/world/{world_name}/model/rambla/link/bumper_left/sensor/'
        'bumper_left/contact'
    )
    bumper_right_topic = (
        f'/world/{world_name}/model/rambla/link/bumper_right/sensor/'
        'bumper_right/contact'
    )

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
            f'{bumper_left_topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
            f'{bumper_right_topic}@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
        ],
        remappings=[
            ('/model/rambla/odometry', '/odom'),
            (bumper_left_topic, '/bumper_left/contact'),
            (bumper_right_topic, '/bumper_right/contact'),
        ],
        output='screen',
    )

    # Injects placeholder non-zero covariance onto /odom and /imu/data so
    # ekf_node has a real per-sensor trust signal (gz-sim's OdometryPublisher
    # and IMU sensor systems publish all-zero covariance) - see
    # covariance_injector.py and robot/pre-slam-audit.md. Also rewrites
    # /odom's frame_id/child_frame_id to the plain names, since the
    # OdometryPublisher plugin can't take <gz_frame_id> (sensor-level only,
    # unlike the LiDAR/IMU sensors). /scan and /camera/* no longer need any
    # republish here: plugins.xacro's <gz_frame_id>/<optical_frame_id> tags
    # set plain frame_ids on those topics at the source. Sim-only shim:
    # dropped (not ported) on real hardware, whose driver nodes publish
    # plain frame_ids and real covariance directly.
    covariance_injector = Node(
        package='rambla_localization',
        executable='covariance_injector',
        name='covariance_injector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Always-on JPEG side channel for /camera/image_raw, published via the
    # standard image_transport republish mechanism rather than anything
    # bespoke to this repo. /camera/image_raw itself (raw, 15Hz) is
    # untouched - this is a second, additional publisher any consumer can
    # opt into for lower bandwidth (e.g. the observation-batch recorder;
    # see record_observation_batch.launch.py), not a replacement. jpeg
    # quality is a launch arg so it can be tuned during M3's mapping-batch
    # validation without a code change; see observation-batch.md for the
    # rationale and chosen default.
    camera_compressor = Node(
        package='image_transport',
        executable='republish',
        name='camera_compressor',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', '/camera/image_raw'),
            ('out/compressed', '/camera/image_raw/compressed'),
        ],
        parameters=[{
            'use_sim_time': True,
            'compressed.jpeg_quality': ParameterValue(
                LaunchConfiguration('camera_jpeg_quality'), value_type=int
            ),
        }],
        output='screen',
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
    # SAF-001-003): overrides /cmd_vel on imminent collision using raw
    # /scan + bumper contact, no map/server dependency. Included here
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

    # gz_sim needs to be up before anything else attaches to or bridges from
    # it. Rather than a fixed TimerAction guess, bringup is gated on two
    # real readiness signals in sequence:
    #   1. robot_state_publisher's process has started (so its latched
    #      robot_description topic exists for spawn_robot to read).
    #   2. spawn_robot (ros_gz_sim create) has exited successfully (so the
    #      entity actually exists in the running gz world before bridge/
    #      covariance_injector/ekf/safety attach to gz-side topics/TF).
    # This event-driven approach removes the race entirely instead of
    # hoping a fixed delay is long enough on a slower machine or heavier
    # world - see robot/pre-slam-audit.md item 5.

    # Gate 1: spawn the robot only once robot_state_publisher's process has
    # started, so `create -topic robot_description` reliably finds RSP's
    # transient-local (latched) robot_description publisher rather than racing
    # its creation.
    spawn_after_rsp = RegisterEventHandler(
        OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[spawn_robot],
        )
    )

    # Gate 2: bring up the bridge and all downstream ROS nodes only after the
    # robot has actually spawned. `ros_gz_sim create` blocks until the gz
    # server is up, spawns the entity, then exits 0 - so its exit is the real
    # readiness signal (this ros_gz_sim build has no `-timeout` flag to lean
    # on). On a nonzero exit the spawn failed, so we fail visibly by shutting
    # the whole bringup down rather than starting nodes that would silently
    # hang forever on topics that never arrive. The bridge waits for full
    # spawn (not just server-up) as a deliberate simplicity choice: it only
    # needs the gz server, which is guaranteed up by the time create exits,
    # and splitting it onto a separate gate would need another readiness
    # signal for negligible benefit.
    def _bringup_on_spawn_exit(event, context):
        if event.returncode == 0:
            return [bridge, covariance_injector, camera_compressor, ekf, safety]
        return [
            LogInfo(
                msg=(
                    'robot spawn (ros_gz_sim create) exited with code '
                    f'{event.returncode}; aborting bringup.'
                )
            ),
            Shutdown(reason='robot spawn failed'),
        ]

    bringup_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=spawn_robot,
            on_exit=_bringup_on_spawn_exit,
        )
    )

    return [
        gz_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_after_rsp,
        bringup_after_spawn,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'world',
            default_value='apartment_world',
            description=(
                "World to load, by worlds/<world>.sdf filename stem "
                "(must equal that file's <world name>): 'apartment_world' "
                "(default, hand-authored 3-room layout) or 'house' "
                '(adopted multi-room apartment, see house.sdf header).'
            ),
        ),
        DeclareLaunchArgument(
            'spawn_x', default_value='',
            description='Robot spawn X (m). Empty = per-world default.',
        ),
        DeclareLaunchArgument(
            'spawn_y', default_value='',
            description='Robot spawn Y (m). Empty = per-world default.',
        ),
        DeclareLaunchArgument(
            'spawn_z', default_value='',
            description='Robot spawn Z (m). Empty = per-world default.',
        ),
        DeclareLaunchArgument(
            'gui', default_value='false',
            description=(
                'true = run gz sim with its GUI attached (needs a real '
                'DISPLAY, e.g. the VM console via startx); false (default) '
                '= headless server-only, required over plain SSH.'
            ),
        ),
        DeclareLaunchArgument(
            'camera_jpeg_quality', default_value='85',
            description=(
                'JPEG quality (0-100) for the always-on /camera/image_raw/'
                'compressed side channel published by camera_compressor. '
                'Does not affect the raw /camera/image_raw feed.'
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
