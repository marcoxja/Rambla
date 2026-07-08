import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Default output dir timestamped so repeat recording sessions don't
    # collide - can be overridden via the observation_batch_dir launch arg,
    # e.g.: ros2 launch rambla_bagging record_observation_batch.launch.py
    #         observation_batch_dir:=./observation_batch_001
    default_dir = os.path.join(
        os.path.expanduser('~'),
        'rambla_bags',
        datetime.now().strftime('observation_batch_%Y%m%d_%H%M%S'))

    observation_batch_dir_arg = DeclareLaunchArgument(
        'observation_batch_dir',
        default_value=default_dir,
        description='Output directory for the recorded rosbag2/MCAP batch.',
    )

    # Topic list and --storage mcap match
    # .claude/internal-docs/robot/slam/plans/real-map-plan-2026-07-06.md
    # Phase 1 exactly: the _fixed frame-corrected topics (not raw /scan,
    # /odom, /imu/data, /camera/*), /odometry/filtered (the EKF's stable
    # pose contract, not raw odom), and /clock (required since the Modal
    # SLAM job runs with use_sim_time:=true and needs the bag's own
    # simulated time base).
    record_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', LaunchConfiguration('observation_batch_dir'),
            '--storage', 'mcap',
            '/clock',
            '/scan_fixed',
            '/camera/image_raw_fixed',
            '/camera/camera_info_fixed',
            '/imu/data_fixed',
            '/odometry/filtered',
        ],
        output='screen',
    )

    return LaunchDescription([observation_batch_dir_arg, record_process])
