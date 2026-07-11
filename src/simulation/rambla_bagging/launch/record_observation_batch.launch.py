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

    # Topic list and --storage mcap match observation-batch.md: raw /scan
    # and /camera/* (plugins.xacro's <gz_frame_id>/<optical_frame_id> tags
    # already give these plain frame_ids at the source, so no _fixed
    # republish is needed for them anymore - Task 1 of the OOMWOO/Kaia
    # foundation decision), /imu/data_fixed (still the covariance_injector's
    # republish - carries non-zero covariance the raw topic lacks),
    # /odometry/filtered (the EKF's stable pose contract, not raw odom),
    # /clock (required since the Modal SLAM job runs with
    # use_sim_time:=true and needs the bag's own simulated time base), and
    # /tf + /tf_static (added after the first real M3 run: rtabmap_launch
    # looks up odom->base_footprint and the static sensor-frame transforms
    # via tf2 even with odom_topic set, not just the odometry message
    # itself - without these two topics every transform lookup fails during
    # replay and rtabmap writes an empty database. See observation-batch.md.
    #
    # /camera/image_raw/compressed (not raw /camera/image_raw) is what's
    # actually recorded: raw camera frames were the dominant contributor to
    # a ~2.9-3.1GB/200s batch (see observation-batch.md and
    # .claude/internal-docs/compute/slam/CLAUDE.md). The JPEG side channel
    # is published continuously by camera_compressor in
    # apartment_world.launch.py - the canonical live /camera/image_raw feed
    # (control panel, RViz, smoke tests) is untouched, this just changes
    # which of the two already-available streams gets bagged.
    # --compression-mode file --compression-format zstd is a second,
    # independent, lossless lever on top of that: MCAP compresses each
    # topic's messages (odometry/tf/imu/scan as well as the now-JPEG camera
    # frames) transparently to any MCAP reader (ros2 bag play,
    # rosbag2_py.SequentialReader), no replay-side changes needed.
    record_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record',
            '-o', LaunchConfiguration('observation_batch_dir'),
            '--storage', 'mcap',
            '--compression-mode', 'file',
            '--compression-format', 'zstd',
            '/clock',
            '/scan',
            '/camera/image_raw/compressed',
            '/camera/camera_info',
            '/imu/data_fixed',
            '/odometry/filtered',
            '/tf',
            '/tf_static',
        ],
        output='screen',
    )

    return LaunchDescription([observation_batch_dir_arg, record_process])
