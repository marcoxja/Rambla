import json
import os
import re
import shutil
import signal
import sqlite3
import subprocess
import time

import modal

image = modal.Image.from_dockerfile("Dockerfile", add_python="3.12")
app = modal.App("rambla-slam-smoke-test", image=image)

# Separate layer, only for the frame-feature diagnostic below - keeps
# opencv/numpy out of run_mapping_job's image entirely.
diagnostic_image = image.pip_install("opencv-python-headless", "numpy", "pyyaml")

data_volume = modal.Volume.from_name("rambla-slam-data", create_if_missing=True)


@app.function()
def smoke_test() -> str:
    result = subprocess.run(
        ["bash", "-c", "source /opt/ros/jazzy/setup.bash && ros2 pkg list"],
        capture_output=True,
        text=True,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"ros2 pkg list failed (exit {result.returncode}):\n{result.stderr}"
        )

    packages = result.stdout.splitlines()
    if "rtabmap_ros" not in packages:
        raise RuntimeError(
            f"rtabmap_ros not found in ros2 pkg list. Packages seen:\n{result.stdout}"
        )

    return "OK: rtabmap_ros found and ros2 pkg list ran cleanly."


# rtabmap_ros's node is a live ROS2 node, not a batch-file processor - the
# sanctioned approach (introlab/rtabmap_ros#1286) is to feed it via
# `ros2 bag play` into the live node graph, then SIGINT it once playback
# ends so it flushes rtabmap.db cleanly on rclcpp shutdown. Args confirmed
# against rtabmap_launch/launch/rtabmap.launch.py source, not guessed:
# frame_id matches ekf.yaml's base_link_frame; visual/icp_odometry:=false
# means rtabmap consumes /odometry/filtered directly rather than computing
# competing odometry of its own (EKF owns odometry, RTAB-Map adds
# loop-closure/graph-SLAM on top).
# Loop-closure verification parameters, confirmed via the fast
# rerun_reprocess harness against observation_batch_002 before being applied
# here (see .claude/internal-docs/compute/slam/CLAUDE.md's loop-closure
# follow-up for the full trace). Root cause: this rig is monocular + LiDAR,
# depth:=false, so Signature::getWords3() (3D visual words) is always empty
# and vision-based loop-closure verification (Reg/Strategy's default, 0=Vis)
# can never pass - confirmed from RegistrationVis.cpp's "Not enough features
# in images (old=%d, new=%d)" using getWords3()/getWords() respectively, and
# from direct sqlite3 queries on rtabmap.db showing 0 of ~30k stored features
# have non-null depth_x/y/z across all 221 nodes.
#   Reg/Strategy=1               - verify loop closures via LiDAR ICP instead
#                                   of vision (the camera can't supply it).
#   Reg/Force3DoF=true            - constrain to x/y/yaw (flat-floor robot).
#   RGBD/LoopClosureIdentityGuess - without this, Memory::computeTransform
#     =true                        still falls back to _registrationVis to
#                                   produce an initial guess for ICP whenever
#                                   no odometry-chain guess exists (true for
#                                   every global/non-adjacent loop-closure
#                                   candidate) - hitting the exact same
#                                   getWords3()==0 wall. Setting this feeds
#                                   ICP an identity guess directly instead.
#   Icp/MaxCorrespondenceDistance - default 0.1m is tuned for consecutive-
#     =1.0                         frame ICP odometry, too tight for a cold
#                                   identity guess against real drift.
#   Icp/MaxRotation=1.5,           default bounds (0.78 rad / 0.2m) are also
#   Icp/MaxTranslation=2.0         tuned for small consecutive corrections;
#                                   loosened to fit observed drift magnitudes
#                                   (up to ~1.8m/~45 deg) without going fully
#                                   unbounded, which let wrong ICP matches
#                                   through for RGBD/OptimizeMaxError (graph
#                                   consistency, left at its default 3.0) to
#                                   then correctly catch and revert.
#
# rgb_topic stays plain /camera/image_raw, no compressed:=true here - see
# _decompress_rgb_cmd below for why. This is otherwise the same invocation
# validated in the original M3 run.
def _rtabmap_launch_cmd(database_path: str) -> str:
    return (
        "source /opt/ros/jazzy/setup.bash && "
        "ros2 launch rtabmap_launch rtabmap.launch.py "
        'rtabmap_args:="--delete_db_on_start '
        "--Reg/Strategy 1 "
        "--Reg/Force3DoF true "
        "--RGBD/LoopClosureIdentityGuess true "
        "--Icp/MaxCorrespondenceDistance 1.0 "
        "--Icp/MaxRotation 1.5 "
        '--Icp/MaxTranslation 2.0" '
        "depth:=false "
        "subscribe_rgb:=true "
        "subscribe_scan:=true "
        "visual_odometry:=false "
        "icp_odometry:=false "
        "frame_id:=base_footprint "
        "odom_frame_id:=odom "
        "odom_topic:=/odometry/filtered "
        "rgb_topic:=/camera/image_raw "
        "camera_info_topic:=/camera/camera_info "
        "scan_topic:=/scan "
        "approx_sync:=true "
        "qos:=1 "
        f"database_path:={database_path} "
        "rtabmap_viz:=false "
        "rviz:=false "
        "use_sim_time:=true"
    )


# Decompresses the batch's /camera/image_raw/compressed (JPEG - see
# record_observation_batch.launch.py and observation-batch.md, raw camera
# frames were the dominant contributor to batch size) back to plain raw
# /camera/image_raw, so rtabmap_launch's rgb_topic subscription above needs
# no changes from the original, already-validated M3 invocation.
#
# Tried rtabmap_launch's own built-in compressed:=true/rgb_image_transport
# support first, then a hand-rolled `image_transport republish` invocation
# using its documented-looking positional arguments (`republish <in>
# <out>`) - both failed identically ("Did not receive data since 5
# seconds!", 0 MB db). Root-caused by reading republish.cpp directly
# (ros-perception/image_common, image_transport/src/republish.cpp,
# Republisher::initialize()): the node reads `in_transport`/`out_transport`
# **only from declared ROS parameters**, never from positional CLI
# arguments - `arguments=[...]` (what both rtabmap_launch's internal
# republish_rgb node AND the first hand-rolled attempt here used) is
# silently ignored, always falling back to the "raw" default and
# subscribing to a topic nothing publishes. This is a real bug/quirk in
# rtabmap_launch's own usage of the node, not something specific to this
# repo. -p in_transport:=compressed below is what actually selects the
# transport.
#
# Also lazy: when out_transport is left unset (the "advertise every
# transport" branch), the input subscription is only created once
# something subscribes to the output topic (image_transport::Publisher's
# matched_callback) - fine here since rtabmap_launch's own rgb/image
# subscriber on /camera/image_raw is exactly that trigger, and the 8s
# warm-up sleep before bag play starts gives discovery/matching plenty of
# time to complete first.
#
# The "in" remap must target the full combined name `in/<transport>`
# (`in/compressed`), not the bare `in` topic - confirmed live via
# `ros2 node info /image_republisher`: a bare `-r in:=/camera/image_raw`
# remap does NOT propagate to the `/compressed`-suffixed subscription
# image_transport actually constructs (it stayed on the literal
# unremapped `/in/compressed`, confirmed via node info showing that exact
# topic name). Asymmetric with the "out" side, where remapping the bare
# `out` topic works fine (out's raw variant has no suffix to begin with).
def _decompress_rgb_cmd() -> str:
    return (
        "source /opt/ros/jazzy/setup.bash && "
        "ros2 run image_transport republish "
        "--ros-args "
        "-p in_transport:=compressed "
        "-r in/compressed:=/camera/image_raw/compressed "
        "-r out:=/camera/image_raw "
        "-p use_sim_time:=true"
    )


_LINK_TYPE_NAMES = {
    0: "neighbor",
    1: "global_closure",
    2: "local_space_closure",
    3: "local_time_closure",
    4: "user_closure",
    5: "virtual_closure",
    6: "neighbor_merged",
    7: "pose_prior",
    8: "landmark",
}


def _link_type_counts(db_path: str) -> dict:
    # Ground truth for accepted loop closures - query the Link table
    # directly rather than trust ROS console log text. rtabmap's WARN/INFO
    # phrasing is inconsistent enough (e.g. "Rejecting all added loop
    # closures" reads as "accepted" under a naive "loop closure" +
    # not-"rejected" substring filter, since it doesn't say "rejected") that
    # line-based classification produced false positives - see the
    # loop-closure follow-up in .claude/internal-docs/compute/slam/CLAUDE.md.
    # type=1 (kGlobalClosure) and type=2 (kLocalSpaceClosure, i.e. proximity)
    # are the two accepted-loop-closure kinds the mono+LiDAR ICP fix targets.
    conn = sqlite3.connect(db_path)
    try:
        rows = conn.execute("SELECT type, COUNT(*) FROM Link GROUP BY type").fetchall()
    finally:
        conn.close()
    return {_LINK_TYPE_NAMES.get(t, f"type_{t}"): c for t, c in rows}


@app.function(volumes={"/data": data_volume}, timeout=900)
def run_mapping_job(batch_name: str) -> dict:
    bag_path = f"/data/observation_batches/{batch_name}"
    output_dir = f"/data/outputs/{batch_name}/map_v001"
    capture_db = f"/data/outputs/{batch_name}/rtabmap.db"
    os.makedirs(output_dir, exist_ok=True)

    # Started before rtabmap_launch so its subscriber to
    # /camera/image_raw/compressed is already up by the time bag play
    # starts publishing - no persistent state of its own, so a plain
    # terminate (no SIGINT-flush handshake) is enough at the end.
    decompress_proc = subprocess.Popen(
        ["bash", "-c", _decompress_rgb_cmd()],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    # Own process group (start_new_session=True) so the eventual SIGINT can
    # be sent to the whole group, not just the `ros2 launch` process -
    # covers any node subprocesses it spawns directly, not relying solely
    # on ros2 launch's own signal relay.
    rtabmap_proc = subprocess.Popen(
        ["bash", "-c", _rtabmap_launch_cmd(capture_db)],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )

    # Let the rtabmap node graph finish coming up before bag play starts -
    # a cold container's first `ros2 launch` is slower than a warm one.
    time.sleep(8)

    play_start = time.monotonic()
    play_result = subprocess.run(
        ["bash", "-c",
         f"source /opt/ros/jazzy/setup.bash && "
         f"ros2 bag play --rate 2 {bag_path}"],
        capture_output=True,
        text=True,
    )
    play_duration_s = time.monotonic() - play_start

    # rtabmap may still be draining its subscriber queue / running a final
    # optimization pass after playback ends - not an arbitrary guess, the
    # RTAB-Map maintainer's documented orchestration for this exact
    # live-node-in-a-script pattern.
    time.sleep(10)

    os.killpg(os.getpgid(rtabmap_proc.pid), signal.SIGINT)
    try:
        rtabmap_stdout, _ = rtabmap_proc.communicate(timeout=120)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(rtabmap_proc.pid), signal.SIGKILL)
        rtabmap_stdout, _ = rtabmap_proc.communicate()
    rtabmap_exit_code = rtabmap_proc.returncode

    os.killpg(os.getpgid(decompress_proc.pid), signal.SIGTERM)
    try:
        decompress_stdout, _ = decompress_proc.communicate(timeout=30)
    except subprocess.TimeoutExpired:
        os.killpg(os.getpgid(decompress_proc.pid), signal.SIGKILL)
        decompress_stdout, _ = decompress_proc.communicate()
    with open(os.path.join(output_dir, "decompress_stdout_full.log"), "w") as f:
        f.write(decompress_stdout)

    # Decouples "did live capture work" from "is the final map good" -
    # rtabmap-reprocess -g2 re-derives the graph/occupancy grid from the
    # captured db and is what actually produces map_map.pgm.
    reprocess_db = os.path.join(output_dir, "map.db")
    reprocess_result = subprocess.run(
        ["bash", "-c",
         f"source /opt/ros/jazzy/setup.bash && "
         f"rtabmap-reprocess -g2 {capture_db} {reprocess_db}"],
        capture_output=True,
        text=True,
    )

    # rtabmap's own per-iteration log line looks like
    # "rtabmap (221): Rate=... (local map=198, WM=198)" - node_count is the
    # final Working Memory size, not a literal "N nodes" string.
    node_count_matches = re.findall(r"WM=(\d+)", rtabmap_stdout)
    accepted_loop_closure_lines = [
        line for line in rtabmap_stdout.splitlines()
        if "loop closure" in line.lower() and "rejected" not in line.lower()
    ]
    rejected_loop_closure_lines = [
        line for line in rtabmap_stdout.splitlines()
        if "loop closure" in line.lower() and "rejected" in line.lower()
    ]

    # Ground truth from the db itself - see _link_type_counts. rtabmap.exit_code
    # being 0 only reflects `ros2 launch`'s own supervisor exit, not whether the
    # SIGINT-triggered db save completed cleanly, so this also doubles as
    # confirmation the capture_db is intact and readable.
    link_type_counts = _link_type_counts(capture_db)

    # Full stdout saved to the volume (not inlined in the JSON report,
    # `stdout_tail` below is truncated to the last 4000 chars) - same
    # reasoning as rerun_reprocess's full_log_path: lets early-launch
    # messages (node startup, transport/subscription confirmations) be
    # pulled and grepped locally without a Modal rerun.
    full_log_path = os.path.join(output_dir, "rtabmap_stdout_full.log")
    with open(full_log_path, "w") as f:
        f.write(rtabmap_stdout)

    report = {
        "batch_name": batch_name,
        "bag_play": {
            "exit_code": play_result.returncode,
            "duration_s": play_duration_s,
            "stderr_tail": play_result.stderr[-2000:],
        },
        "rtabmap": {
            "exit_code": rtabmap_exit_code,
            "node_count": (
                int(node_count_matches[-1]) if node_count_matches else None
            ),
            "link_type_counts": link_type_counts,
            "accepted_global_loop_closures": link_type_counts.get("global_closure", 0),
            "accepted_proximity_closures": link_type_counts.get("local_space_closure", 0),
            # Line-count fields below are approximate (naive stdout substring
            # matching) - kept for quick eyeballing/debugging only. Trust
            # link_type_counts above for the real accepted/rejected story.
            "accepted_loop_closure_count_approx": len(accepted_loop_closure_lines),
            "accepted_loop_closure_lines": accepted_loop_closure_lines[:20],
            "rejected_loop_closure_count_approx": len(rejected_loop_closure_lines),
            "rejected_loop_closure_lines": rejected_loop_closure_lines[:20],
            "stdout_tail": rtabmap_stdout[-4000:],
            "full_log_path": full_log_path,
        },
        "reprocess": {
            "exit_code": reprocess_result.returncode,
            "stderr_tail": reprocess_result.stderr[-2000:],
        },
    }

    with open(os.path.join(output_dir, "processing_report.json"), "w") as f:
        json.dump(report, f, indent=2)

    shutil.copy(capture_db, os.path.join(output_dir, "rtabmap.db"))

    data_volume.commit()

    return report


# Fast A/B harness for RTAB-Map parameter changes: rtabmap-reprocess replays
# an already-captured db through the full pipeline (feature use + loop
# closure detection AND verification - confirmed via tools/Reprocess/main.cpp
# calling rtabmap.process() per node, not just re-optimizing existing poses)
# and accepts `--Param/Name value` overrides (Parameters::parseArguments).
# That means a parameter change can be tested against the db already on the
# Volume with no bag replay and no live `ros2 launch` - see
# .claude/internal-docs/compute/slam/CLAUDE.md's loop-closure follow-up for
# why old=0 rejections happen (mono camera -> Signature::getWords3() is
# always empty -> visual loop-closure verification can never pass) and why
# Reg/Strategy=1 (LiDAR ICP verification instead of vision) is the fix.
@app.function(volumes={"/data": data_volume}, timeout=300)
def rerun_reprocess(batch_name: str, extra_rtabmap_args: str = "") -> dict:
    capture_db = f"/data/outputs/{batch_name}/rtabmap.db"
    out_dir = f"/data/outputs/{batch_name}/diagnostics/reprocess_{int(time.time())}"
    os.makedirs(out_dir, exist_ok=True)
    output_db = os.path.join(out_dir, "map.db")

    cmd = (
        "source /opt/ros/jazzy/setup.bash && "
        f"rtabmap-reprocess -g2 {extra_rtabmap_args} {capture_db} {output_db}"
    )
    result = subprocess.run(["bash", "-c", cmd], capture_output=True, text=True)

    # "Total loop closures = ..." is reprocess's final summary line, not a
    # per-candidate result - explicitly excluded so it can't masquerade as
    # an accepted candidate in the count below.
    accepted_loop_closure_lines = [
        line for line in result.stdout.splitlines()
        if "loop closure" in line.lower()
        and "rejected" not in line.lower()
        and "total loop closures" not in line.lower()
    ]
    rejected_loop_closure_lines = [
        line for line in result.stdout.splitlines()
        if "rejected" in line.lower()
    ]
    summary_lines = [
        line for line in result.stdout.splitlines()
        if "total loop closures" in line.lower()
    ]

    # Full stdout saved to the volume (not inlined in the JSON report) so it
    # can be pulled and grepped locally without a Modal rerun if the
    # summary/rejected/accepted line filters above miss something.
    full_log_path = os.path.join(out_dir, "reprocess_stdout_full.log")
    with open(full_log_path, "w") as f:
        f.write(result.stdout)

    # Ground truth (see _link_type_counts's docstring for why the stdout
    # line filters above are approximate only) - only queryable if reprocess
    # actually produced a readable output db.
    link_type_counts = (
        _link_type_counts(output_db)
        if result.returncode == 0 and os.path.exists(output_db)
        else {}
    )

    report = {
        "batch_name": batch_name,
        "extra_rtabmap_args": extra_rtabmap_args,
        "exit_code": result.returncode,
        "summary_lines": summary_lines,
        "link_type_counts": link_type_counts,
        "accepted_global_loop_closures": link_type_counts.get("global_closure", 0),
        "accepted_proximity_closures": link_type_counts.get("local_space_closure", 0),
        "accepted_loop_closure_count_approx": len(accepted_loop_closure_lines),
        "accepted_loop_closure_lines": accepted_loop_closure_lines[:20],
        "rejected_loop_closure_count_approx": len(rejected_loop_closure_lines),
        "rejected_loop_closure_lines": rejected_loop_closure_lines[:20],
        "stderr_tail": result.stderr[-2000:],
        "full_log_path": full_log_path,
    }

    with open(os.path.join(out_dir, "reprocess_report.json"), "w") as f:
        json.dump(report, f, indent=2)

    data_volume.commit()

    return report


# Diagnostic to distinguish "the scene has too little visual texture for
# loop closure" from "the pipeline is mishandling images RTAB-Map could
# otherwise use" - runs a feature detector directly against real recorded
# frames, independent of rtabmap_launch, against a batch that's already on
# the Volume (no sim rerun needed). See
# .claude/internal-docs/compute/slam/CLAUDE.md's loop-closure follow-up.
@app.function(image=diagnostic_image, volumes={"/data": data_volume}, timeout=300)
def extract_and_score_frames(batch_name: str, n_frames: int = 8) -> dict:
    import cv2
    import numpy as np
    import rosbag2_py
    import yaml
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import CompressedImage as CompressedImageMsg

    CAMERA_TOPIC = "/camera/image_raw/compressed"

    bag_path = f"/data/observation_batches/{batch_name}"
    output_dir = f"/data/outputs/{batch_name}/diagnostics/frame_features"
    os.makedirs(output_dir, exist_ok=True)

    with open(os.path.join(bag_path, "metadata.yaml")) as f:
        bag_info = yaml.safe_load(f)["rosbag2_bagfile_information"]

    topic_info = next(
        t for t in bag_info["topics_with_message_count"]
        if t["topic_metadata"]["name"] == CAMERA_TOPIC
    )
    message_count = topic_info["message_count"]
    if message_count == 0:
        raise RuntimeError(
            f"No {CAMERA_TOPIC} messages found in batch '{batch_name}'"
        )

    start_ns = bag_info["starting_time"]["nanoseconds_since_epoch"]
    duration_ns = bag_info["duration"]["nanoseconds"]
    end_ns = start_ns + duration_ns
    # Spread across the full time range, not the first N messages - a
    # time-localized failure (e.g. bag-play rate outrunning extraction
    # later in the run) would be missed by only sampling the start.
    target_timestamps = [
        int(start_ns + (end_ns - start_ns) * i / max(n_frames - 1, 1))
        for i in range(n_frames)
    ]

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
        rosbag2_py.ConverterOptions("", ""),
    )
    reader.set_filter(rosbag2_py.StorageFilter(topics=[CAMERA_TOPIC]))

    gftt = cv2.GFTTDetector_create()
    orb = cv2.ORB_create()

    frame_results = []
    annotated_pngs = []
    target_idx = 0
    while reader.has_next() and target_idx < len(target_timestamps):
        topic, data, t = reader.read_next()
        if t < target_timestamps[target_idx]:
            continue

        # sensor_msgs/msg/CompressedImage.data is already a JPEG byte
        # stream (msg.format e.g. "rgb8; jpeg compressed
        # bgr8"/"jpeg") - decode straight to a BGR array, no manual
        # width/height/step reshape needed (that was only required for the
        # old raw sensor_msgs/msg/Image path).
        msg = deserialize_message(data, CompressedImageMsg)
        encoding = msg.format
        jpeg_bytes = np.frombuffer(msg.data, dtype=np.uint8)
        bgr = cv2.imdecode(jpeg_bytes, cv2.IMREAD_COLOR)
        height, width = bgr.shape[:2]
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

        gftt_keypoints = gftt.detect(gray, None)
        orb_keypoints = orb.detect(gray, None)

        frame_results.append({
            "sample_index": target_idx,
            "timestamp_ns": t,
            "encoding": encoding,
            "width": width,
            "height": height,
            "gftt_keypoint_count": len(gftt_keypoints),
            "orb_keypoint_count": len(orb_keypoints),
        })

        if target_idx == 0 or target_idx == len(target_timestamps) // 2:
            annotated = cv2.drawKeypoints(
                bgr, gftt_keypoints, None, color=(0, 255, 0)
            )
            png_path = os.path.join(output_dir, f"frame_{target_idx:02d}_gftt.png")
            cv2.imwrite(png_path, annotated)
            annotated_pngs.append(png_path)

        target_idx += 1

    summary = {
        "batch_name": batch_name,
        "camera_topic": CAMERA_TOPIC,
        "camera_source_format_urdf": "R8G8B8",
        "message_count_total": message_count,
        "n_frames_requested": n_frames,
        "n_frames_sampled": len(frame_results),
        "frames": frame_results,
        "annotated_pngs": annotated_pngs,
    }

    with open(os.path.join(output_dir, "frame_features.json"), "w") as f:
        json.dump(summary, f, indent=2)

    data_volume.commit()

    return summary


@app.local_entrypoint()
def main() -> None:
    print(smoke_test.remote())


@app.local_entrypoint()
def run_mapping(batch_name: str) -> None:
    report = run_mapping_job.remote(batch_name)
    print(json.dumps(report, indent=2))


@app.local_entrypoint()
def run_frame_diagnostic(batch_name: str, n_frames: int = 8) -> None:
    summary = extract_and_score_frames.remote(batch_name, n_frames)
    print(json.dumps(summary, indent=2))


@app.local_entrypoint()
def run_reprocess_diagnostic(batch_name: str, extra_rtabmap_args: str = "") -> None:
    report = rerun_reprocess.remote(batch_name, extra_rtabmap_args)
    print(json.dumps(report, indent=2))
