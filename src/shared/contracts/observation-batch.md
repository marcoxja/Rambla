# Observation-Batch Contract

Canonical shape of an observation batch: the recorded input a SLAM (or
future perception) burst-compute job consumes. Formalizes part of
`DESIGN_SPEC.md` INT-004. Consolidates prose previously in
`.claude/internal-docs/compute/slam/CLAUDE.md` and
`.claude/internal-docs/robot/localization/CLAUDE.md` — those files now point
here rather than restating it. Matches what
`src/simulation/rambla_bagging/launch/record_observation_batch.launch.py`
actually records.

---

## Format

A batch is a `rosbag2` recording, written with `--storage mcap` and
`--compression-mode file --compression-format zstd` (lossless — MCAP readers,
including `ros2 bag play` and `rosbag2_py.SequentialReader`, decompress it
transparently, so nothing on the replay/processing side needs to know), of
**exactly eight topics**:

| Topic | Message type | Why it's in the batch |
|---|---|---|
| `/clock` | `rosgraph_msgs/msg/Clock` | Carries no sensor data, but the Modal SLAM job runs with `use_sim_time:=true`, which needs the bag's own simulated time base to replay meaningfully. Without it, `use_sim_time` is a silent no-op. |
| `/scan` | `sensor_msgs/msg/LaserScan` | Raw LiDAR — 2D scan-matching/registration input. Carries a correct `frame_id` at the source (see `robot-interface.md`), so no `_fixed` republish is needed. |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | JPEG-compressed monocular RGB — visual features and loop-closure input for RTAB-Map (LiDAR + monocular RGB mode, not RGB-D — see `map-artifact.md`). **Not** the raw `/camera/image_raw` topic: raw camera frames were the dominant contributor to batch size (a ~2.9-3.1GB/200s batch, "dominated by raw camera images" — see `.claude/internal-docs/compute/slam/CLAUDE.md`), so the batch records the JPEG side channel published continuously by `camera_compressor` (an `image_transport republish` node in `apartment_world.launch.py`) instead. The canonical live `/camera/image_raw` feed (control panel, RViz, `test_camera_image_rate`) is untouched — this is an additional publisher, not a replacement, and it's a general-purpose topic any consumer can subscribe to, not a SLAM-only artifact. JPEG quality is the `camera_jpeg_quality` launch arg on `apartment_world.launch.py` (default 85, validated live 2026-07-11: 72-77x batch-size reduction with no visible map-quality loss). On the processing side, `modal_app.py`'s `run_mapping_job` decompresses this topic back to plain `/camera/image_raw` via its own explicit `image_transport republish` subprocess (`_decompress_rgb_cmd`) before `rtabmap_launch` runs — **not** `rtabmap_launch`'s own built-in `compressed:=true` support, which was tried first but found broken in this ROS Jazzy build (its internal `republish` node never sets the `in_transport` ROS parameter it depends on, so it silently falls back to `raw` and never receives data; root-caused by reading `image_transport`'s `republish.cpp` source directly — see `.claude/internal-docs/compute/slam/CLAUDE.md`). |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Camera intrinsics paired with the camera topic above. |
| `/imu/data_fixed` | `sensor_msgs/msg/Imu` | Not raw `/imu/data` — this is `covariance_injector`'s republish carrying injected non-zero covariance, a requirement for the EKF/RTAB-Map to have a real per-sensor trust signal, not a frame_id fix (see `robot-interface.md`'s sim-shim section). |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | The EKF-fused pose/velocity estimate — **not raw `/odom`**, per the same "consumers use `/odometry/filtered`" rule as the rest of the interface contract. |
| `/tf` | `tf2_msgs/msg/TFMessage` | The dynamic transform tree (notably `odom` → `base_footprint`). Added after the first real M3 mapping run: `rtabmap_launch` looks up this transform via tf2 even with `odom_topic` set to `/odometry/filtered` — the odometry topic alone isn't enough. Without it, every transform lookup fails during replay and RTAB-Map writes an empty database. |
| `/tf_static` | `tf2_msgs/msg/TFMessage` | The static sensor-frame transforms (e.g. `base_footprint` → LiDAR/camera frames), normally latched once by `robot_state_publisher` from the URDF. Nothing publishes these during Modal replay unless they're captured in the batch itself. |

**No depth topic.** LiDAR + monocular RGB only — there is no depth camera in
sim, and the real robot isn't expected to have one either
(`DESIGN_SPEC.md` SNS-003/SNS-007 remain placeholders).

---

## Output layout

A rosbag2 MCAP directory: `metadata.yaml` plus one or more `.mcap` files,
written under:

```
~/rambla_bags/observation_batch_<YYYYMMDD_HHMMSS>/
```

Overridable via the `observation_batch_dir` launch argument to
`record_observation_batch.launch.py`. The timestamped default exists so
repeat recording sessions don't collide.

---

## Versioning & compatibility

The batch's own on-disk format (eight-topic rosbag2/MCAP layout above) is
settled for this milestone. Batch-to-batch versioning, and how a mapping job
declares/checks compatibility with a given batch's format, follow the same
policy as map artifacts — see `map-artifact.md`'s Versioning section for the
shared policy. The durable persistent-state backend for batch metadata
(job records, which batch produced which map version) is a separate, still
open question — `DESIGN_SPEC.md` OQ-014, decision gate README M11. This
contract codifies the batch's *format*, not that backend.
