# Robot Interface Contract

Canonical ROS 2 runtime contract: topic names, message types, frame
ownership, and command arbitration for the boundary between application
layers (localization, SLAM, navigation, behavior, control panel) and
whatever publishes the robot's sensors/actuators — simulation today, real
hardware later. Formalizes `DESIGN_SPEC.md` INT-001, INT-002, INT-003,
INT-005. Consolidates prose previously scattered across
`.claude/internal-docs/robot/localization/CLAUDE.md`,
`.claude/internal-docs/robot/behavior/rambla_safety/CLAUDE.md`, and
`.claude/internal-docs/architecture/CLAUDE.md` — those files now point here
rather than restating it.

Per INT-001: application layers consume this contract without caring
whether the publisher underneath is `apartment_world.launch.py` (sim) or a
future real-hardware bridge.

---

## Topic contract

| Topic | Message type | Direction | frame_id | Rate |
|---|---|---|---|---|
| `/scan` | `sensor_msgs/msg/LaserScan` | robot → consumers | `base_scan` | 5 Hz |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | robot → consumers | `camera_link_optical` | 15 Hz |
| `/camera/image_raw/compressed` | `sensor_msgs/msg/CompressedImage` | robot → consumers | `camera_link_optical` | 15 Hz |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | robot → consumers | `camera_link_optical` | 15 Hz |
| `/imu/data` | `sensor_msgs/msg/Imu` | robot → consumers | `imu_link` | 100 Hz |
| `/imu/data_fixed` | `sensor_msgs/msg/Imu` | robot → consumers | `imu_link` | 100 Hz |
| `/odom` | `nav_msgs/msg/Odometry` | robot → consumers | `odom` / `base_footprint` | 50 Hz |
| `/odometry/filtered` | `nav_msgs/msg/Odometry` | robot → consumers | `odom` / `base_footprint` | 30 Hz |
| `/cmd_vel_raw` | `geometry_msgs/msg/Twist` | consumer → robot | — | event-driven |
| `/cmd_vel_teleop` | `geometry_msgs/msg/Twist` | consumer → robot | — | event-driven |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | robot input | — | 20 Hz |
| `/control_authority` | `std_msgs/msg/String` (`"AUTO"`/`"MANUAL"`) | robot → consumers | — | 20 Hz |
| `/bumper_left/contact` | `ros_gz_interfaces/msg/Contacts` | robot → consumers | — | 50 Hz |
| `/bumper_right/contact` | `ros_gz_interfaces/msg/Contacts` | robot → consumers | — | 50 Hz |

Notes:

- **Consumers use `/odometry/filtered`, not raw `/odom`.** `/odom` is
  `ekf_filter_node`'s own (unfused) input, published via the
  `covariance_injector` shim — see the sim-shim section below. Do not wire
  new nodes (SLAM, navigation, a costmap) directly to raw `/odom`.
- **`/imu/data_fixed`** is the `covariance_injector` republish of `/imu/data`
  carrying injected non-zero covariance (see sim-shim section). It is the
  current consumer-facing IMU contract for anything that needs a real
  per-sensor trust signal (currently: `ekf_filter_node`; also recorded into
  the observation batch, see `observation-batch.md`).
- **`/camera/image_raw/compressed`** is a JPEG side channel published by
  `camera_compressor` (`image_transport republish`, launched from
  `apartment_world.launch.py`), derived from `/camera/image_raw` at the same
  15 Hz cadence — not a separate sensor source. It's an *additional*
  publisher, not a replacement: `/camera/image_raw` keeps publishing raw,
  unchanged, at full quality and rate for anything that needs it. Introduced
  for the observation batch (see `observation-batch.md`), where raw camera
  frames dominated batch size, but it's general-purpose — any consumer
  wanting lower bandwidth can subscribe to it instead of raw. JPEG quality is
  the `camera_jpeg_quality` launch arg on `apartment_world.launch.py`.
- `/scan` and `/camera/*` carry correct `frame_id`s at the source (via
  `<gz_frame_id>`/`<optical_frame_id>` tags in `plugins.xacro`) and need no
  `_fixed` republish — unlike `/odom`/`/imu/data`, which still need
  `covariance_injector` for injected covariance (and, for `/odom` only,
  frame_id rewriting — see sim-shim section).
- `/cmd_vel` and `/control_authority` are republished on a steady 20 Hz
  timer (`CMD_VEL_RATE_HZ`) by `rambla_safety`, decoupled from
  sensor/command jitter — not a raw passthrough rate.
- No depth/point-cloud topic exists or is planned at this layer — sensing is
  2D LiDAR + monocular RGB only (`DESIGN_SPEC.md` SNS-003/SNS-007 remain
  placeholders, not committed to depth).

---

## TF / frame ownership

Single-writer discipline per transform — exactly one node publishes each
edge of the tree:

- **`odom → base_footprint`** — owned **solely by `ekf_filter_node`**
  (`publish_tf: true` in `rambla_localization/config/ekf.yaml`). The Gazebo
  `OdometryPublisher` plugin's `<tf_topic>` was deliberately removed from
  `plugins.xacro` to prevent two nodes publishing the same transform and
  corrupting the TF tree — see the inline comment at the removal site. Do
  not re-add `<tf_topic>` there while `ekf_filter_node` runs with
  `publish_tf: true`.
- **`base_footprint → base_link → {sensor/wheel links}`** — owned by
  `robot_state_publisher`, driven by the URDF's fixed-joint chain.
- **`map`** — reserved, unused. No map-frame source exists yet (no
  AMCL/SLAM pose correction integrated); position covariance on
  `/odometry/filtered` grows unboundedly over time by design until one does.

---

## `/cmd_vel` arbitration (INT-005 — already implemented)

`rambla_safety` (`SafetyNode`) is the **sole final publisher to `/cmd_vel`**,
for both the autonomous and manual paths. No other node publishes there.

- **`/cmd_vel_raw`** (in) — autonomous motion intent, from `rambla_traversal`.
- **`/cmd_vel_teleop`** (in) — manual motion intent, from
  `rambla_control_panel`. Kept on a separate topic from `/cmd_vel_raw` by
  design, so arbitration is explicit rather than merged-then-hoped-safe.
- **`/cmd_vel`** (out) — the arbitrated, collision-filtered command.

**AUTO/MANUAL state machine** (`ControlAuthority`):

- **AUTO → MANUAL**: immediate and exclusive on any `/cmd_vel_teleop`
  arrival.
- **MANUAL → AUTO**: only after `manual_timeout_s` (default **0.4 s**) of no
  teleop message.
- Auto commands are frozen (ignored, not just unselected) for the whole time
  mode is MANUAL, so a still-publishing autonomous behavior can't replay a
  stale command the instant MANUAL releases.

The collision filter (bumper contact, or LiDAR front-arc minimum range below
`stop_distance_m`) is applied to whichever source is selected — it is
authoritative for MANUAL too, not just AUTO. Full state-machine detail and
regression traps live in
`.claude/internal-docs/robot/behavior/rambla_safety/CLAUDE.md`.

---

## Control authority + robot status/health

- **`/control_authority`** (`std_msgs/msg/String`, `"AUTO"`/`"MANUAL"`) —
  implemented, published by `rambla_safety` alongside `/cmd_vel` each tick.
  Lets a top-level autonomous-behavior node pause/yield during MANUAL
  without its own arbitration logic (`rambla_traversal` subscribes to this
  today).

> **PLANNED — M10, not implemented.** `/robot_state` is reserved as the name
> for a future robot status/health topic (battery, fault state, etc. — see
> `.claude/internal-docs/architecture/CLAUDE.md`'s planned-topic list). No
> message shape is frozen here; do not build against an assumed schema.

---

## INT-003: sim-shim boundary

Sim-only naming/transport artifacts must not propagate into higher-level
consumers. The current sim-shim surface:

- **`covariance_injector`** (`rambla_localization/covariance_injector.py`,
  launched from `apartment_world.launch.py`) subscribes to raw `/odom` and
  `/imu/data` and republishes `/odom_fixed` / `/imu/data_fixed` with
  injected placeholder covariance — gz-sim's `OdometryPublisher` and IMU
  sensor systems publish all-zero covariance, a known gz-sim gap, not a real
  sensor noise model (`DESIGN_SPEC.md` PHY-004). It also rewrites `/odom`'s
  `frame_id`/`child_frame_id` to plain names (the one frame_id fix it still
  performs — `<gz_frame_id>` is sensor-level only and can't reach the
  `OdometryPublisher` plugin).
- This node is **sim-only and gets dropped, not ported, on real hardware** —
  a real driver/bridge would publish plain frame_ids and real covariance
  directly on `/odom`/`/imu/data`, with no `_fixed` republish needed. Any
  future adapter must not require a consumer to know about `_fixed` topics.
- If gz-sim ever fixes odom frame-id prefixing or starts populating real
  covariance upstream, `covariance_injector` should be retired (or shrunk),
  not left running as a no-op passthrough — see the localization doc's
  "If this gets re-investigated later" note.

---

## No custom messages

Standard ROS 2 messages only (REP-103 for camera/optical frame conventions,
REP-105 for TF frame naming). No custom `.msg` types have been introduced
anywhere in this contract — deliberate, so the same topic/message shapes
work unchanged against a future real-hardware publisher.
