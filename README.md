<div align="center">

# Rambla

**A strange little home robot — not quite a pet, not quite an assistant.**

ROS2 (Jazzy) · Raspberry Pi 5 · STM32 / micro-ROS · oomwoo form factor (current direction)

![License](https://img.shields.io/badge/license-Apache--2.0-blue)
![Status](https://img.shields.io/badge/status-design%20%2F%20pre--hardware-orange)

</div>

## What is this?

For the authoritative definition of the project, read:

- **[PROJECT_VISION.md](PROJECT_VISION.md)** — narrative vision, personality direction, design principles.
- **[DESIGN_SPEC.md](DESIGN_SPEC.md)** — functional requirements, phased scope (P0–Future), non-goals, open questions. This now also covers a typed activity/cognition/operational-history capability area — recording perceptions, thoughts, goals, decisions, state changes, and incidents over time.

Rambla is a low-profile wheeled home robot — a robot-vacuum-style form
factor ([oomwoo](https://github.com/makerspet/oomwoo) platform) without the
cleaning hardware, no arms, legs, or humanoid locomotion. It roams the
apartment, maps its world, notices what changes, listens when people talk,
and occasionally rolls into the room with an opinion nobody asked for. This
repo is the software brain behind it: cognition, mapping, behavior, memory,
and social modeling. Hardware-specific implementations are replaceable
beneath a stable robot-facing contract; the repository includes the
reference robot description, simulation, platform adapters, and all
higher-level software.

The long-term goal is a capable, strange little intelligence sharing the
home — closer in spirit to Rocky from *Project Hail Mary* than to a robot
pet or a chatbot on wheels: curious, observant, independent, occasionally
stubborn, and grounded in what it can actually sense and know. It should not
fake certainty or perform a personality; character should emerge from real
behavior and observation, not constant talking. Utility grows over time —
starting with navigation, mapping, and basic observation, then expanding
toward roaming security-lite awareness, room-change detection, mess
awareness, voice interaction, familiar-person recognition, expressive eyes,
and behavior driven by internal state rather than simple commands.

## Design Philosophy

The system is layered intelligence, not a monolithic AI controller:

- **Robot-local (Pi, always-on):** safety, perception, movement, reflexive behavior. Never depends on the network.
- **Advisory (burst compute, persistent state, hosted services):** memory, planning, semantic understanding, personality, LLM reasoning. Suggests goals — never controls motion directly.

> The advisory plane suggests what the robot should do. The robot-local plane decides how to do it and whether it is safe.

(The advisory plane spans three separate placements — burst compute,
persistent state, and hosted services — not one always-on machine; see
[docs/architecture.md](docs/architecture.md) /
`.claude/internal-docs/architecture/CLAUDE.md`'s Runtime and Compute
Placement section for the full split.)

```
   Robot-local (Pi 5, always-on)   Burst compute (Modal)      Persistent state
   ┌─────────────────────┐      ┌───────────────────┐      ┌───────────────────┐
   │ safety · sensing    │batch │ RTAB-Map SLAM     │ save │ Supabase          │
   │ movement · nav      │─────▶│ ephemeral,        │─────▶│ (candidate)       │
   │ never network-      │◀─────│ on-demand         │      │ job + map history │
   │ dependent           │ map  └───────────────────┘      └───────────────────┘
   └─────────────────────┘ artifact
```

## Stack

DESIGN_SPEC.md deliberately keeps exact architecture and hardware open
(NG-008, NG-009, OQ-001–OQ-003) so the project isn't locked in too early.
The rows below are the current leading direction, not settled decisions —
expect them to be revisited as hardware is selected and design work
continues.

**Current direction:**

| Layer | Choice |
|---|---|
| Middleware | ROS2 (Jazzy) |
| Robot compute | Raspberry Pi 5 (8GB) |
| Hardware bridge | STM32 PCB via micro-ROS |
| Body platform | [oomwoo](https://github.com/makerspet/oomwoo) form factor — wheeled, differential drive (form-factor inspiration and a possible future Kaia.ai firmware/micro-ROS bridge at real-hardware time; Rambla's sim/description stack is Rambla's own, not adopted from OOMWOO-One/Kaia — see `.claude/internal-docs/architecture/CLAUDE.md`'s foundation-decision section) |

**Decided:**

| Layer | Choice |
|---|---|
| SLAM package | RTAB-Map (decided) — runs as an ephemeral burst-compute job on Modal, not on the Pi — see [docs/architecture.md](docs/architecture.md) |
| Burst compute for SLAM | Modal (decided/current) — this is a decision about SLAM's compute specifically, not a commitment to Modal for every future burst workload; see "Open / deferred" below |

**Current development platform** (subject to future reassessment, not a permanent commitment like the row above):

| Layer | Choice |
|---|---|
| Simulation | Gazebo Harmonic (see [docs/getting-started-sim.md](docs/getting-started-sim.md)) |

**Open / deferred:**

| Layer | Leading candidate | Status |
|---|---|---|
| Navigation | Nav2 | Open — decision gate at M6 |
| Hosted API/control services | — | Open |
| Burst compute provider (beyond SLAM) | — | Not decided whether Modal is used for every future burst workload |
| Cognition/LLM provider | — | Open |
| Persistent state (job/map metadata) | Supabase (candidate, not committed) | Open — decision gate at M11 |

## Structure

This tree reflects the current physical layout on disk: directory name and
runtime placement now match. `robot/` is Pi-side only; the simulation
packages, the Modal-side SLAM burst-compute job, and the hosted/advisory
namespace each got their own top-level directory in the Session 6 `src/`
reorganization (tracked in the documentation remediation plan). See
`.claude/internal-docs/architecture/CLAUDE.md`'s Runtime and Compute
Placement for the placement rationale behind this split.

```
src/
├── robot/               # Pi-side ROS2 nodes — robot-local intelligence
│   ├── sensors/          # Sensor driver nodes (lidar, camera, imu, tof, cliff, bumper) — not yet its own package; sensors are modeled in rambla_description for now
│   ├── mapping/          # Topological, semantic, risk layer nodes (not started)
│   ├── localization/     # EKF sensor fusion (`rambla_localization`) — publishes `/odometry/filtered`
│   ├── navigation/       # Navigation stack config + behavior execution (stack TBD — Nav2 leading candidate, M6 decision gate)
│   └── behavior/         # Robot-local safety-reflex + AUTO/MANUAL `/cmd_vel` arbitration (`rambla_safety`), autonomous mapping-traversal (`rambla_traversal`)
├── simulation/           # Gazebo Harmonic sim — robot description, apartment/house worlds, smoke test, observation-batch recording (NOT Pi-side)
│   ├── rambla_sim_smoke/    # Minimal Gazebo world + ROS2 bridge smoke test
│   ├── rambla_description/  # Robot xacro description — chassis, wheels, LiDAR, camera, IMU, bumper
│   ├── rambla_sim/          # Apartment-scale worlds (`apartment_world`, `house`) + bringup launch/bridge
│   └── rambla_bagging/      # Records observation batches for SLAM processing
├── compute/
│   └── slam/             # Modal burst-compute container (not Pi-executed) — RTAB-Map + ROS2 deps; record→process pipeline validated end-to-end (M3), producing a real `map_v001`; localizing the Pi against it is M5; was src/robot/slam/
├── server/               # Hosted-service + advisory namespace — not one always-on host (see Runtime and Compute Placement); persistent state and burst compute live in that same split, not "on the server"
│   ├── memory/           # Long-term place and environment store (not started) — future Persistent state plane
│   ├── cognition/        # LLM reasoning, conversation, planning (not started) — future Cognition hosted service; was server/ai/
│   ├── social/           # Human and animal profiles (not started)
│   └── api/              # Hosted API / robot-facing relay
│       └── rambla_control_panel/  # Web joystick teleop, live camera feed, sensor/node debug tabs — Hosted service (README M4)
└── shared/
    └── contracts/        # Contracts, schemas, shared docs — `robot-interface.md`, `observation-batch.md`, `map-artifact.md` (Roadmap M2)

scripts/                  # Setup and helper scripts
docs/                     # Polished, external-facing documentation (generated periodically)
assets/                   # Images and other non-code project assets
```

## Documentation

The project definition lives at the repo root: [DESIGN_SPEC.md](DESIGN_SPEC.md)
(requirements) and [PROJECT_VISION.md](PROJECT_VISION.md) (vision and
personality direction).

Polished, external-facing documentation lives in [docs/](docs/README.md) and
is generated periodically as the project matures. Start with
[docs/architecture.md](docs/architecture.md) (system architecture) and
[docs/getting-started-sim.md](docs/getting-started-sim.md) (simulation
orientation).

## Status

Design and pre-hardware. Software is being built and validated ahead of any
physical hardware purchase, against a body adapter designed to be swapped
for real hardware later.

## Roadmap

An ordered plan of milestones from here through the first physical
prototype, updated as the project progresses. Each milestone lists its
sub-tasks and the condition that marks it done; requirement IDs in
parentheses trace back to [DESIGN_SPEC.md](DESIGN_SPEC.md), which remains
the complete phased spec. Two entries are **decision gates** — the spec
deliberately leaves the choice open, and it is made at the last responsible
moment, right before the work that depends on it. A ⚠ note on a milestone
flags a first-pass piece that should be hardened before that milestone
relies on it; longer-lived "works but barebones" items live in the
**Hardening backlog** at the end.

### M0 — Initial simulation and system foundation

- [x] Gazebo smoke-test world running (`rambla_sim_smoke`)
- [x] Rambla robot description + apartment world in sim (`rambla_description`, `rambla_sim`) — LiDAR, camera, IMU, bumper, diff-drive, odometry _(first pass — apartment world and robot geometry are placeholder-grade; hardened in M1)_
- [x] EKF sensor fusion (`rambla_localization`) — `/odometry/filtered`, verified no longer diverging unboundedly (was 18+ meters, pinned, non-recovering); tested 4x under the audit's drive test with 0.005m-3.4m of normal dead-reckoning spread, never pinned/corrupted
- [x] Pre-SLAM foundation fixes — EKF covariance, `/scan`+camera frame_id fix, RViz config, and smoke test all verified end-to-end in sim
- [x] Basic teleop / manual drive working in sim (`rambla_control_panel`) — web-based joystick drive, live camera feed, and sensor/node debug tabs; broader/public reachability is deferred to `DESIGN_SPEC.md` CTL-009, scheduled at M4
- [x] RTAB-Map + ROS2 deps packaged as a container and validated running on Modal (smoke test: container builds, `rtabmap_ros` found, exits cleanly)
- [x] Local collision-safety reflex (`rambla_safety`) and autonomous mapping-traversal behavior (`rambla_traversal`, `rambla_bagging`) — reproducible hands-off wander + recording run in sim, no map dependency _(both first pass — stop-only safety and crude wander coverage; see Hardening backlog)_

### M1 — Sim world + robot geometry polish (SIM-003, PHY-004)

The apartment world and robot description were placeholder-grade; this
hardens them before map/nav quality depends on them.

- [x] Adopt the enhanced apartment world — multi-room `house.world` baseline with higher-fidelity, low-compute furniture geometry and visual/semantic distinction (SIM-003) _(adopted as `house.sdf`, a second selectable world via `world:=house`; furniture fidelity pass — legs/clearance, stacked-box silhouettes, rugs — done and verified live in the VM)_
- [x] Fix the robot bumper geometry (currently two flat chords per side meeting in a ~5cm forward-protruding wedge at the front centerline, not a smooth arc — distorts collision/clearance behavior; see `.claude/internal-docs/audits/2026-07-08-accepted-stabilization-findings.md` §3/P1-1 for the precise geometry) and revisit sensor placement/mounting on the description _(bumper: each half is now a solid circular-segment mesh following the true body radius — contact function preserved (`bumper_left`/`bumper_right` links, sensors, and topics unchanged) and verified live: real contact fires on both sides against actual wall geometry, and the safety stop script halts precisely at the configured 0.35m threshold with no contact. Sensor placement: root-caused the LiDAR's forward self-return to the camera (not the bumper, as previously documented) and fixed it by raising the LiDAR turret so the scan plane clears the camera — confirmed live, no near-range self-return in the front arc; `min_valid_range_m` dropped 0.3→0.12 accordingly. See §3/§4 of the audit doc for the full resolution.)_

_Retained regression fixtures (not gating, already true — kept for focused debugging): `apartment_world.sdf` (the original 3-room layout, still the launch default) and `rambla_sim_smoke`'s `smoke_world.sdf` both remain in the repo and selectable; neither was removed when `house.sdf` was adopted._

_Done when: SLAM and nav runs use the enhanced world and a corrected robot body, so map quality and collision behavior aren't fighting placeholder geometry._

### M2 — System boundary contract foundation

Makes the application layers independent of whether their source is
simulation or physical hardware, before localization and navigation get
built on top of ad hoc topic/frame assumptions. `src/shared/contracts/`
now holds the three canonical contract files below (session 1 of this
milestone). See
`.claude/internal-docs/architecture/CLAUDE.md`'s Runtime and Compute
Placement principles for the boundary types this codifies.

- [x] Canonical ROS runtime contract: topic names, message types, and TF/frame ownership for both sim and (future) physical adapters, including the single-writer TF and `/cmd_vel` invariants already established informally in `architecture/CLAUDE.md` — `src/shared/contracts/robot-interface.md`
- [x] Observation-batch contract (topic set, metadata, and output layout that `rambla_bagging` records) — `src/shared/contracts/observation-batch.md`
- [x] Map-artifact contract (`rtabmap.db` + occupancy grid + processing report layout and versioning) — `src/shared/contracts/map-artifact.md`
- [x] Robot status/health contract and control-authority/command contract — `/control_authority` documented in `robot-interface.md`; `/robot_state` reserved as PLANNED — M10, not implemented
- [x] Versioning and compatibility policy for the above — in `map-artifact.md` (shared with `observation-batch.md`)
- [x] Early interface-contract smoke test: a minimal `launch_testing` check asserting `/odometry/filtered`, `/tf`, `/scan`, and camera topics publish at expected rates with expected frame_ids (moved up from the old late-stage contract-test slot — see M8 for the later reliability-hardening version) — `src/simulation/rambla_sim/test/test_smoke_topics.py`

_Done when: the ROS runtime, observation-batch, and map-artifact contracts are written down and a minimal automated check enforces the ROS runtime contract, so M3-onward work has something concrete to build against instead of ad hoc assumptions._

> Formalized in `DESIGN_SPEC.md` as INT-001..006 (interface-portability requirements). Contract docs (INT-001..005) and INT-006's automated check are both implemented.

### M3 — Real map in sim (MAP-001)

- [x] Complete and validate the observation-recording stage, then run the record → Modal Volume → RTAB-Map → artifact pipeline end-to-end _(validated live 2026-07-11: an 8-topic `rambla_bagging` batch — the original six plus `/tf`/`/tf_static`, added mid-milestone, see below — recorded on `house.sdf`, uploaded to the `rambla-slam-data` Modal Volume, and run through `run_mapping_job`'s bag-play → live `rtabmap_launch` node graph → SIGINT-flush → `rtabmap-reprocess` pipeline; `map_v001` retrieved)_
- [x] Validate the occupancy grid resembles the apartment (grep `processing_report.json`, eyeball `map_map.pgm`) _(`map_map.pgm` shows real wall/hallway structure, not blank/noise, from a 200s/~26m traverse. Loop closure initially didn't fire — root-caused to this rig being monocular + LiDAR, so RTAB-Map's default vision-based loop-closure verification could never pass (no 3D visual words from a mono camera); fixed by verifying via LiDAR ICP instead (`Reg/Strategy=1` + `RGBD/LoopClosureIdentityGuess=true` + loosened ICP bounds). Confirmed live: 375-387 accepted loop closures, up from 0 — see `compute/slam/CLAUDE.md` for the full root-cause trace and fix)_
- [x] Amend the observation-batch camera contract to substantially reduce
  mapping-batch storage and transfer size without reducing the quality or
  availability of the robot's source camera stream _(validated live
  2026-07-11: added an always-on `/camera/image_raw/compressed` JPEG side
  channel (`camera_compressor`, an `image_transport republish` node in
  `apartment_world.launch.py`) alongside the untouched raw `/camera/image_raw`
  feed; the observation batch now records the compressed topic plus MCAP
  `zstd` compression instead of raw frames. Re-recorded the same 200s/~26m
  traverse: batch size dropped from ~2.9-3.1GB to **38.3MiB (~72-77x
  smaller)**, `modal volume put` dropped from ~18min to **13.7s**. The Modal
  mapping job needed a real fix, not just a config change: `rtabmap_launch`'s
  own built-in `compressed:=true` support turned out to be broken in this
  ROS Jazzy build (its internal `republish` node never actually sets the
  `in_transport` ROS parameter it depends on, root-caused by reading
  `image_transport`'s `republish.cpp` source directly) — replaced with an
  explicit decompressor subprocess in `modal_app.py`, verified live via
  `modal shell` before being wired into the real job. End-to-end run on the
  new batch: 200 working-memory nodes, 14 global + 373 proximity loop
  closures accepted, and `map_map.pgm` shows the same real wall/room
  structure as the pre-compression baseline — see `compute/slam/CLAUDE.md`
  for the full trace)_

M3 is complete: a representative observation batch was recorded and
processed end to end into a valid versioned map artifact, and the
mapping-only camera representation has been validated to avoid the previous
raw-image data volume without degrading RTAB-Map input quality or map
output. **Met.**

> ⚠ Map quality depends on M1's enhanced world and the barebones mapping-run wander (see Hardening backlog) — a fuller-coverage run gives a better first map.

### M4 — Deployable control panel (CTL-009)

Takes the sim-only control panel (`rambla_control_panel`, built in M0) to a
hosted, securely-reachable deployment without exposing ROS 2 directly to
the public internet — see audit finding #10 for the full rationale.

- [ ] Deploy to a hosted platform (e.g. Vercel or equivalent) behind basic authentication
- [ ] Add a controlled robot-facing relay/API in front of ROS 2 — no direct public ROS 2 exposure
- [ ] Default to read-only diagnostics (camera feed, sensor/debug tabs); gate teleop behind an explicit action, protected more strongly than basic dashboard viewing
- [ ] Handle the robot-unreachable case gracefully (offline state, no hung UI)

_Done when: the control panel is reachable from the public internet with basic auth, ROS 2 itself is not directly exposed, and teleop is harder to reach than read-only viewing._

### M5 — Localize against the cached map on the Pi (MAP-005)

- [ ] Add a Pi-side localization node that scan-matches `/scan` against the retrieved map artifact and publishes the `map→odom` correction (closes the "no map-frame correction" gap the EKF audit noted by design)
- [ ] Surface live position for the control panel's future map HUD (unblocks CTL-005)

_Done when: the robot reports an absolute pose in the map frame that stays bounded while driving._

### M6 — Decide the navigation stack *(decision gate — OQ-003)*

- [ ] Evaluate Nav2 vs. alternatives against the record-then-process / cached-map / local-safety constraints; record the decision and rationale, resolving OQ-003

_Done when: a nav stack is chosen and written down, so M7 config work isn't speculative._

### M7 — Autonomous navigation to a goal pose in sim (NAV-001/002/003)

- [ ] Bring up the chosen nav stack against the cached map + M5 localization; `rambla_safety` is already the sole final `/cmd_vel` arbiter for both the autonomous and manual (AUTO/MANUAL, `rambla_control_panel`) paths — see `.claude/internal-docs/robot/behavior/rambla_safety/CLAUDE.md` for the arbitration contract (resolves P1-3)
- [ ] Point-to-point goal navigation with obstacle avoidance and detectable/recoverable failure (NAV-004)

_Done when: the robot drives to a commanded goal pose and stops — or reports failure — without a collision._

> ⚠ The safety reflex is a first-pass stop-only implementation (see Hardening backlog) — nav failure-recovery may need it upgraded to back-off/re-plan behavior.

### M8 — Reliability hardening pass

- [ ] Expand the M2 interface-contract smoke test into full reliability-hardening coverage: sustained-runtime rate assertions, edge-case frame_id regressions, and observation-batch/map-artifact compatibility checks — not just presence/rate checks on live ROS topics
- [x] Confirm `bumper_right` + the bridged contact-topic layer before collision-recovery code depends on it _(confirmed live in the M1 bumper-geometry pass, 2026-07-09: driving into a wall produced real `gz.msgs.Contacts` data on both `/bumper_left/contact` and `/bumper_right/contact`, correct collision names on both sides, bridged correctly under both `apartment_world` and `house`)_
- [x] Add a launch-ordering/readiness guard to `apartment_world.launch.py` _(event-driven two-gate sequencing replaced the prior blind `TimerAction(3.0)`: robot spawn waits for `robot_state_publisher` to start, and bringup — bridge/EKF/safety — waits for spawn to exit 0, failing visibly on a nonzero spawn exit instead of leaving nodes hanging)_
- [ ] Add a `SafetyMonitor` boundary-value unit test (0.20m/0.29m/0.31m/0.34m across multiple angles in the front arc, in the existing `test_safety_monitor.py` style) — direct regression coverage for the current LiDAR self-detection fix, no sim required (resolves the stabilization audit's P1-5, see `.claude/internal-docs/audits/2026-07-08-accepted-stabilization-findings.md` §10)
- [ ] Define a reproducible build boundary (base image or install script + `rosdep` resolution for `ros_gz_*`/`robot_localization`/`xacro`, none of which is declared anywhere machine-readable as an apt dependency today) and add one minimal build-only CI job gated on it, no Gazebo/display dependency (resolves the stabilization audit's P1-6 + its CI follow-on)

_Done when: contract-test coverage extends beyond the M2 baseline to batch/artifact compatibility and sustained-runtime reliability, catching a broken topic/frame/artifact contract automatically instead of by chance._

### M9 — Autonomous return-to-dock in sim (CHG-001/004, OQ-005)

- [ ] Model a dock in the sim world plus a dock-approach/detection method (resolve OQ-005 for sim)
- [ ] Implement return-to-dock as a navigation behavior on top of M7, runnable with the server unavailable (LOC-003)

_Done when: from an arbitrary pose the robot navigates to and aligns with the dock, locally, with no network dependency._

### M10 — Basic internal state tracking (STA-001/002, CHG-002)

- [ ] Wire a state node: battery/power estimate (sim-modeled), current task, and a nav/localization confidence signal, published on a stable topic
- [ ] State influences behavior (low battery → prefer dock; low confidence → slow/stop per SAF-003) and is surfaced read-only in the control panel (unblocks CTL-011)

_Done when: internal state is published, visible in the panel, and demonstrably changes at least one behavior._

### M11 — Decide the persistent-state backend *(decision gate — OQ-014)*

- [ ] Choose where job records / map version history live (Supabase candidate); resolve OQ-014 and define the observation-batch + artifact-versioning contract beyond the current ad-hoc Modal Volume

_Done when: a backend is chosen and the map-versioning/job-record contract is written down._

### M12 — Hardware selection & BOM (OQ-001/OQ-002, PHY-004)

- [ ] Select sensors/compute against the sim-proven stack; draft the BOM; identify which oomwoo components are reusable

_Done when: a concrete BOM exists and the PHY-004 placeholder dimensions/sensors are replaced with committed parts._

### M13 — First physical prototype assembled

- [ ] Assemble the base and bring up real-hardware drivers (dropping the sim-only `frame_id_fixer` shim per the portability note)
- [ ] Re-verify the safety reflex on real sensors

_Done when: the physical robot runs the local safety + teleop stack end-to-end._

## Hardening backlog

Pieces that work today but are first-pass or barebones — tracked here so
they aren't mistaken for finished. Each is pulled into a milestone (or its
own focused pass) when it starts to hurt; the trigger is noted.

- **Safety reflex** (`rambla_safety`) — first-pass stop-only: halts on bumper/LiDAR but has no back-off, re-plan, or recovery. _Revisit when M7 nav failure-recovery needs more than a hard stop._
- **Mapping-run wander** (`rambla_traversal`) — crude reactive coverage, well below Roomba-grade systematic exploration; leaves gaps a good first map wants filled. _Revisit for M3 map completeness / coverage quality._
- **EKF noise model** (`ekf.yaml`, `frame_id_fixer.py`) — hand-picked placeholder covariance, not a real sensor-noise model (PHY-004). _Revisit when real hardware sensor characteristics are known (M12/M13)._
- **Data contract & tests** — topic/frame contract is prose-only and automated coverage is thin. _Partially addressed in M2 (early interface-contract smoke test); expanded in M8 as more topics stack up._
- **EKF covariance unit test** (`covariance_injector.py`'s covariance-injection logic) — guards the single worst historical bug (18m EKF divergence); the integration smoke test only partially covers it today. _Revisit alongside M8's contract-test work._
- **README Structure diagram** — visually distinguish placeholder vs. implemented `server/`/`robot/` subdirectories so an at-a-glance read doesn't imply parity. _Low-urgency cosmetic fix._
- **`.gitignore` stale build-artifact path** — two-line correction, no urgency, no artifacts have leaked.
- **`rambla_traversal` / `rambla_control_panel` subsystem `CLAUDE.md` docs** — simpler invariants than `rambla_safety`'s, already stated in existing docstrings. _Write when either subsystem's contract needs to be discoverable outside its source._

## License

Code is released under the [Apache License 2.0](LICENSE).
