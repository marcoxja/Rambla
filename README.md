<div align="center">

# Rambla

**A strange little home robot — not quite a pet, not quite an assistant.**

ROS2 (Jazzy) · Raspberry Pi 5 · STM32 / micro-ROS · oomwoo platform (current direction)

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
and social modeling. The body is an abstraction; the project is everything
above it.

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

- **Local (Pi, always-on):** safety, perception, movement, reflexive behavior. Never depends on the network.
- **Server (advisory):** memory, planning, semantic understanding, personality, LLM reasoning. Suggests goals — never controls motion directly.

> The server suggests what the robot should do. The local robot decides how to do it and whether it is safe.

("Server" here is the advisory/cognition role, not necessarily one
always-on machine — see [docs/architecture.md](docs/architecture.md) for
the current three-plane split, where heavy mapping runs as ephemeral burst
compute rather than a persistent host.)

```
   Local (Pi 5, always-on)      Burst compute (Modal)      Persistent state
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
| Body platform | [oomwoo](https://github.com/makerspet/oomwoo) (wheeled, differential drive) |

**Resolved (mapping):**

| Layer | Choice |
|---|---|
| SLAM package | RTAB-Map (runs as an ephemeral burst-compute job on Modal, not on the Pi — see [docs/architecture.md](docs/architecture.md)) |

**Open / deferred:**

| Layer | Leading candidate |
|---|---|
| Simulation | Gazebo |
| Navigation | Nav2 |
| Server AI/LLM stack | — |
| Persistent state (job/map metadata) | Supabase (candidate, not committed) |

## Structure

```
src/
├── robot/               # Runs on the Pi — ROS2 nodes, local intelligence
│   ├── sensors/         # Sensor driver nodes (lidar, camera, imu, tof, cliff, bumper)
│   ├── slam/             # SLAM config + launch files (stack TBD)
│   ├── mapping/          # Topological, semantic, risk layer nodes
│   ├── navigation/       # Navigation stack config + behavior execution (stack TBD)
│   ├── behavior/         # Drive arbitration node
│   └── simulation/       # Simulation world files, robot model, sensor plugins (stack TBD)
│       └── rambla_sim_smoke/  # Minimal Gazebo world + ROS2 bridge smoke test (working)
└── server/               # Runs on the home server
    ├── memory/           # Long-term place and environment store
    ├── ai/               # LLM reasoning, conversation, planning
    ├── social/           # Human and animal profiles
    └── api/              # Server ↔ robot interface

scripts/                  # Setup and helper scripts
shared/                   # Contracts, schemas, shared docs
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

### Completed

- [x] Gazebo smoke-test world running (`rambla_sim_smoke`)
- [x] Rambla robot description + apartment world in sim (`rambla_description`, `rambla_sim`) — LiDAR, camera, IMU, bumper, diff-drive, odometry _(first pass — apartment world and robot geometry are placeholder-grade; hardened in M1)_
- [x] EKF sensor fusion (`rambla_localization`) — `/odometry/filtered`, verified no longer diverging unboundedly (was 18+ meters, pinned, non-recovering); tested 4x under the audit's drive test with 0.005m-3.4m of normal dead-reckoning spread, never pinned/corrupted
- [x] Pre-SLAM foundation fixes — EKF covariance, `/scan`+camera frame_id fix, RViz config, and smoke test all verified end-to-end in sim
- [x] Basic teleop / manual drive working in sim (`rambla_control_panel`) — web-based joystick drive, live camera feed, and sensor/node debug tabs; broader/public reachability is deferred to `DESIGN_SPEC.md` CTL-009
- [x] RTAB-Map + ROS2 deps packaged as a container and validated running on Modal (smoke test: container builds, `rtabmap_ros` found, exits cleanly)
- [x] Local collision-safety reflex (`rambla_safety`) and autonomous mapping-traversal behavior (`rambla_traversal`, `rambla_bagging`) — reproducible hands-off wander + recording run in sim, no map dependency _(both first pass — stop-only safety and crude wander coverage; see Hardening backlog)_

### M1 — Sim world + robot geometry polish (SIM-003, PHY-004)

The apartment world and robot description are placeholder-grade; this
hardens them before map/nav quality depends on them.

- [ ] Adopt the enhanced apartment world per [RAMBLA_APARTMENT_WORLD_ENHANCEMENT.md](RAMBLA_APARTMENT_WORLD_ENHANCEMENT.md) — multi-room `house.world` baseline with higher-fidelity, low-compute furniture geometry and visual/semantic distinction (SIM-003)
- [ ] Fix the robot bumper geometry (currently two flat chords per side meeting in a ~5cm forward-protruding wedge at the front centerline, not a smooth arc — distorts collision/clearance behavior; see `.claude/internal-docs/audits/2026-07-08-accepted-stabilization-findings.md` §3/P1-1 for the precise geometry) and revisit sensor placement/mounting on the description
- [ ] Keep the simpler worlds available for focused debugging and regression

_Done when: SLAM and nav runs use the enhanced world and a corrected robot body, so map quality and collision behavior aren't fighting placeholder geometry._

### M2 — Real map in sim (MAP-001)

- [ ] Run the record → Modal Volume → RTAB-Map → artifact pipeline end-to-end (Phase 1 recording is already built via `rambla_bagging`; this is Phases 2-5 of the SLAM real-map plan)
- [ ] Validate that loop closure fires and the occupancy grid resembles the apartment (grep `processing_report.json`, eyeball `map_map.pgm`)

_Done when: a versioned `map_v001` artifact showing visible apartment structure comes back from Modal._

> ⚠ Map quality depends on M1's enhanced world and the barebones mapping-run wander (see Hardening backlog) — a fuller-coverage run gives a better first map.

### M3 — Localize against the cached map on the Pi (MAP-005)

- [ ] Add a Pi-side localization node that scan-matches `/scan_fixed` against the retrieved map artifact and publishes the `map→odom` correction (closes the "no map-frame correction" gap the EKF audit noted by design)
- [ ] Surface live position for the control panel's future map HUD (unblocks CTL-005)

_Done when: the robot reports an absolute pose in the map frame that stays bounded while driving._

### M4 — Decide the navigation stack *(decision gate — OQ-003)*

- [ ] Evaluate Nav2 vs. alternatives against the record-then-process / cached-map / local-safety constraints; record the decision and rationale, resolving OQ-003

_Done when: a nav stack is chosen and written down, so M5 config work isn't speculative._

### M5 — Autonomous navigation to a goal pose in sim (NAV-001/002/003)

- [ ] Bring up the chosen nav stack against the cached map + M3 localization, with `rambla_safety` remaining the sole final `/cmd_vel` arbiter for the autonomous path (manual teleop via `rambla_control_panel` bypasses `rambla_safety` by current design and is a separate, unresolved motion-authority question — see accepted stabilization findings §5/P1-3)
- [ ] Point-to-point goal navigation with obstacle avoidance and detectable/recoverable failure (NAV-004)

_Done when: the robot drives to a commanded goal pose and stops — or reports failure — without a collision._

> ⚠ The safety reflex is a first-pass stop-only implementation (see Hardening backlog) — nav failure-recovery may need it upgraded to back-off/re-plan behavior.

### M6 — Reliability hardening pass

- [ ] Add a minimal `launch_testing` contract check (assert `/odometry/filtered`, `/tf`, `/scan_fixed`, and map topics publish at expected rates with expected frame_ids) — belongs in the currently-empty `shared/`
- [ ] Confirm `bumper_right` + the bridged contact-topic layer before collision-recovery code depends on it
- [ ] Add a launch-ordering/readiness guard to `apartment_world.launch.py`

_Done when: a launch-time smoke test catches a broken topic/frame contract automatically, instead of by chance._

### M7 — Autonomous return-to-dock in sim (CHG-001/004, OQ-005)

- [ ] Model a dock in the sim world plus a dock-approach/detection method (resolve OQ-005 for sim)
- [ ] Implement return-to-dock as a navigation behavior on top of M5, runnable with the server unavailable (LOC-003)

_Done when: from an arbitrary pose the robot navigates to and aligns with the dock, locally, with no network dependency._

### M8 — Basic internal state tracking (STA-001/002, CHG-002)

- [ ] Wire a state node: battery/power estimate (sim-modeled), current task, and a nav/localization confidence signal, published on a stable topic
- [ ] State influences behavior (low battery → prefer dock; low confidence → slow/stop per SAF-003) and is surfaced read-only in the control panel (unblocks CTL-011)

_Done when: internal state is published, visible in the panel, and demonstrably changes at least one behavior._

### M9 — Decide the persistent-state backend *(decision gate — OQ-014)*

- [ ] Choose where job records / map version history live (Supabase candidate); resolve OQ-014 and define the observation-batch + artifact-versioning contract beyond the current ad-hoc Modal Volume

_Done when: a backend is chosen and the map-versioning/job-record contract is written down._

### M10 — Hardware selection & BOM (OQ-001/OQ-002, PHY-004)

- [ ] Select sensors/compute against the sim-proven stack; draft the BOM; identify which oomwoo components are reusable

_Done when: a concrete BOM exists and the PHY-004 placeholder dimensions/sensors are replaced with committed parts._

### M11 — First physical prototype assembled

- [ ] Assemble the base and bring up real-hardware drivers (dropping the sim-only `frame_id_fixer` shim per the portability note)
- [ ] Re-verify the safety reflex on real sensors

_Done when: the physical robot runs the local safety + teleop stack end-to-end._

## Hardening backlog

Pieces that work today but are first-pass or barebones — tracked here so
they aren't mistaken for finished. Each is pulled into a milestone (or its
own focused pass) when it starts to hurt; the trigger is noted.

- **Safety reflex** (`rambla_safety`) — first-pass stop-only: halts on bumper/LiDAR but has no back-off, re-plan, or recovery. _Revisit when M5 nav failure-recovery needs more than a hard stop._
- **Mapping-run wander** (`rambla_traversal`) — crude reactive coverage, well below Roomba-grade systematic exploration; leaves gaps a good first map wants filled. _Revisit for M2 map completeness / coverage quality._
- **Robot bumper + sensor placement** (`rambla_description`) — bumper bars form a two-flat-chord wedge extending ~5cm past the body at the front centerline (not a smooth arc) and sensor mounting is placeholder. _Folded into M1; re-verify on real hardware at M11._
- **EKF noise model** (`ekf.yaml`, `frame_id_fixer.py`) — hand-picked placeholder covariance, not a real sensor-noise model (PHY-004). _Revisit when real hardware sensor characteristics are known (M10/M11)._
- **Data contract & tests** — topic/frame contract is prose-only and automated coverage is thin. _Partially addressed in M6; expand as more topics stack up._

## License

Code is released under the [Apache License 2.0](LICENSE).
