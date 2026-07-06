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
- **[DESIGN_SPEC.md](DESIGN_SPEC.md)** — functional requirements, phased scope (P0–Future), non-goals, open questions.

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

**Open / deferred:**

| Layer | Leading candidate |
|---|---|
| Simulation | Gazebo |
| SLAM | RTAB-Map |
| Navigation | Nav2 |
| Server AI/LLM stack | — |

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
is generated periodically as the project matures.

*(TODO: nothing published yet.)*

## Status

Design and pre-hardware. Software is being built and validated ahead of any
physical hardware purchase, against a body adapter designed to be swapped
for real hardware later.

## Roadmap

A running list of near-term milestones, updated as the project progresses.
Not a full spec — see [DESIGN_SPEC.md](DESIGN_SPEC.md) for the complete
phased requirements.

- [x] Gazebo smoke-test world running (`rambla_sim_smoke`)
- [ ] Basic teleop / manual drive working in sim
- [ ] SLAM producing a usable map in sim
- [ ] Autonomous navigation to a goal pose in sim
- [ ] Autonomous return-to-dock behavior in sim
- [ ] Basic internal state tracking (battery, task, confidence) wired up
- [ ] Hardware selected and BOM drafted
- [ ] First physical prototype assembled

## License

Code is released under the [Apache License 2.0](LICENSE).
