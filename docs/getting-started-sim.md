# Getting Started: Simulation

Rambla is being built software-first. Before any physical hardware exists,
the simulation is where the robot's software brain — perception, sensor
fusion, mapping, navigation — gets exercised and validated. The physical
robot remains the source of truth (see `SIM-001`/`SIM-002` in
[DESIGN_SPEC.md](../DESIGN_SPEC.md)): simulation follows the real robot's
requirements, it doesn't define them.

The sim runs on **Gazebo Harmonic** paired with **ROS2 Jazzy**, the same
middleware targeted for the physical robot.

## The Simulation Packages

Simulation code lives under `src/simulation/` as four packages,
each with a distinct role:

- **`rambla_sim_smoke`** — a minimal Gazebo world with a trivial box model
  and odometry bridged to ROS2. This is the baseline "is the toolchain
  working" package: the first thing to check when validating a fresh
  simulation environment.

- **`rambla_description`** — Rambla's simulated body. An xacro robot
  description defining the chassis, differential-drive wheels, caster,
  LiDAR, a forward camera, an IMU, and a front bumper with contact sensors.
  (Physical dimensions here are placeholders per `PHY-004` in the design
  spec, not a committed form factor.)

- **`rambla_sim`** — the apartment-scale scenario. Multi-room apartment
  worlds plus the launch file that spawns the robot and bridges all its
  sensor and control topics into ROS2.

- **`rambla_bagging`** — records an observation batch (rosbag2/MCAP) of the
  stable sensor topic contract during a sim run, for later SLAM/mapping
  processing.

## The Apartment World

`rambla_sim` launches Rambla into a multi-room apartment and exposes the
following ROS2 topics. Two worlds are available via the `world` launch
argument: `apartment_world` (default, hand-authored 3-room layout) and
`house` (adopted 6-room apartment layout, per `SIM-003`):

```
ros2 launch rambla_sim apartment_world.launch.py world:=house
```

| Topic | Purpose |
|---|---|
| `/cmd_vel` | Arbitrated drive command output — sole publisher is `rambla_safety`; bridged to the Gazebo `DiffDrive` plugin |
| `/cmd_vel_raw` | Autonomous drive intent, published by `rambla_traversal` |
| `/cmd_vel_teleop` | Manual drive intent, published by `rambla_control_panel` |
| `/control_authority` | Current arbitration mode (`AUTO`/`MANUAL`), published by `rambla_safety` |
| `/odom` | Raw odometry |
| `/scan` | LiDAR |
| `/camera/image_raw` | Forward camera feed |
| `/imu/data` | IMU |
| bumper contact topics | Front bumper collision sensing |
| `/joint_states` | Joint state (wheels, etc.) |

An EKF sensor-fusion node consumes these to publish a filtered odometry
estimate, which is what localization is built on. `rambla_safety` is the
sole publisher of `/cmd_vel`: it arbitrates between `/cmd_vel_raw`
(autonomous) and `/cmd_vel_teleop` (manual), applying a collision filter to
whichever source has authority — see
`.claude/internal-docs/robot/behavior/rambla_safety/CLAUDE.md` for the full
contract.

See `SIM-003` in the design spec for the requirement this world was
adopted against.

## The Control Panel

`rambla_control_panel` (under `src/server/api/`) is a web-based control and
debug interface that works against the simulated robot today (and is
intended to work against the real robot later). It provides:

- On-screen joystick driving
- A live camera feed
- Sensor and node debug tabs

## Working with the VM

The sim and gateway run on a dev VM (`rambla-vm`) reached over SSH.
`scripts/vm.sh` has ready-made functions for starting/stopping/checking on
both — source it (`source scripts/vm.sh`, or add that line to your shell rc
file) then run `rambla_vm_help` for the full list.

## Where to Look Next

- [architecture.md](architecture.md) — how the three compute planes fit
  together and how mapping actually works
- [../DESIGN_SPEC.md](../DESIGN_SPEC.md) — the full functional requirements
  this simulation work is validating against
