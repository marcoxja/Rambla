# Getting Started: Simulation

Rambla is being built software-first. Before any physical hardware exists,
the simulation is where the robot's software brain — perception, sensor
fusion, mapping, navigation — gets exercised and validated. The physical
robot remains the source of truth (see `SIM-001`/`SIM-002` in
[DESIGN_SPEC.md](../DESIGN_SPEC.md)): simulation follows the real robot's
requirements, it doesn't define them.

The sim runs on **Gazebo Harmonic** paired with **ROS2 Jazzy**, the same
middleware targeted for the physical robot.

## The Three Simulation Packages

Simulation code lives under `src/robot/simulation/` as three packages,
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

- **`rambla_sim`** — the apartment-scale scenario. A multi-room apartment
  world plus the launch file that spawns the robot and bridges all its
  sensor and control topics into ROS2.

## The Apartment World

`rambla_sim` launches Rambla into a multi-room apartment and exposes the
following ROS2 topics:

| Topic | Purpose |
|---|---|
| `/cmd_vel` | Drive command input |
| `/odom` | Raw odometry |
| `/scan` | LiDAR |
| `/camera/image_raw` | Forward camera feed |
| `/imu/data` | IMU |
| bumper contact topics | Front bumper collision sensing |
| `/joint_states` | Joint state (wheels, etc.) |

An EKF sensor-fusion node consumes these to publish a filtered odometry
estimate, which is what localization is built on.

A larger apartment world is planned as a forward enhancement — see
`SIM-003` in the design spec.

## The Control Panel

`rambla_control_panel` (under `src/server/api/`) is a web-based control and
debug interface that works against the simulated robot today (and is
intended to work against the real robot later). It provides:

- On-screen joystick driving
- A live camera feed
- Sensor and node debug tabs

## Where to Look Next

- [architecture.md](architecture.md) — how the three compute planes fit
  together and how mapping actually works
- [../DESIGN_SPEC.md](../DESIGN_SPEC.md) — the full functional requirements
  this simulation work is validating against
