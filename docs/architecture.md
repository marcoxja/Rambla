# Architecture

Rambla is layered intelligence, not a monolithic controller. A single
principle governs the split between layers:

> The advisory layer suggests goals. The local robot decides how — and
> whether it's safe — to act on them.

Nothing about cognition, memory, or planning is allowed to compromise the
robot's ability to stay safe and keep functioning on its own.

## Three Compute Planes

Rambla's software spans three compute planes, each with a different job,
a different lifecycle, and a different tolerance for being unavailable.

### 1. Local real-time (robot)

Runs on a Raspberry Pi 5 under ROS2 Jazzy. This plane is safety-critical
and always-on, and it never depends on the network to do its job. It owns:

- Emergency stop and collision avoidance
- Sensor fusion (EKF) for a filtered pose estimate
- Localization — lightweight scan-matching against a cached map
- Drive arbitration
- Navigation (Nav2 is the leading candidate, not yet committed)

If every other plane disappeared, this is the layer that keeps the robot
from hurting itself, a person, a pet, or the furniture.

### 2. Burst compute (Modal — current direction)

Ephemeral, on-demand compute for heavy, infrequent work — there is no
always-on server in this model. A job starts when triggered, does its work,
writes a versioned artifact, and exits.

Today this plane runs SLAM map assembly with RTAB-Map: loop closure and
occupancy-grid optimization, the computationally expensive part of mapping
that has no business running continuously on the robot's own compute
budget. Future heavy jobs — semantic labeling, and possibly LLM or
decision-layer inference — are expected to follow the same burst pattern:
spin up, process, persist a result, spin down.

### 3. Persistent state (backend undecided; Supabase is a candidate)

Durable records that need to outlive any single compute run: job metadata,
map version history, and — looking further out — long-term place/social
memory and the robot's activity and cognition history (perceptions,
thoughts, goals, decisions, state changes, and incidents; see the
`EVT-*` requirements in [DESIGN_SPEC.md](../DESIGN_SPEC.md)).

The specific backend for this plane isn't committed. Supabase is the
current leading candidate, not a settled decision.

## How Mapping Works (Record-Then-Process)

Mapping is deliberately **not** a live, connection-dependent process. It
follows a record-then-process flow:

1. The robot records a deliberate mapping run as an observation batch.
2. The batch is uploaded.
3. A Modal burst job assembles the map (RTAB-Map: loop closure +
   occupancy-grid optimization) and writes a versioned map artifact.
4. The artifact and its metadata are persisted.
5. The robot retrieves the latest artifact and localizes against it
   **locally** — scan-matching happens on the Pi, not over the network.

The robot never blocks on a live connection to build or use a map. Mapping
is something that happens in discrete, versioned steps; localization is
something that happens continuously and locally against whatever map
artifact is currently cached.

```
        Local real-time plane (always-on, safety owns this)
    ┌─────────────────────────────────────────────────────┐
    │  Raspberry Pi 5 · ROS2 Jazzy                         │
    │  e-stop · collision avoidance · EKF · localization   │
    │  drive arbitration · navigation (Nav2 candidate)     │
    └───────────────┬─────────────────────▲───────────────┘
                     │ observation batch    │ versioned map
                     │ (deliberate map run)  │ artifact (cached, read-only)
                     ▼                      │
    ┌──────────────────────────┐   ┌───────┴──────────────┐
    │  Burst compute (Modal)   │   │  Persistent state    │
    │  RTAB-Map SLAM assembly  │──▶│  (Supabase candidate)│
    │  ephemeral · on-demand   │   │  job + map metadata  │
    └──────────────────────────┘   └──────────────────────┘
     record-then-process, not a live bridge
```

The robot uploads a batch, a burst job assembles the map, the artifact and
its metadata are persisted, and the robot pulls the artifact back down to
localize against it on its own compute. Safety-critical behavior never sits
downstream of any of that — it stays entirely in the local plane.

## Degraded Mode

If burst compute or the network is unavailable:

- **Reflex safety is unaffected.** E-stop and collision avoidance never
  depended on remote compute in the first place.
- **Navigation and dock-return keep working** against the last cached map
  artifact.
- The robot idles or falls back to a safe default rather than freezing or
  waiting on a connection that may not come back.

Nothing about losing network access should ever translate into losing
control of the robot.

## What's Settled vs. Open

**Settled:**

- SLAM package: RTAB-Map
- Middleware: ROS2 Jazzy
- Robot compute: Raspberry Pi 5
- Simulation middleware: Gazebo Harmonic

**Open / candidate, not committed:**

- Navigation stack — Nav2 is the leading candidate (see `OQ-003` in
  [DESIGN_SPEC.md](../DESIGN_SPEC.md))
- Persistent-state backend — Supabase is a candidate (see `OQ-014`)
- Hosted cognition/LLM stack — undecided
- Event/cognition history storage and schema — undecided (see `OQ-015`)

See [DESIGN_SPEC.md](../DESIGN_SPEC.md) for the full set of open questions
and phased requirements, and [PROJECT_VISION.md](../PROJECT_VISION.md) for
the narrative vision this architecture is in service of.
