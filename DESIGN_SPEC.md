# Rambla Design Specification

## Purpose

This document describes the high-level design requirements for Rambla.

The goal is to define what Rambla should generally support, not how each part must be implemented. This spec should guide development without locking the project into a specific architecture, tech stack, hardware model, ROS structure, behavior tree format, database schema, or server design too early.

Rambla should be developed in a way that keeps the core idea intact: a reliable autonomous home robot with grounded perception, internal state, decision-making, and restrained personality.

## Design Scope

Rambla is a low-profile wheeled home robot based on a robot vacuum style form factor. Its physical design is conceptually based on the OOMWOO-style open robot vacuum direction, but without the cleaning apparatus.

Rambla is not intended to have arms, legs, or humanoid locomotion. It should operate as a compact mobile robot that can move through a home environment, build and update a map, return to its dock, observe changes, and eventually develop higher-level behavior and utility.

Simulation may be used during development, but simulation should follow the real robot requirements. This document does not define the simulation environment in detail.

## Phase Definitions

| Phase | Meaning |
|---|---|
| P0 | Foundation required early for the robot to operate safely and reliably. |
| P1 | Basic autonomy, decision-making, internal state, and interaction. |
| P2 | Believable behavior, personality, memory, and richer expression. |
| P3 | Home utility expansion after core autonomy is reliable. |
| Future | Long-term or optional capability not required for early versions. |

## Functional Requirements

| ID | Area | Requirement | Phase | Notes |
|---|---|---|---|---|
| PHY-001 | Physical Design | Rambla shall use a low-profile wheeled base similar to a robot vacuum form factor. | P0 | No legs, arms, or humanoid body plan. |
| PHY-002 | Physical Design | Rambla shall omit cleaning hardware from the robot vacuum-style design. | P0 | The project is not a vacuum clone. |
| PHY-003 | Physical Design | Rambla shall be designed around practical movement in a home environment. | P0 | Furniture, thresholds, rugs, clutter, and tight spaces should be considered. |
| PHY-004 | Physical Design | Rambla shall keep exact component models as placeholders until hardware is selected. | P0 | Specific parts can be filled in later. |
| NAV-001 | Navigation | Rambla shall autonomously navigate through a home environment. | P0 | Exact navigation stack is not specified here. |
| NAV-002 | Navigation | Rambla shall avoid known and detected obstacles while moving. | P0 | Safety and reliability are more important than personality. |
| NAV-003 | Navigation | Rambla shall support point-to-point movement within its known environment. | P0 | Exact command format is deferred. |
| NAV-004 | Navigation | Rambla shall handle navigation failure in a detectable and recoverable way. | P0 | Recovery behavior can be designed later. |
| NAV-005 | Navigation | Rambla shall support basic route planning between known locations. | P1 | Implementation details are intentionally open. |
| MAP-001 | Mapping | Rambla shall build a map of the home environment. | P0 | Exact SLAM or mapping method is not defined here. |
| MAP-002 | Mapping | Rambla shall update its map as the home environment changes. | P0 | Map updates should be grounded in observations. Incremental/merged mapping runs are README M7.5; artifact-level lifecycle (prune/rollback) is README M11.5. |
| MAP-003 | Mapping | Rambla shall recognize rooms or meaningful areas at a high level. | P1 | Room labels and area semantics can evolve later. First addressed at README M7.5 (room/zone structure). |
| MAP-004 | Mapping | Rambla shall distinguish between stable structure and movable clutter where possible. | P1 | Useful for change detection and navigation confidence. |
| MAP-005 | Mapping | Rambla shall maintain awareness of its own estimated location. | P0 | Uncertainty should be represented where possible. |
| CHG-001 | Charging | Rambla shall be able to return to its charging dock. | P0 | Required for basic autonomy. |
| CHG-002 | Charging | Rambla shall track battery or power state. | P0 | Exact thresholds are deferred. |
| CHG-003 | Charging | Rambla shall decide to charge before power becomes unsafe. | P0 | Charging should not depend only on user command. |
| CHG-004 | Charging | Rambla shall treat successful docking as a core reliability requirement. | P0 | Docking should be developed before personality polish. |
| CHG-005 | Charging | Rambla shall support safe fallback behavior when low on power. | P0 | Minimum expected behavior is to attempt safe return to dock. |
| SIM-001 | Simulation | Rambla may use simulation to test movement, sensing, mapping, and behavior. | P1 | Simulation details should be specified elsewhere. |
| SIM-002 | Simulation | Simulation should reflect real robot requirements rather than define them. | P1 | The physical robot remains the source of truth. |
| SIM-003 | Simulation | Rambla's primary residential simulation world should follow the apartment-world enhancement design brief. | P1 | A larger single-floor apartment layout with higher-fidelity navigation geometry and lightweight visual/semantic differentiation for perception testing — implemented as `house.sdf` in `rambla_sim`, selectable via `world:=house`; see the README M1 milestone. |

## Sensing Requirements

| ID | Area | Requirement | Phase | Notes |
|---|---|---|---|---|
| SNS-001 | Sensing | Rambla shall use placeholder sensor definitions until exact parts are selected. | P0 | Hardware model names should be filled in later. |
| SNS-002 | Sensing | Rambla shall support basic navigation sensing suitable for an OOMWOO-inspired wheeled base. | P0 | Likely includes LiDAR, drive feedback, bumper/contact sensing, and dock-related sensing. |
| SNS-003 | Sensing | Rambla shall include LiDAR or equivalent spatial sensing for mapping and navigation. | P0 | Exact sensor model is deferred. |
| SNS-004 | Sensing | Rambla shall include wheel or drive feedback where needed for movement estimation. | P0 | Exact odometry approach is deferred. |
| SNS-005 | Sensing | Rambla shall include contact or collision detection where needed for safe operation. | P0 | Could include bump sensors or equivalent. |
| SNS-006 | Sensing | Rambla shall include sensing needed to identify or approach its charging dock. | P0 | Exact docking method is deferred. |
| SNS-007 | Vision | Rambla shall include camera capability for future visual awareness. | P1 | Exact camera model is deferred. |
| SNS-008 | Vision | Rambla shall eventually support object recognition or visual classification. | P2 | Should be grounded in actual camera input. |
| SNS-009 | Audio | Rambla shall include microphone capability for voice input or environmental audio awareness. | P1 | Exact microphone model is deferred. |
| SNS-010 | Audio | Rambla may support multiple microphones for approximate sound direction. | P2 | Useful for relative sound origin but not required at the foundation stage. |
| SNS-011 | Sensing | Rambla shall distinguish between confirmed observations and uncertain detections. | P1 | Supports truthful perception. |
| SNS-012 | Sensing | Rambla shall preserve sensor uncertainty where practical. | P1 | Exact representation is deferred. |

## Perception and World Model Requirements

| ID | Area | Requirement | Phase | Notes |
|---|---|---|---|---|
| PER-001 | Perception | Rambla shall convert raw sensor input into useful observations. | P1 | Exact perception pipeline is not specified. |
| PER-002 | Perception | Rambla shall avoid treating uncertain detections as confirmed facts. | P1 | Required for grounded behavior. |
| PER-003 | Perception | Rambla shall be able to notice meaningful changes in the environment. | P2 | Example: moved furniture, unusual clutter, new object, blocked route. |
| PER-004 | Perception | Rambla shall be able to recognize recurring navigation problems. | P2 | Example: a sock or chair that repeatedly blocks a route. |
| PER-005 | Perception | Rambla shall eventually support familiar person recognition. | P3 | Exact method and privacy design are deferred. |
| PER-006 | Perception | Rambla shall eventually support basic activity or context awareness. | P3 | Example: party, quiet hours, usual dinner time. |
| MEM-001 | Memory / World Model | Rambla shall maintain a working model of the home environment. | P1 | Should include map, rooms, known objects, and recent changes where possible. |
| MEM-002 | Memory / World Model | Rambla shall remember recurring features of the environment. | P2 | Example: common clutter locations, preferred paths, frustrating areas. |
| MEM-003 | Memory / World Model | Rambla shall eventually maintain simple profiles of familiar humans. | P3 | Profiles should be based on observed patterns and explicit information. |
| MEM-004 | Memory / World Model | Rambla shall eventually learn general household routines. | P3 | Example: workdays, bedtime patterns, dinner locations. |
| MEM-005 | Memory / World Model | Rambla shall represent uncertainty in memory where needed. | P2 | Avoid false certainty about people, objects, and events. |
| MEM-006 | Memory / World Model | Rambla shall allow important remembered information to be reviewed or changed later. | Future | Exact interface is deferred. |

## Decision-Making Requirements

| ID | Area | Requirement | Phase | Notes |
|---|---|---|---|---|
| DEC-001 | Decision-Making | Rambla shall support autonomous decision-making without constant user input. | P1 | Required for the project identity. |
| DEC-002 | Decision-Making | Rambla shall support layered decision-making at a conceptual level. | P1 | Reflexive behavior, tasks, wants, goals, and high-level reasoning. |
| DEC-003 | Decision-Making | Rambla shall prioritize safety-critical behavior over personality or curiosity. | P0 | Safety and reliability come first. |
| DEC-004 | Decision-Making | Rambla shall be able to choose basic tasks on its own. | P1 | Example: explore, check area, charge, idle, observe. |
| DEC-005 | Decision-Making | Rambla shall eventually support longer-term goals. | P2 | Example: improve map confidence, monitor recurring obstacle, learn routine. |
| DEC-006 | Decision-Making | Rambla shall be able to defer, stop, or change tasks when conditions change. | P1 | Example: low battery, blocked route, user command. |
| DEC-007 | Decision-Making | Rambla shall be able to reason about tradeoffs at a high level. | P2 | Exact mechanism is deferred. |
| DEC-008 | Decision-Making | Rambla shall not rely on language generation for core robotics decisions. | P0 | Movement and safety must remain dependable. |
| STA-001 | Internal State | Rambla shall maintain basic internal state. | P1 | Example: battery, current task, confidence, frustration, energy. |
| STA-002 | Internal State | Rambla shall use internal state to influence behavior. | P1 | State should affect decisions, not just be displayed. |
| STA-003 | Internal State | Rambla shall support utility-style states such as confidence, frustration, novelty, energy, curiosity, and social need. | P1 | Exact state model is deferred. |
| STA-004 | Internal State | Rambla may map internal utility states to human-readable emotional labels. | P2 | Example: curious, annoyed, tired, cautious. |
| STA-005 | Internal State | Rambla’s emotional labels shall be grounded in actual system state. | P2 | No fake mood generation detached from behavior. |
| STA-006 | Internal State | Rambla shall allow internal state to affect speech, expression, and task choice. | P2 | Exact implementation is deferred. |

## Activity, Cognition, and Operational History Requirements

| ID | Area | Requirement | Phase | Notes |
|---|---|---|---|---|
| EVT-001 | Event History | Rambla shall maintain a structured, typed historical record of meaningful operational, behavioral, perceptual, and cognitive events. | P1 | The exact database, event bus, schema, and storage architecture are deferred. |
| EVT-002 | Event History | Recorded events shall identify their event type and subtype where applicable. | P1 | Example types may include compute job, robot activity, perception, thought, goal, decision, state change, and incident. |
| EVT-003 | Event History | Recorded events shall include relevant timing information. | P1 | May include creation time, start time, update time, completion time, or duration depending on event type. |
| EVT-004 | Event History | Event types that represent ongoing work or conditions shall support type-appropriate status values. | P1 | Status models may differ by type; a compute job, robot activity, goal, and incident should not be forced into one universal status model. |
| EVT-005 | Event History | Rambla shall retain recent event history for a configurable period of time. | P1 | Retention may vary by event type, volume, importance, and storage constraints rather than using one fixed duration for all records. |
| EVT-006 | Event History | Rambla shall support different retention classes for high-volume ephemeral events and higher-value historical events. | P2 | Important goals, decisions, incidents, and semantic changes may be retained longer than routine perceptions or transient thoughts. |
| EVT-007 | Event History | Recorded events should support importance, salience, severity, or equivalent prioritization metadata where useful. | P2 | This may influence retention, UI visibility, alerting, memory promotion, or later analysis. |
| EVT-008 | Event Correlation | Rambla shall support associating related events across different event types. | P2 | Example: a perception may lead to a thought, state change, goal, decision, robot activity, and compute job that all relate to the same occurrence. |
| EVT-009 | Compute Jobs | Rambla shall record meaningful background and external compute jobs. | P1 | Examples include SLAM processing, map optimization, semantic processing, ML inference, observation upload, artifact generation, and synchronization. |
| EVT-010 | Compute Jobs | Compute job records shall support relevant execution metadata. | P1 | May include queued time, start time, end time, status, execution location, processing stage, failure state, input references, and output references. |
| EVT-011 | Robot Activities | Rambla shall record meaningful robot activities and tasks. | P1 | Examples include patrolling, mapping, wandering, scanning, talking, navigating, docking, charging, sleeping, waiting, and investigating. |
| EVT-012 | Robot Activities | Robot activity records shall distinguish intended activity from incidents or failures affecting that activity. | P1 | Example: navigation may become blocked while a separate stuck incident is recorded. |
| EVT-013 | Perceptions | Rambla shall record meaningful externally representable perceptions or observations where useful. | P1 | Examples include object detection, human detection, unusual sound, lighting classification, geometry mismatch, or possible environmental change. Related: PER-001, PER-002, PER-003, SNS-011, SNS-012. |
| EVT-014 | Perceptions | Perception records shall preserve uncertainty or confidence where applicable. | P1 | A possible chair movement should remain distinguishable from a confirmed change. Related: PER-001, PER-002, PER-003, SNS-011, SNS-012. |
| EVT-015 | Thoughts / Interpretations | Rambla shall support recording meaningful short-lived interpretations derived from perception, memory, internal state, or context. | P2 | Examples include “chair may have moved,” “room seems unusually quiet,” or “this obstacle may be temporary.” |
| EVT-016 | Thoughts / Interpretations | Rambla shall avoid treating low-level processing noise as meaningful thought history. | P2 | The system should preserve useful externally representable cognitive events rather than every intermediate computation. |
| EVT-017 | Goals | Rambla shall record meaningful current, candidate, deferred, and historical goals. | P1 | Goals represent what Rambla may want or intend to do, including goals not ultimately selected for action. Related: DEC-004, DEC-005, DEC-006. |
| EVT-018 | Goals | Goal records shall support type-appropriate lifecycle status. | P1 | Example statuses may include candidate, active, deferred, blocked, satisfied, abandoned, expired, or superseded. Exact values are deferred. Related: DEC-004, DEC-005, DEC-006. |
| EVT-019 | Decisions | Rambla shall support recording meaningful decisions where preserving the selected choice or reasoning context is useful. | P2 | Decisions should be recorded selectively rather than for every low-level control action. Related: DEC-005, DEC-006, DEC-007. |
| EVT-020 | Decisions | Decision records should preserve relevant relationships between considered goals, selected outcomes, alternatives, and contributing factors where available. | P2 | This supports later explanation of why one course of action was selected over another. Related: DEC-005, DEC-006, DEC-007. |
| EVT-021 | State Changes | Rambla shall record meaningful changes to internal behavioral, emotional, motivational, or confidence-related state. | P1 | Examples include confidence decrease, curiosity increase, social need increase, frustration increase, or energy-related change. Related: STA-001, STA-002, STA-003. |
| EVT-022 | State Changes | State-change history should prioritize meaningful transitions over excessive logging of insignificant value fluctuations. | P1 | Exact thresholds, aggregation, and sampling behavior are deferred. Related: STA-001, STA-002, STA-003. |
| EVT-023 | Incidents | Rambla shall record meaningful incidents, anomalies, degradations, and recovery events. | P1 | Examples include becoming stuck, sensor failure, localization degradation, server unavailability, map synchronization failure, or compute job timeout. |
| EVT-024 | Incidents | Incident records shall support type-appropriate severity, status, timing, and recovery information where useful. | P1 | Exact incident taxonomy is deferred. |
| EVT-025 | UI / Observability | Stored event history shall be available for user interfaces, dashboards, status indicators, timelines, and other operational views. | P1 | Example uses include background-process dashboards, current activity displays, recent thought streams, goal views, and incident history. Related: CTL-004, CTL-011. |
| EVT-026 | UI / Observability | Different event types shall be independently queryable so interfaces can present focused views without processing one undifferentiated log stream. | P1 | Example: background compute jobs should be viewable separately from perceptions or robot goals. Related: CTL-004, CTL-011. |
| EVT-027 | Automation / Triggers | Stored or newly emitted events may be used as inputs to triggers, alerts, automation, behavior, or later processing. | P2 | Example: a failed mapping job, persistent environmental change, or repeated incident may trigger follow-up action. |
| EVT-028 | LLM Context | Rambla shall support selectively surfacing relevant event history to an LLM or other reasoning system to inform grounded responses. | P2 | The system should retrieve relevant records rather than indiscriminately sending the full event history. Related: DEC-008, HUM-005, HUM-006, SAF-004. |
| EVT-029 | LLM Context | Rambla shall support efficient structured querying of event history by authorized reasoning systems. | P2 | Queries may filter by time, event type, status, related entity, activity, goal, location, correlation context, or other useful metadata. Related: DEC-008, HUM-005, HUM-006, SAF-004. |
| EVT-030 | LLM Explanation | Rambla shall be able to use relevant historical records to answer questions about its own recent behavior, goals, observations, state changes, and actions. | P2 | Example: answering “Why did you start patrolling?” using stored evidence from preceding perceptions, goals, decisions, and activities. Related: DEC-008, HUM-005, HUM-006, SAF-004. |
| EVT-031 | LLM Explanation | Explanations of Rambla’s behavior shall be grounded in available recorded history and shall not invent missing reasons. | P2 | If the causal history is incomplete, Rambla should express uncertainty rather than fabricate a rationale. Related: DEC-008, HUM-005, HUM-006, SAF-004. |
| EVT-032 | LLM Explanation | Rambla should distinguish direct recorded causes from retrospective inference when explaining past behavior. | P2 | Example: “I started patrolling because an unexpected sound triggered a security check” differs from “I think that may have been why.” Related: DEC-008, HUM-005, HUM-006, SAF-004. |
| EVT-033 | LLM Explanation | Event history should support reconstructing meaningful causal chains across perception, interpretation, state, goal, decision, activity, and compute processing where available. | P2 | This supports questions such as why an activity began, why a goal was deferred, or what caused a state change. Related: DEC-008, HUM-005, HUM-006, SAF-004. |
| EVT-034 | Access and Scope | The system shall allow event history exposed to interfaces or reasoning systems to be filtered according to relevance and intended use. | P2 | Operational telemetry, user-facing thought streams, and LLM context may require different subsets of the same underlying history. |
| EVT-035 | Reliability | Failure to record non-critical event history shall not block immediate safety-critical robot behavior. | P0 | Event history is important for observability and reasoning but should not become a single point of failure for movement safety. Related: SAF-005, LOC-001, LOC-004. |
| EVT-036 | Architecture | This specification shall not require all event types to use one physical storage system or one retention policy. | P1 | High-volume perceptions, durable goals, compute-job metadata, and long-term incidents may eventually use different storage strategies. |

## Human Interaction Requirements

| ID | Area | Requirement | Phase | Notes |
|---|---|---|---|---|
| HUM-001 | Human Interaction | Rambla shall accept user commands. | P1 | Exact command interface is deferred. |
| HUM-002 | Human Interaction | Rambla shall support a high-level command and override concept. | P1 | User commands ultimately win. Exact hierarchy is deferred. |
| HUM-003 | Human Interaction | Rambla may express resistance or preference before obeying a command. | P2 | Example: it wants to keep exploring but follows after confirmation. |
| HUM-004 | Human Interaction | Rambla shall not become intentionally difficult to control. | P1 | Stubbornness should not undermine usability or safety. |
| HUM-005 | Human Interaction | Rambla shall be able to answer questions when supported by available knowledge or systems. | P2 | Should not fake certainty. |
| HUM-006 | Human Interaction | Rambla shall distinguish between what it knows, infers, and does not know. | P2 | Required for truthful perception. |
| HUM-007 | Human Interaction | Rambla shall eventually recognize familiar humans where supported by sensors and memory. | P3 | Exact identity system is deferred. |
| HUM-008 | Human Interaction | Rambla shall eventually adapt behavior to social context. | P3 | Example: stay out of the way during a party. |
| HUM-009 | Human Interaction | Rambla shall avoid constant interruption. | P1 | Presence should be restrained. |

## Developer Control & Debug Interface Requirements

| ID | Area | Requirement | Phase | Notes |
|---|---|---|---|---|
| CTL-001 | Control Interface | Rambla shall provide a web-based control and debug interface usable against either the simulated or real robot. | P1 | Must be reachable from the developer's own machine/network, not only from a browser running on the sim VM itself — see CTL-008 for network reachability and CTL-009 for public internet hosting. |
| CTL-002 | Control Interface | The interface shall support manual driving via on-screen virtual joysticks (translation and rotation). | P1 | Depends on `/cmd_vel_teleop` (already exists), arbitrated into `/cmd_vel` by `rambla_safety`'s AUTO/MANUAL control authority. Pairs with the README "Basic teleop" milestone. |
| CTL-003 | Control Interface | The interface shall display a live camera feed from the robot, with manual-drive controls overlaid on top of it. | P1 | Depends only on `/camera/image_raw` (already exists). No SLAM/navigation dependency. |
| CTL-004 | Control Interface | The interface shall provide debug views for raw sensor data, node output, and other diagnostic metrics, organized into navigable tabs. | P1 | Depends only on existing topics/nodes. Directly addresses the "no visual debug tooling" gap noted in `robot/pre-slam-audit.md`; buildable before SLAM. |
| CTL-005 | Control Interface | The interface shall provide a HUD-style overlay showing a top-down view of the current SLAM map with a marker for the robot's live estimated position. | P2 | Hard dependency: requires MAP-001 (SLAM producing a usable map) and MAP-005 (live position estimate) to exist first. Do not build before then — there is nothing to display. |
| CTL-006 | Control Interface | The interface shall provide buttons for programmed movement actions, including at minimum: return to dock, navigate to a named room, and initiate a new full map scan. | P2 | Depends on CHG-001 (dock return), NAV-005 (route planning) + MAP-003 (room recognition) for goto-room, and MAP-001 (SLAM) for rescan. Each button should only be added once its underlying capability exists — do not stub non-functional buttons. |
| CTL-007 | Control Interface | The interface shall present a layout appropriate to its viewport: a detailed, information-dense layout on desktop/web, and a simplified layout on mobile optimized for on-the-go control and at-a-glance map/camera awareness. | P1 | Applies to whichever of CTL-002 through CTL-006 are implemented at a given time — this is a cross-cutting layout constraint, not a separate feature phase. |
| CTL-008 | Control Interface | The interface shall be reachable from the developer's home/dev network, not only from the sim VM's local console. | P1 | Moved up from a later phase: the sim already runs on a VM, so "local-only" hosting would otherwise require solving VM console/SSH access anyway — serving it over the network is no more work and unblocks day-to-day use immediately. |
| CTL-009 | Control Interface | Rambla's control interface shall eventually be securely reachable from the public internet, not just the developer's home/dev network, for the final design. | P3 | Distinct from CTL-008: this is public exposure with real auth/TLS/hardening, not just off-VM reachability. Deferred until the interface has functionality worth hardening. |
| CTL-010 | Control Interface | The interface shall default to a read-only view — showing the live camera feed and map HUD while the robot autonomously decides its own behavior — and shall require an explicit "take over control" action before manual joysticks and programmed movement buttons are surfaced or take effect. | P2 | Hard dependency: not meaningful until DEC-004 (autonomous task choice) and DEC-006 (deferring/changing tasks) exist — there is no autonomous behavior to take over from until then. Taking over should cancel/suspend whatever the robot was doing (see DEC-006). |
| CTL-011 | Control Interface | The interface shall show a small overlay describing the robot's current activity in plain terms (e.g. "mapping," "navigating to kitchen," "returning to dock"). | P2 | Same dependency as CTL-010 — needs DEC-004-style task selection to exist to have anything true to report. Should reflect actual internal state/task (per STA-001/STA-002), not a fabricated status. |

## Expression and Speech Requirements

| ID | Area | Requirement | Phase | Notes |
|---|---|---|---|---|
| EXP-001 | Expression | Rambla shall support restrained expression. | P2 | Expression should be meaningful, not constant. |
| EXP-002 | Speech | Rambla shall speak only when useful, meaningful, funny, or contextually appropriate. | P2 | Avoid constant narration. |
| EXP-003 | Speech | Rambla shall not verbally react to every routine bump, correction, or minor event. | P2 | Prevent annoying behavior. |
| EXP-004 | Speech | Rambla may comment on meaningful environmental changes. | P2 | Example: unusually messy room, recurring blocked route. |
| EXP-005 | Speech | Rambla’s speech shall be grounded in observations, memory, or internal state. | P2 | No random personality filler. |
| EXP-006 | Speech | Rambla shall express uncertainty when appropriate. | P2 | Example: “possible sadness detected” rather than “human is sad.” |
| EXP-007 | Nonverbal Expression | Rambla may use subtle sounds to communicate state. | P2 | Sounds should be audible but not annoying. |
| EXP-008 | Visual Expression | Rambla may use screen-based eyes or visual indicators to communicate state. | P2 | Exact display hardware is deferred. |
| EXP-009 | Expression | Rambla’s expression should reflect current task and internal state. | P2 | Example: focused, curious, cautious, low energy. |

## Interface & Portability Requirements

| ID | Area | Requirement | Phase | Notes |
|---|---|---|---|---|
| INT-001 | Interface Contracts | Application layers shall consume normalized contracts independent of whether their source is simulation or physical hardware. | P1 | Canonical owner: `src/shared/contracts/robot-interface.md`. |
| INT-002 | Interface Contracts | ROS topics and transforms exposed above the platform adapter shall use stable canonical names, message types, and frame conventions. | P1 | Formalizes the single-writer TF and `/cmd_vel` arbitration invariants already established informally in `architecture/CLAUDE.md`. Canonical owner: `src/shared/contracts/robot-interface.md`. |
| INT-003 | Interface Contracts | Simulation-only naming or transport artifacts shall not propagate into higher-level consumers. | P1 | Example: sim-only topic remaps or frame_id shims must not leak into localization/navigation/control-panel code paths. See `src/shared/contracts/robot-interface.md`'s sim-shim boundary section. |
| INT-004 | Interface Contracts | Observation-batch and map-artifact formats shall be versioned and compatibility-checkable. | P1 | Canonical owners: `src/shared/contracts/observation-batch.md`, `src/shared/contracts/map-artifact.md`. Related: MAP-001, OQ-014. |
| INT-005 | Interface Contracts | Actuator commands shall pass through one canonical arbitration boundary. | P0 | Already implemented: `rambla_safety` is the sole `/cmd_vel` publisher, arbitrating `/cmd_vel_raw` (autonomous) against `/cmd_vel_teleop` (manual). See `architecture/CLAUDE.md`. |
| INT-006 | Interface Contracts | Contract conformance shall be automatically testable. | P1 | Implemented: `src/simulation/rambla_sim/test/test_smoke_topics.py` asserts topic liveness, frame_ids, and rates (`/odometry/filtered`, `/scan`, camera topics) plus the `odom→base_footprint` TF, against `robot-interface.md`. See README M2 and M8's later reliability-hardening extension. |

## Local vs External Processing Requirements

| ID | Area | Requirement | Phase | Notes |
|---|---|---|---|---|
| LOC-001 | Local Processing | Rambla shall maintain basic safety behavior without external server availability. | P0 | Required fallback capability. |
| LOC-002 | Local Processing | Rambla shall be able to avoid obstacles without external server availability. | P0 | Core safety behavior should be local or otherwise dependable. |
| LOC-003 | Local Processing | Rambla shall be able to attempt return-to-dock behavior without external server availability. | P0 | Especially important at low battery. |
| LOC-004 | Local Processing | Rambla shall not require external processing for immediate movement safety. | P0 | Exact local/server split is deferred. |
| LOC-005 | External Processing | Rambla may use an external server for heavier processing. | P1 | Example: high-level reasoning, memory, speech, model inference. |
| LOC-006 | External Processing | Rambla shall degrade safely if external processing is unavailable. | P0 | Degraded mode should favor safety and docking. |
| LOC-007 | External Processing | The project shall later define which functions must run locally and which may be offloaded. | P1 | This spec does not decide that split. |

## Reliability and Safety Requirements

| ID | Area | Requirement | Phase | Notes |
|---|---|---|---|---|
| SAF-001 | Safety | Rambla shall prioritize safe movement over task completion. | P0 | Safety comes before goals or personality. |
| SAF-002 | Safety | Rambla shall avoid causing damage to itself, people, pets, furniture, or the home. | P0 | Exact safety mechanisms are deferred. |
| SAF-003 | Safety | Rambla shall stop, slow down, or recover when navigation confidence is too low. | P0 | Exact confidence threshold is deferred. |
| SAF-004 | Safety | Rambla shall avoid making unsupported claims about safety, people, or security events. | P1 | Truthful uncertainty is required. |
| SAF-005 | Reliability | Rambla shall make failures visible to the system. | P0 | Failures should not be silently ignored. |
| SAF-006 | Reliability | Rambla shall handle blocked paths gracefully. | P0 | Exact recovery behavior is deferred. |
| SAF-007 | Reliability | Rambla shall maintain awareness of whether it is operating normally or in a degraded mode. | P1 | Useful for local/server fallback. |
| SAF-008 | Reliability | Rambla shall treat autonomous charging as a reliability requirement, not a personality feature. | P0 | Docking is foundational. |
| SAF-009 | Reliability | Rambla shall avoid continuing risky behavior for the sake of curiosity or character. | P0 | Personality must not override safety. |
| SAF-010 | Reliability | Rambla shall log or preserve enough information to diagnose important failures later. | P1 | Exact logging system is deferred. |

## Future Utility Requirements

| ID | Area | Requirement | Phase | Notes |
|---|---|---|---|---|
| UTL-001 | Home Awareness | Rambla shall eventually check rooms and report meaningful changes. | P3 | Depends on reliable mapping and perception. |
| UTL-002 | Home Awareness | Rambla shall eventually maintain awareness of normal household patterns. | P3 | Example: usual locations, routines, quiet hours. |
| UTL-003 | Security-Lite | Rambla shall eventually support basic security-lite awareness. | P3 | Not a substitute for a security system. |
| UTL-004 | Security-Lite | Rambla may patrol occasionally when household members are away. | P3 | Requires reliable navigation and clear safety rules. |
| UTL-005 | Security-Lite | Rambla may listen for unexpected sounds when docked or idle. | P3 | Exact detection method is deferred. |
| UTL-006 | Security-Lite | Rambla shall avoid overclaiming security conclusions. | P3 | Example: “unexpected sound detected,” not “intruder confirmed” without evidence. |
| UTL-007 | Smart Home | Rambla may eventually interact with smart home devices. | Future | Not an early requirement. |
| UTL-008 | Smart Home | Rambla may eventually act as a mobile smart home interface. | Future | Exact integrations are deferred. |
| UTL-009 | Question Answering | Rambla may answer questions using local state, memory, and external knowledge where available. | P2 | Should remain clear about uncertainty and source of knowledge. |
| UTL-010 | Social Awareness | Rambla may eventually adjust behavior based on social situations. | P3 | Example: staying away during parties. |

## Non-Goals

| ID | Area | Non-Goal | Phase | Notes |
|---|---|---|---|---|
| NG-001 | Physical Design | Rambla is not intended to have arms or legs. | P0 | Wheeled base only. |
| NG-002 | Physical Design | Rambla is not intended to be humanoid. | P0 | Avoid humanoid design assumptions. |
| NG-003 | Product Direction | Rambla is not a vacuum clone. | P0 | Cleaning is not the purpose. |
| NG-004 | Personality | Rambla is not a robot pet. | P1 | Avoid cute pet framing. |
| NG-005 | Personality | Rambla is not a constantly talking companion. | P1 | Quiet presence matters. |
| NG-006 | AI Behavior | Rambla should not fake observations or awareness. | P1 | All claims should be grounded. |
| NG-007 | AI Behavior | Rambla should not use artificial stupidity as personality. | P1 | It should be capable and strange, not dumb. |
| NG-008 | Architecture | This document does not define exact implementation architecture. | P0 | Architecture can be designed later. |
| NG-009 | Hardware | This document does not lock exact sensor or part models. | P0 | Hardware can be filled in later. |
| NG-010 | Simulation | This document does not define the full simulation environment. | P1 | Simulation planning belongs elsewhere. |

## Open Questions

| ID | Area | Question | Phase | Notes |
|---|---|---|---|---|
| OQ-001 | Hardware | Which exact sensors and component models will Rambla use? | P0 | Fill in after hardware selection. |
| OQ-002 | Hardware | Which OOMWOO components can be reused directly? | Resolved | No OOMWOO-One/Kaia package adopted wholesale — only the sensor-level `<gz_frame_id>` technique was harvested into Rambla's own sim/description stack. Kaia's firmware (ESP32) + micro-ROS telemetry decode remain the leading real-hardware bridge candidate, deferred to hardware selection (M12/M13). See `architecture/CLAUDE.md`'s foundation-decision section. |
| OQ-003 | Navigation | What navigation stack or approach should be used? | Resolved | Both halves resolved. Mapping: RTAB-Map (see `compute/slam/CLAUDE.md`). Navigation: **Nav2** — decided 2026-07-15 at README M6. M5 already deployed `nav2_amcl`, `nav2_map_server`, and `nav2_lifecycle_manager`; the arbitration contract and USABLE-gating rule already presume Nav2's architecture; no ROS2-Jazzy-compatible alternative ecosystem exists. Full rationale in `.claude/internal-docs/robot/navigation/CLAUDE.md`. |
| OQ-004 | Mapping | How should the map represent rooms, objects, uncertainty, and change? | P1 | Needs separate design work. First addressed at README M7.5. |
| OQ-005 | Docking | How exactly should Rambla detect and approach its dock? | P0 | May depend on inherited design and hardware. Whatever the mechanism, it must work independent of localization/map confidence — dock-return is the intended recovery path when AMCL is LOST, not just a normal-operation goal. |
| OQ-006 | Decision-Making | What is the right structure for reflexes, tasks, wants, and goals? | P1 | Needs separate behavior design. |
| OQ-007 | Local vs Server | Which functions must run locally, and which can be offloaded? | P1 | Resolved for mapping, still open elsewhere: SLAM map assembly runs as an ephemeral burst-compute job (Modal), not an always-on server — the Pi records an observation batch, uploads it, and later retrieves a versioned map artifact to localize against. Safety reflexes stay local, never delegated (see INT-005). See `architecture/CLAUDE.md`'s Runtime and Compute Placement table and `compute/slam/CLAUDE.md`. Which *other* future workloads (semantic labeling, AI/decision-layer inference) get the same local/burst split is still undecided. Persistent-state backend, observation package format, and job/artifact lifecycle/versioning are separate open questions — see OQ-014. |
| OQ-008 | Memory | What should Rambla remember, forget, summarize, or expose for review? | P2 | Needs separate memory design. |
| OQ-009 | Expression | What screen, sounds, or physical cues should communicate Rambla’s state? | P2 | Hardware and UX decision. |
| OQ-010 | Human Override | What exact command and override hierarchy should Rambla use? | P1 | This spec only requires that one exists. |
| OQ-011 | Security-Lite | What counts as meaningful home-awareness or security-lite detection? | P3 | Must avoid false certainty. |
| OQ-012 | Smart Home | Which smart home systems should Rambla eventually control or observe? | Future | Not an early decision. |
| OQ-013 | Performance | What CPU/RAM/power budget should each Pi-side subsystem target? | P1 | Needs real Pi 5 hardware to finalize; see `robot/resource-budget/CLAUDE.md` for the interim framework. |
| OQ-014 | Burst Compute / Persistent State | Where should job records, observation metadata, and map version history live, and what format should an observation batch use? | P1 | Not decided — decision gate at README M11. The current Modal Volume (`rambla-slam-data`) used by the SLAM job is a *temporary* stand-in scoped to that milestone only, not a resolution of this question (see `compute/slam/CLAUDE.md`). Candidates and tradeoffs for the durable backend are recorded in `architecture/hosting-research.md` (e.g. Supabase for persistent state) and `compute/slam/research.md` (observation batch format). Applies beyond SLAM to any future burst-compute job (semantic labeling, AI/decision-layer inference). Lifecycle *implementation* (listing/pruning/rollback of map artifacts) follows at README M11.5, once this decision is made. |
| OQ-015 | Event History | Where should event and cognition history (perceptions, thoughts, goals, decisions, state changes, incidents) be stored, and how should it be structured? | P1 | Not decided. See EVT-001/EVT-036, which defer the exact database, event bus, schema, and storage architecture. Likely shares the persistent-state backend decision with OQ-014 (Supabase is a candidate there). |
