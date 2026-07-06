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
| MAP-002 | Mapping | Rambla shall update its map as the home environment changes. | P0 | Map updates should be grounded in observations. |
| MAP-003 | Mapping | Rambla shall recognize rooms or meaningful areas at a high level. | P1 | Room labels and area semantics can evolve later. |
| MAP-004 | Mapping | Rambla shall distinguish between stable structure and movable clutter where possible. | P1 | Useful for change detection and navigation confidence. |
| MAP-005 | Mapping | Rambla shall maintain awareness of its own estimated location. | P0 | Uncertainty should be represented where possible. |
| CHG-001 | Charging | Rambla shall be able to return to its charging dock. | P0 | Required for basic autonomy. |
| CHG-002 | Charging | Rambla shall track battery or power state. | P0 | Exact thresholds are deferred. |
| CHG-003 | Charging | Rambla shall decide to charge before power becomes unsafe. | P0 | Charging should not depend only on user command. |
| CHG-004 | Charging | Rambla shall treat successful docking as a core reliability requirement. | P0 | Docking should be developed before personality polish. |
| CHG-005 | Charging | Rambla shall support safe fallback behavior when low on power. | P0 | Minimum expected behavior is to attempt safe return to dock. |
| SIM-001 | Simulation | Rambla may use simulation to test movement, sensing, mapping, and behavior. | P1 | Simulation details should be specified elsewhere. |
| SIM-002 | Simulation | Simulation should reflect real robot requirements rather than define them. | P1 | The physical robot remains the source of truth. |

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
| OQ-002 | Hardware | Which OOMWOO components can be reused directly? | P0 | Requires repo and BoM review. |
| OQ-003 | Navigation | What navigation stack or approach should be used? | P0 | Do not decide in this spec. |
| OQ-004 | Mapping | How should the map represent rooms, objects, uncertainty, and change? | P1 | Needs separate design work. |
| OQ-005 | Docking | How exactly should Rambla detect and approach its dock? | P0 | May depend on inherited design and hardware. |
| OQ-006 | Decision-Making | What is the right structure for reflexes, tasks, wants, and goals? | P1 | Needs separate behavior design. |
| OQ-007 | Local vs Server | Which functions must run locally, and which can be offloaded? | P1 | Safety-critical behavior should remain dependable. |
| OQ-008 | Memory | What should Rambla remember, forget, summarize, or expose for review? | P2 | Needs separate memory design. |
| OQ-009 | Expression | What screen, sounds, or physical cues should communicate Rambla’s state? | P2 | Hardware and UX decision. |
| OQ-010 | Human Override | What exact command and override hierarchy should Rambla use? | P1 | This spec only requires that one exists. |
| OQ-011 | Security-Lite | What counts as meaningful home-awareness or security-lite detection? | P3 | Must avoid false certainty. |
| OQ-012 | Smart Home | Which smart home systems should Rambla eventually control or observe? | Future | Not an early decision. |
