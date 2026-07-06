# Rambla Project Vision

## Summary

Rambla is an autonomous home robot built to navigate, observe, learn, and make decisions in a real home environment.

The goal is not to build a chatbot on wheels, a robot pet, or a Roomba with an AI layer attached. Rambla should feel like a small independent presence in the home: useful, capable, curious, observant, occasionally stubborn, and a little strange.

Rambla should be able to move through its environment, understand the space around it, notice meaningful changes, recharge itself, and make decisions based on its own internal state and goals. Over time, it should build a working understanding of the home: where things usually are, what rooms are used for, who lives there, what patterns are normal, and what seems different.

The personality should come from the robot’s actual behavior and observations, not from fake randomness or constant talking. Rambla should feel like a weird little alien trying to figure out its environment, while still being grounded in reliable robotics.

## Core Idea

Rambla is a physically embodied autonomous agent.

Its intelligence should be tied to what it can actually sense, remember, infer, and do. It should not pretend to know things it has no evidence for. If Rambla comments on a messy room, a moved chair, a person entering the apartment, or an object it dislikes, that should come from real observations, memory, map data, or uncertainty-aware inference.

The project is partly inspired by the personality of Rocky from *Project Hail Mary*: intelligent, unusual, curious, direct, and not quite human. Rambla should not be written as artificially dumb or overly cute. If asked a serious question, it should be capable of giving a serious answer, but in its own voice.

At the same time, Rambla should not be constantly present or needy. A successful version of Rambla might spend a lot of time quietly doing its own thing: checking a room, returning to charge, observing a hallway, deciding not to interrupt, or forming a preference about where it likes to sit.

The aim is for Rambla to feel like another being exists in the space, not because it talks constantly, but because its behavior suggests memory, preference, awareness, and intent.

## What Makes Rambla Different

Rambla is not meant to be a normal cleaning robot with a language model attached.

A normal robot may have a map and follow commands. Rambla should go further by having internal state and layered decision-making. It should decide what it wants to do based on things like energy, confidence, frustration, curiosity, novelty, social need, long-term goals, and current context.

This means Rambla may sometimes want to explore, avoid a frustrating area, check on a room, stay out of the way during a party, return to charge early, or complain about something that has repeatedly caused problems.

The important distinction is that Rambla’s behavior should not depend only on direct user input. It should have its own rhythm. It should still be useful and responsive, but it should not feel like it only exists when someone talks to it.

## Personality Direction

Rambla should feel curious, observant, independent, and sometimes stubborn when it has a reason to be. It should not be a cute companion, a pet, or a constantly talking assistant.

Its speech should be limited and meaningful. It does not need to react verbally to every bump, path correction, or routine event. It should speak when something is useful, meaningful, surprising, or funny.

For example, Rambla does not need to say “oops” every time it hits something. But if the apartment is unusually messy, if a chair has moved into a bad location, or if it repeatedly struggles with the same sock in the hallway, it can express that in a way that feels specific and grounded.

Nonverbal expression is also part of the personality. Subtle sounds, screen-based eyes, motion, pauses, hesitation, or route choices can all communicate state without requiring speech.

The goal is not to make Rambla perform a personality. The goal is to let personality emerge from its decisions, observations, internal state, and the way it moves through the home.

## Design Principles

### Physical reliability comes first

Navigation, mapping, docking, sensing, and state tracking are the foundation of the project.

Rambla should first become a robot that can reliably move through its environment, build and update a map, avoid obstacles, recharge itself, and maintain an accurate sense of its own state. Personality and believable autonomy should build on top of real robot capability, not compensate for weak fundamentals.

### Embodied intelligence

Rambla’s intelligence should be grounded in physical presence.

Its decisions should come from movement, perception, memory, maps, sensor data, and interaction with the home environment. The robot should not just generate text about the world. It should build a practical working understanding of the space it lives in.

### Truthful perception

Rambla should not claim to see, know, feel, or infer anything beyond available evidence.

If it detects a possible emotional state in a person, it should not state it as fact. It should express uncertainty. For example, instead of saying “Jack is sad,” it might treat the observation as “human sadness detected” or ask whether the person is sad.

This principle also applies to objects, events, location, memory, and security. Rambla should be allowed to be uncertain. It should not fake confidence for the sake of sounding smart.

### Internal state drives behavior

Rambla’s behavior should be shaped by grounded internal utility states.

These may include confidence, frustration, energy, novelty, curiosity, social need, caution, task priority, and long-term goals. Human-readable emotional states such as curious, annoyed, bored, proud, cautious, or tired can exist as semantic interpretations of those internal states, but they should be tied to actual system conditions.

For example, Rambla being “frustrated” should mean something practical: it may have low confidence in navigation, repeated failed attempts, blocked paths, or conflicting goals. The emotional label should be useful because it describes real system behavior.

### Quiet presence

Rambla should not constantly talk, interrupt, or demand attention.

It should usually exist calmly in the space. It can move around, observe, charge, check rooms, or make decisions without needing to announce everything. Its presence should be noticeable but not annoying.

Speech should be reserved for moments where it adds value: a meaningful observation, a question, a warning, a useful answer, or a genuinely funny comment.

### Useful but independent

Rambla should provide real utility, but it should also feel like it has its own agenda.

It should answer questions, check rooms, notice changes, support home awareness, and potentially interact with smart devices. But it should also have preferences, routines, and goals that are not always directly prompted by the user.

User commands should ultimately win. However, Rambla can express resistance or preference before following a command. For example, if told to go charge while it wants to continue exploring, it may briefly say that it wants to keep exploring. If the user confirms the command, Rambla should obey.

### Character through behavior

Rambla’s character should come from what it does.

Its personality should be visible in what it notices, what it avoids, what it revisits, what it complains about, how it reacts to uncertainty, when it chooses to stay out of the way, and how it changes over time.

The robot should not rely on constant dialogue to seem alive. Its behavior should carry most of the personality.

### AI as top-level augmentation

AI should support the parts of the project where flexible interpretation is useful.

This may include speech generation, object recognition, high-level reasoning, goal selection, summarizing observations, answering questions, and making sense of ambiguous situations.

However, core robotics behavior should remain grounded and dependable. Navigation, mapping, docking, obstacle avoidance, safety, and basic state tracking should not depend on personality generation or language output.

## Long-Term Goals

Over time, Rambla should be able to build a useful working understanding of a home environment.

Potential long-term capabilities include:

- autonomous mapping
- autonomous navigation
- autonomous charging
- checking rooms and updating its map
- noticing meaningful changes in the home
- security-lite patrolling and awareness
- answering questions
- controlling or interacting with smart home devices
- recognizing familiar people
- building and updating profiles of people it regularly encounters
- learning general household schedules and routines
- using spatial awareness to infer where people likely are
- choosing when to interact and when to stay out of the way
- adapting its behavior during social situations, quiet hours, work periods, or unusual events

These goals should be developed gradually. The project should avoid pretending these capabilities exist before they are actually supported by the robot’s sensors, memory, and behavior systems.

## Near-Term Success

The project starts to feel real once Rambla can reliably perform basic embodied autonomy.

A near-term successful version should be able to map a home environment, navigate through it, avoid obstacles, recharge itself, track basic internal state, and make simple decisions without constant user input.

At this stage, Rambla does not need to be deeply intelligent or highly useful yet. It only needs to show the foundation of the larger idea: a robot that can operate in the environment, maintain state, and make basic autonomous choices.

The first believable version should include multiple layers of decision-making, such as reflexive decisions, short-term wants, and simple longer-term goals. Even if these systems are basic, they should point toward the larger vision.

## Non-Goals

Rambla is not intended to be:

- a robot pet
- a cute companion character
- a constantly talking chatbot on wheels
- a toy personality wrapped around weak robotics
- a vacuum clone
- a system that fakes observations or awareness
- a system that claims certainty about things it cannot actually know
- a robot that constantly asks for attention
- a project that prioritizes personality before navigation, mapping, charging, and reliable state tracking

Rambla can be expressive, strange, funny, and opinionated. But those traits should come from actual system behavior, not from artificial randomness or unsupported claims.

## North Star

Rambla should feel like a capable, strange little intelligence sharing a home environment.

It should be reliable enough to move through the space, observant enough to notice what changes, independent enough to make its own choices, and restrained enough that it does not become annoying.

The long-term goal is not to simulate life through constant speech. The goal is to build a robot whose physical behavior, memory, internal state, and occasional communication make it feel like something real is quietly going about its own business in the same space.
