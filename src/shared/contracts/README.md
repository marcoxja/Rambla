# `src/shared/contracts/` — canonical interface contracts

This directory holds the source-independent interface contracts that
normalize the robot's ROS 2 topics/frames, the observation batch format, and
the map-artifact format across sim and (future) real hardware. Authored as
part of Roadmap **M2 — System boundary contract foundation** (`README.md`).

Files:

- [`robot-interface.md`](robot-interface.md) — ROS runtime contract: topic
  names, message types, TF/frame ownership, `/cmd_vel` arbitration,
  control-authority (`DESIGN_SPEC.md` INT-001, INT-002, INT-003, INT-005)
- [`observation-batch.md`](observation-batch.md) — observation-batch
  contract (`DESIGN_SPEC.md` INT-004)
- [`map-artifact.md`](map-artifact.md) — map-artifact contract, including the
  versioning/compatibility policy shared with `observation-batch.md`
  (`DESIGN_SPEC.md` INT-004)

These are now the canonical owners for the areas above — see the ownership
map in `.claude/internal-docs/documentation-workflow/CLAUDE.md` (Rule 7).
Automated conformance-checking (INT-006) is implemented as the M2
launch_testing smoke check —
`src/simulation/rambla_sim/test/test_smoke_topics.py` — which asserts
liveness, frame_ids, and rates against `robot-interface.md`.
