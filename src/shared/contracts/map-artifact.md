# Map-Artifact Contract

Canonical shape of a map artifact: the output a SLAM burst-compute job
(RTAB-Map, on Modal) writes back after processing an observation batch.
Formalizes part of `DESIGN_SPEC.md` INT-004. Consolidates prose previously
in `.claude/internal-docs/compute/slam/CLAUDE.md` — that file now points
here rather than restating it.

---

## Job I/O shape

```
input:
  observation_batch          (rosbag2/MCAP — see observation-batch.md)
  base_map (optional)        (for incremental remap of an existing space)

output:
  map_vNNN/
```

`map_vNNN` (e.g. `map_v001`) is the versioned output directory RTAB-Map's
Modal job writes per mapping run, monotonically numbered — see Versioning
below.

---

## Artifact contents

An artifact is not a dense point cloud — RTAB-Map runs in **LiDAR +
monocular RGB mode** (not RGB-D; no depth camera exists in sim or is
committed to on real hardware, `DESIGN_SPEC.md` SNS-003/SNS-007). It
produces a sparse visual-feature graph for loop closure, plus a 2D
occupancy grid from the LiDAR.

| Abstract role | Concrete filename | Contents |
|---|---|---|
| Graph database | `rtabmap.db` | RTAB-Map's own sparse graph/database (visual features + poses), not a dense point cloud. |
| Occupancy grid | `map_map.pgm` | 2D occupancy grid, for localization/navigation. Produced by `rtabmap-reprocess -g2`, which is **PGM-only** — no YAML sidecar with resolution/origin. |
| Processing report | `processing_report.json` | What ran: timings, exit codes, node/loop-closure counts parsed from RTAB-Map's own log output. |

**Known gap:** `rtabmap-reprocess -g2`'s PGM-only export means there is no
Nav2-style `nav2_map_server`-compatible sidecar (resolution/origin YAML)
today. Producing one is a distinct, out-of-scope future step if a Nav2-style
consumer needs it later — not assumed by this contract.

---

## Versioning & compatibility

- **`map_vNNN` is monotonically versioned** per space/job lineage (e.g.
  `map_v001`, `map_v002`, ...) — a fresh mapping run, or an incremental
  remap against a `base_map` input, produces the next version, never
  overwrites a prior one in place.
- **Reprocessing an old observation batch with a newer RTAB-Map version must
  remain possible.** Because the observation batch (input) and the map
  artifact (output) are versioned independently, and the batch's own format
  is frozen by `observation-batch.md`, a batch recorded today should still
  be replayable through `rtabmap-reprocess` after a future RTAB-Map upgrade
  without redesigning either contract — this mirrors the "mapping as
  compilation" framing in `.claude/internal-docs/compute/slam/research.md`.
- **Compatibility declaration/checking**: not yet implemented. Today,
  compatibility is implicit (a given `map_vNNN/` is only known-valid against
  the RTAB-Map version and observation-batch format that produced it, judged
  by `processing_report.json`'s recorded run info). A formal
  declared/checked compatibility mechanism is future work, tracked with the
  persistent-state backend below.
- **Explicitly out of scope for this contract**: the durable persistent-state
  backend for job/artifact/version metadata, and the full artifact
  lifecycle (retries, failure handling, rollback, how the robot discovers a
  new artifact is ready). See `DESIGN_SPEC.md` OQ-014 — decision gate
  README M11. This contract codifies the artifact's *format*, not that
  backend or lifecycle.
