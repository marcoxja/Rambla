#!/usr/bin/env python3
"""Validate a map_vNNN artifact directory (map.yaml + occupancy-grid image).

Usage:
    python3 scripts/validate_map.py --map-dir ~/ros2_ws/maps/map_v001

Meant to run wherever the map directory lives (Mac, rambla-vm, eventually the
Pi). For a remote host with no copy of this file, pipe it over SSH stdin
instead of scp'ing it into the target's own git checkout (same pattern as
sample_resources.py):

    ssh rambla-vm python3 - --map-dir ~/ros2_ws/maps/map_v001 \\
        < scripts/validate_map.py

Same structural checks as src/compute/slam/modal_app.py's _verify_map_yaml
(M5 Phase 1) -- re-run here (M5 Phase 2, via scripts/vm.sh's rambla_map_fetch/
rambla_map_activate) after modal volume get + scp, in case the transfer
itself corrupted or truncated the artifact. map_saver_cli's output format is
a flat key: value YAML (image/resolution/origin/negate/occupied_thresh/
free_thresh, no nesting) so this is hand-parsed rather than requiring
pyyaml on hosts that may not have it (e.g. rambla-vm).

Also prints a coverage report (M7 map-coverage checkpoint, MAP_COVERAGE_PLAN.md
Workstream C): free/occupied/unknown pixel counts, free area in m^2, and a
connected-free-component count via a cheap flood fill -- "how many distinct
explored areas", a proxy for room count. This is informational only (does not
affect the pass/fail exit code): the reach_goal/obstacle_avoid coverage bars
are judged by a human/agent reading these numbers against the map, not
hardcoded here, since the bar is specific to what a given M7 phase needs.
PGM parsing is hand-rolled (raw P5, no numpy/Pillow) for the same
dependency-free-over-SSH-stdin reason as the rest of this file.
"""

import argparse
import os
import sys

FREE = 1
OCCUPIED = 2
UNKNOWN = 0

# map_server/map_saver_cli defaults, used only if map.yaml omits these
# fields (real map.yaml files always write them explicitly). 0.196 is not
# an arbitrary choice -- it's the exact normalized value of the mid-gray
# "unknown" pixel (205) that map_saver_cli writes, so an unknown pixel lands
# precisely on the free/unknown boundary rather than inside either bucket.
DEFAULT_FREE_THRESH = 0.196
DEFAULT_OCCUPIED_THRESH = 0.65


def _parse_map_yaml_fields(yaml_path):
    fields = {}
    with open(yaml_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, _, value = line.partition(":")
            fields[key.strip()] = value.strip()
    return fields


def verify_map_yaml(yaml_path):
    if not os.path.exists(yaml_path):
        raise RuntimeError(f"{yaml_path} not found")
    fields = _parse_map_yaml_fields(yaml_path)
    resolution = fields.get("resolution")
    origin = fields.get("origin")
    image_name = fields.get("image")
    if not resolution or not origin:
        raise RuntimeError(
            f"{yaml_path} missing resolution/origin: parsed fields={fields}"
        )
    if not image_name:
        raise RuntimeError(f"{yaml_path} missing image field: parsed fields={fields}")
    image_path = os.path.join(os.path.dirname(yaml_path), image_name)
    if not os.path.exists(image_path):
        raise RuntimeError(
            f"map image {image_path} referenced by {yaml_path} not found"
        )
    image_size_bytes = os.path.getsize(image_path)
    if image_size_bytes < 1024:
        raise RuntimeError(
            f"map image {image_path} is implausibly small ({image_size_bytes} bytes)"
        )
    return {
        "yaml_path": yaml_path,
        "image_path": image_path,
        "image_size_bytes": image_size_bytes,
        "resolution": resolution,
        "origin": origin,
    }


def read_pgm_raster(image_path):
    """Read a raw (binary) P5 PGM fully into memory: width, height, maxval,
    and the row-major pixel bytes. map_saver_cli only ever emits P5, so
    other formats (P2 ASCII, PNG) are out of scope for the coverage scan
    (verify_map_yaml's structural check above still passes for those --
    this is additional, not a replacement)."""
    with open(image_path, "rb") as f:
        data = f.read()
    if not data.startswith(b"P5"):
        raise ValueError(
            f"unsupported map image format for coverage scan: {data[:2]!r} "
            "(expected raw P5 PGM)"
        )
    idx = 2
    tokens = []
    while len(tokens) < 3:
        while idx < len(data) and data[idx:idx + 1].isspace():
            idx += 1
        if idx < len(data) and data[idx:idx + 1] == b"#":
            while idx < len(data) and data[idx:idx + 1] != b"\n":
                idx += 1
            continue
        start = idx
        while idx < len(data) and not data[idx:idx + 1].isspace():
            idx += 1
        if idx == start:
            raise RuntimeError(f"{image_path}: truncated PGM header")
        tokens.append(data[start:idx])
    idx += 1  # single whitespace byte separating maxval from the raster, per PGM spec
    width, height, maxval = (int(t) for t in tokens)
    pixels = data[idx:idx + width * height]
    if len(pixels) != width * height:
        raise RuntimeError(
            f"{image_path}: raster truncated (expected {width * height} "
            f"bytes, got {len(pixels)})"
        )
    return width, height, maxval, pixels


def classify_pixels(width, height, maxval, pixels, negate, free_thresh, occupied_thresh):
    """ROS map_server/map_saver_cli convention: normalize each pixel to
    [0,1] (inverted unless negate), then occupied if > occupied_thresh, free
    if < free_thresh, else unknown."""
    classes = bytearray(width * height)
    free_count = occupied_count = unknown_count = 0
    for i, p in enumerate(pixels):
        normalized = (p / maxval) if negate else ((maxval - p) / maxval)
        if normalized > occupied_thresh:
            classes[i] = OCCUPIED
            occupied_count += 1
        elif normalized < free_thresh:
            classes[i] = FREE
            free_count += 1
        else:
            unknown_count += 1
    return classes, free_count, occupied_count, unknown_count


def flood_fill_component_sizes(classes, width, height, target):
    """Cheap 4-connected flood fill over pixels == target, iterative (no
    recursion depth risk on large maps). Returns component sizes in px,
    largest first -- a proxy for 'how many distinct explored areas', not a
    real room segmentation (a wide-open doorway merges two rooms into one
    component; a completely unmapped gap between two rooms splits them into
    two even though the plan below might join them -- read alongside the
    raster, not as ground truth)."""
    visited = bytearray(width * height)
    sizes = []
    for start in range(width * height):
        if classes[start] != target or visited[start]:
            continue
        size = 0
        stack = [start]
        visited[start] = 1
        while stack:
            idx = stack.pop()
            size += 1
            x, y = idx % width, idx // width
            if x > 0:
                nidx = idx - 1
                if not visited[nidx] and classes[nidx] == target:
                    visited[nidx] = 1
                    stack.append(nidx)
            if x < width - 1:
                nidx = idx + 1
                if not visited[nidx] and classes[nidx] == target:
                    visited[nidx] = 1
                    stack.append(nidx)
            if y > 0:
                nidx = idx - width
                if not visited[nidx] and classes[nidx] == target:
                    visited[nidx] = 1
                    stack.append(nidx)
            if y < height - 1:
                nidx = idx + width
                if not visited[nidx] and classes[nidx] == target:
                    visited[nidx] = 1
                    stack.append(nidx)
        sizes.append(size)
    sizes.sort(reverse=True)
    return sizes


def compute_coverage(yaml_path):
    fields = _parse_map_yaml_fields(yaml_path)
    resolution = float(fields["resolution"])
    negate = int(float(fields.get("negate", "0"))) != 0
    free_thresh = float(fields.get("free_thresh", DEFAULT_FREE_THRESH))
    occupied_thresh = float(fields.get("occupied_thresh", DEFAULT_OCCUPIED_THRESH))
    image_path = os.path.join(os.path.dirname(yaml_path), fields["image"])

    width, height, maxval, pixels = read_pgm_raster(image_path)
    classes, free_count, occupied_count, unknown_count = classify_pixels(
        width, height, maxval, pixels, negate, free_thresh, occupied_thresh)

    total = width * height
    free_components = flood_fill_component_sizes(classes, width, height, FREE)
    px_area_m2 = resolution * resolution

    return {
        "width_px": width,
        "height_px": height,
        "resolution": resolution,
        "width_m": round(width * resolution, 2),
        "height_m": round(height * resolution, 2),
        "free_px": free_count,
        "occupied_px": occupied_count,
        "unknown_px": unknown_count,
        "free_pct": round(100.0 * free_count / total, 1),
        "occupied_pct": round(100.0 * occupied_count / total, 1),
        "unknown_pct": round(100.0 * unknown_count / total, 1),
        "free_m2": round(free_count * px_area_m2, 1),
        "free_component_count": len(free_components),
        "free_component_sizes_m2": [
            round(size * px_area_m2, 1) for size in free_components[:5]
        ],
    }


def print_coverage_report(coverage):
    print(
        f"COVERAGE: {coverage['width_m']} x {coverage['height_m']} m "
        f"({coverage['width_px']}x{coverage['height_px']} px @ "
        f"{coverage['resolution']:.3f} m/px)"
    )
    print(
        f"  free:     {coverage['free_m2']:7.1f} m^2 "
        f"({coverage['free_px']} px, {coverage['free_pct']}%)"
    )
    print(
        f"  occupied: {'':7} "
        f"({coverage['occupied_px']} px, {coverage['occupied_pct']}%)"
    )
    print(
        f"  unknown:  {'':7} "
        f"({coverage['unknown_px']} px, {coverage['unknown_pct']}%)"
    )
    print(
        f"  free components: {coverage['free_component_count']} "
        f"(top sizes m^2: {coverage['free_component_sizes_m2']})"
    )


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--map-dir", required=True,
        help="directory containing map.yaml (e.g. ~/ros2_ws/maps/map_v001)",
    )
    args = parser.parse_args()

    map_dir = os.path.expanduser(args.map_dir)
    yaml_path = os.path.join(map_dir, "map.yaml")
    try:
        result = verify_map_yaml(yaml_path)
    except RuntimeError as exc:
        print(f"INVALID: {exc}", file=sys.stderr)
        sys.exit(1)

    print(
        f"OK: {result['yaml_path']} valid "
        f"(resolution={result['resolution']}, origin={result['origin']}, "
        f"image={result['image_path']}, {result['image_size_bytes']} bytes)"
    )

    # Coverage is informational only -- a failure here (e.g. an unsupported
    # image format) does not flip the exit code, since the structural check
    # above is what rambla_map_activate gates the atomic symlink swap on.
    try:
        coverage = compute_coverage(yaml_path)
        print_coverage_report(coverage)
    except Exception as exc:
        print(f"COVERAGE: skipped ({exc})", file=sys.stderr)


if __name__ == "__main__":
    main()
