#!/usr/bin/env python3
"""Generate the front-bumper collision/visual meshes as watertight, convex
solids: a circular segment (an outer arc closed by a single straight chord)
extruded in Z, one per 90-degree bumper half.

Why a convex circular segment, not a curved shell: the fix this script
supports (M1 bumper geometry, see README.md + the accepted-stabilization
audit's P1-1) needs the bumper's OUTER contact surface to follow the body's
true circular radius instead of the old single-flat-box-per-90-deg-half
wedge. A thin curved SHELL (inner arc + outer arc) is concave on its inner
face, which risks silent failure if the physics engine's mesh-collision path
falls back to a convex hull or convex decomposition for a non-static body (a
real risk in gz-sim/DART - see bugs/bumper-contact-sensor-silent.md for how
badly an untested assumption about this sensor's collision handling bit this
repo before). A circular segment (arc + one chord, no inner arc) is
mathematically convex for any arc under 180 degrees, so it is immune to that
failure mode by construction - what the physics engine uses is exactly what
was authored, convex-hull fallback or not. This keeps the fix inside
"function over form": the OUTER surface (what touches obstacles) is a close
polygonal approximation of the true circle at radius R_outer; only the
interior (never a contact surface, always inside the body's silhouette) is
flat instead of hollow.

Output: bumper_left.stl (absolute angle 0..+90 deg) and bumper_right.stl
(absolute angle -90..0 deg), both authored directly in base_link-frame
coordinates (no local half-rotation needed - see rambla.urdf.xacro's
bumper_half macro, which mounts each at the base_link origin with identity
rotation). Re-run this script and commit the output whenever the geometry
constants below change; keep them in sync with params.xacro (xacro cannot
parameterize an offline-generated mesh file).
"""

import math
import os

# Keep in sync with rambla_description/urdf/params.xacro.
BASE_DIAMETER = 0.349  # base_diameter
BUMPER_THICKNESS = 0.010  # bumper_thickness (radial); outer face ~5mm proud
BUMPER_HEIGHT = 0.050  # bumper_height

BODY_RADIUS = BASE_DIAMETER / 2.0
OUTER_RADIUS = BODY_RADIUS + BUMPER_THICKNESS / 2.0  # matches the old box's
                                                      # outer-face radius
HALF_HEIGHT = BUMPER_HEIGHT / 2.0

# Facets across the 90-degree arc. 24 segments = 3.75 deg/facet, an order of
# magnitude finer than the ~2-facet wedge this replaces.
SEGMENTS = 24

OUTPUT_DIR = os.path.dirname(os.path.abspath(__file__))


def _arc_points(start_deg, end_deg):
    """Outer-radius points along the arc, base_link-frame XY, start..end deg."""
    pts = []
    for i in range(SEGMENTS + 1):
        t = i / SEGMENTS
        ang = math.radians(start_deg + t * (end_deg - start_deg))
        pts.append((OUTER_RADIUS * math.cos(ang), OUTER_RADIUS * math.sin(ang)))
    return pts


def _facet(f, v1, v2, v3):
    """Write one STL facet with an outward normal from the winding order."""
    ux, uy, uz = (v2[0] - v1[0], v2[1] - v1[1], v2[2] - v1[2])
    vx, vy, vz = (v3[0] - v1[0], v3[1] - v1[1], v3[2] - v1[2])
    nx, ny, nz = (uy * vz - uz * vy, uz * vx - ux * vz, ux * vy - uy * vx)
    norm = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
    nx, ny, nz = (nx / norm, ny / norm, nz / norm)
    f.write(f"  facet normal {nx:.6f} {ny:.6f} {nz:.6f}\n")
    f.write("    outer loop\n")
    for v in (v1, v2, v3):
        f.write(f"      vertex {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}\n")
    f.write("    endloop\n")
    f.write("  endfacet\n")


def _side_quad(f, top_p, bot_p, top_q, bot_q):
    """One outward-facing vertical quad wall between boundary points p and q
    (p -> q along the arc or the closing chord, top_* at +h/2, bot_* at
    -h/2). Winding verified by cross-product sign against the known outward
    radial direction near angle 0 - do not reorder without re-checking."""
    _facet(f, top_p, bot_p, bot_q)
    _facet(f, top_p, bot_q, top_q)


def write_bumper_stl(path, start_deg, end_deg):
    """Circular segment (arc + closing chord) extruded from -h/2 to +h/2.

    The 2D cross-section is a convex polygon: the arc vertices in order,
    closed by a single straight edge from the last arc vertex back to the
    first (the chord) - no vertex at the center, no inner arc. A prism over
    a convex polygon is convex, so this solid is convex regardless of facet
    count or which axis it's viewed from.
    """
    arc = _arc_points(start_deg, end_deg)
    top = [(x, y, HALF_HEIGHT) for x, y in arc]
    bot = [(x, y, -HALF_HEIGHT) for x, y in arc]
    n = len(arc)

    with open(path, "w") as f:
        f.write(f"solid {os.path.splitext(os.path.basename(path))[0]}\n")

        # Top cap (fan from vertex 0), normal +Z -> CCW looking from +Z.
        for i in range(1, n - 1):
            _facet(f, top[0], top[i], top[i + 1])

        # Bottom cap (fan from vertex 0), normal -Z -> reversed winding.
        for i in range(1, n - 1):
            _facet(f, bot[0], bot[i + 1], bot[i])

        # Side faces along the arc edges (i, i+1), i = 0..n-2.
        for i in range(n - 1):
            _side_quad(f, top[i], bot[i], top[i + 1], bot[i + 1])

        # Closing chord side face (edge n-1 -> 0).
        _side_quad(f, top[n - 1], bot[n - 1], top[0], bot[0])

        f.write(f"endsolid {os.path.splitext(os.path.basename(path))[0]}\n")


def main():
    left_path = os.path.join(OUTPUT_DIR, "bumper_left.stl")
    right_path = os.path.join(OUTPUT_DIR, "bumper_right.stl")
    write_bumper_stl(left_path, 0.0, 90.0)
    write_bumper_stl(right_path, -90.0, 0.0)
    print(f"wrote {left_path}")
    print(f"wrote {right_path}")
    print(f"body_radius={BODY_RADIUS:.4f} outer_radius={OUTER_RADIUS:.4f} "
          f"half_height={HALF_HEIGHT:.4f} segments={SEGMENTS}")


if __name__ == "__main__":
    main()
