"""Pure, rclpy-independent reactive wander decision logic.

Deliberately not a general-purpose navigation/planning system: this only
needs to keep the robot moving through the house world long enough to
cross at least one doorway and produce a usable mapping bag (per
.claude/internal-docs/compute/slam/plans/real-map-plan-2026-07-06.md's
"~30-60s, crosses one doorway" recording target) - not to reach a
specific goal or build a map itself. No map dependency, matching
DESIGN_SPEC.md's decision to keep the Nav2/navigation-stack question
(OQ-003) unresolved for now.

Strategy: drive forward; when the front arc gets tight, turn in place
toward whichever side (left/right arc) has more clearance until the front
is clear again, then resume forward. Always publishes to cmd_vel_raw, not
cmd_vel directly - rambla_safety is the sole final arbiter of cmd_vel and
can still override this if something is missed (e.g. sensor noise, a
diagonal obstacle this simple front/left/right split doesn't catch).
"""
import math

DEFAULT_FORWARD_SPEED = 0.15
DEFAULT_TURN_SPEED = 0.5
# Larger than rambla_safety's stop_distance_m (0.35m) so this node steers
# away before the safety node's hard override would ever need to engage
# during normal wandering - the safety node remains the last-resort net,
# not the primary steering mechanism.
DEFAULT_TURN_THRESHOLD_M = 0.6
DEFAULT_FRONT_ARC_DEG = 40
DEFAULT_SIDE_ARC_DEG = 70
# See rambla_safety.safety_monitor.DEFAULT_MIN_VALID_RANGE_M - same
# self-detection artifact (LiDAR seeing the robot's own front bumper at
# ~0.24-0.27m), confirmed live in sim, applies here too.
DEFAULT_MIN_VALID_RANGE_M = 0.3


class WanderBehavior:

    def __init__(self,
                 forward_speed=DEFAULT_FORWARD_SPEED,
                 turn_speed=DEFAULT_TURN_SPEED,
                 turn_threshold_m=DEFAULT_TURN_THRESHOLD_M,
                 front_arc_deg=DEFAULT_FRONT_ARC_DEG,
                 side_arc_deg=DEFAULT_SIDE_ARC_DEG,
                 min_valid_range_m=DEFAULT_MIN_VALID_RANGE_M):
        self._forward_speed = forward_speed
        self._turn_speed = turn_speed
        self._turn_threshold_m = turn_threshold_m
        self._front_arc_deg = front_arc_deg
        self._side_arc_deg = side_arc_deg
        self._min_valid_range_m = min_valid_range_m
        self._turning_direction = None  # +1 left, -1 right, None = not turning

    def next_command(self, ranges, angle_min, angle_increment):
        """Returns (linear, angular) given the latest LaserScan fields."""
        front_min = self._arc_min(ranges, angle_min, angle_increment, 0.0, self._front_arc_deg)

        if front_min is None or front_min >= self._turn_threshold_m:
            self._turning_direction = None
            return self._forward_speed, 0.0

        if self._turning_direction is None:
            left_min = self._arc_min(
                ranges, angle_min, angle_increment, 90.0, self._side_arc_deg)
            right_min = self._arc_min(
                ranges, angle_min, angle_increment, -90.0, self._side_arc_deg)
            left_clearance = left_min if left_min is not None else float('inf')
            right_clearance = right_min if right_min is not None else float('inf')
            self._turning_direction = 1 if left_clearance >= right_clearance else -1

        return 0.0, self._turning_direction * self._turn_speed

    def _arc_min(self, ranges, angle_min, angle_increment, center_deg, half_width_deg):
        if not ranges or angle_increment == 0.0:
            return None
        center_rad = center_deg * math.pi / 180.0
        half_width_rad = half_width_deg * math.pi / 180.0
        relevant = []
        for i, r in enumerate(ranges):
            if r != r or r < self._min_valid_range_m:  # nan, invalid, or self-detection
                continue
            angle = angle_min + i * angle_increment
            wrapped = (angle + math.pi) % (2 * math.pi) - math.pi
            diff = (wrapped - center_rad + math.pi) % (2 * math.pi) - math.pi
            if -half_width_rad <= diff <= half_width_rad:
                relevant.append(r)
        return min(relevant) if relevant else None
