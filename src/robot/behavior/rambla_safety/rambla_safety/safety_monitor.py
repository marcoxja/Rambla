"""Pure, rclpy-independent collision-safety decision logic.

Kept as a plain class (no ROS message types, no rclpy) so it's unit
testable without a running node - same shape as rambla_control_panel's
DeadmanTimer. safety_node.py wraps this with rclpy subscriptions/timers.

Two independent trip conditions, either one overrides forward motion:
  - Bumper contact (either side): hard stop, contact already happened.
  - LiDAR front-arc range below STOP_DISTANCE_M: stop before contact.
Rotation-in-place is always allowed even while tripped, since turning
away is the recovery action and can't itself cause the same collision.
"""

# Ranges below this are the LiDAR seeing its own front bumper, not a real
# obstacle - confirmed live in sim (rambla-vm, 2026-07-08): base_scan sits
# lidar_center_offset=-0.0965m behind body center (params.xacro), so the
# bumper's outer face at the front (angle 0, where both bumper_half halves
# meet with no gap - see rambla.urdf.xacro) sits ~0.28m from the LiDAR
# origin, not the ~0.17m body radius alone. A handful of samples right at
# angle 0 read ~0.24-0.27m even in open room with nothing nearby, which
# would otherwise show as "blocked" at spawn. This is a sensor placement
# fact, not sensor noise - excluded here rather than fixed at the geometry
# level (out of scope for the safety/traversal nodes). Must stay below
# STOP_DISTANCE_M or the stop condition could never trigger.
DEFAULT_MIN_VALID_RANGE_M = 0.3
# Must be > DEFAULT_MIN_VALID_RANGE_M - real obstacles closer than this
# (but beyond the self-detection band above) trigger a stop.
DEFAULT_STOP_DISTANCE_M = 0.35
# Front arc spans the wrap-around point (angle 0) of a 360-sample,
# 0..2pi scan (REP-103: index 0 = angle_min = straight ahead, increasing
# counterclockwise) - see rambla_description/urdf/plugins.xacro's gpu_lidar
# config. +/-30 degrees either side of straight ahead.
DEFAULT_FRONT_ARC_DEG = 30


class SafetyMonitor:

    def __init__(self, stop_distance_m=DEFAULT_STOP_DISTANCE_M,
                 front_arc_deg=DEFAULT_FRONT_ARC_DEG,
                 min_valid_range_m=DEFAULT_MIN_VALID_RANGE_M):
        self._stop_distance_m = stop_distance_m
        self._front_arc_deg = front_arc_deg
        self._min_valid_range_m = min_valid_range_m
        self._bumper_left_contact = False
        self._bumper_right_contact = False
        self._min_front_range = None

    def on_scan(self, ranges, angle_min, angle_increment):
        self._min_front_range = self._min_front_arc_range(
            ranges, angle_min, angle_increment)

    def on_bumper_left(self, has_contact):
        self._bumper_left_contact = has_contact

    def on_bumper_right(self, has_contact):
        self._bumper_right_contact = has_contact

    def is_blocked(self):
        if self._bumper_left_contact or self._bumper_right_contact:
            return True
        if self._min_front_range is not None and self._min_front_range < self._stop_distance_m:
            return True
        return False

    def filter_cmd(self, linear, angular):
        """Given a commanded (linear, angular) velocity, return the safe
        velocity to actually publish. Forward motion is blocked while
        tripped; reverse and rotation are always passed through, since
        they're the recovery actions and can't drive further into
        whatever tripped the block."""
        if self.is_blocked() and linear > 0.0:
            return 0.0, angular
        return linear, angular

    def _min_front_arc_range(self, ranges, angle_min, angle_increment):
        if not ranges or angle_increment == 0.0:
            return None
        half_arc_rad = self._front_arc_deg * 3.141592653589793 / 180.0
        relevant = []
        for i, r in enumerate(ranges):
            if r != r or r < self._min_valid_range_m:  # nan, invalid, or self-detection
                continue
            angle = angle_min + i * angle_increment
            # Normalize to [-pi, pi] around 0 (straight ahead) so the
            # wrap-around at the 0/2pi seam is handled correctly.
            wrapped = (angle + 3.141592653589793) % (2 * 3.141592653589793) - 3.141592653589793
            if -half_arc_rad <= wrapped <= half_arc_rad:
                relevant.append(r)
        return min(relevant) if relevant else None
