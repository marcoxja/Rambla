"""Pure, rclpy-independent collision-safety decision logic.

Kept as a plain class (no ROS message types, no rclpy) so it's unit
testable without a running node - same shape as rambla_control_panel's
DeadmanTimer, including the injected-clock pattern for the bumper contact
timeout below. safety_node.py wraps this with rclpy subscriptions/timers.

Two independent trip conditions, either one overrides forward motion:
  - Bumper contact (either side): hard stop, contact already happened.
  - LiDAR front-arc range below STOP_DISTANCE_M: stop before contact.
Rotation-in-place is always allowed even while tripped, since turning
away is the recovery action and can't itself cause the same collision.
"""
import time

# Historical note (resolved 2026-07-09, M1 bumper/sensor-placement fix):
# this floor used to mask a real forward self-return of ~0.24-0.27m at
# angle 0, confirmed live in sim on 2026-07-08. That was mislabeled at the
# time as "the LiDAR seeing its own front bumper" - direct geometric
# calculation later showed the bumper's z-band never reached the scan
# plane at all; the actual obstruction was the forward camera box, whose
# z-band straddled the old scan height exactly at that distance. Fixed at
# the source (rambla_description/urdf/params.xacro's laser_puck_offset
# raised so the scan plane clears the camera), not by filtering here -
# confirmed live post-fix: open-space front-arc minimum is ~2.9m, no
# near-range self-return. This floor now only exists as a generic guard
# against noise near the LiDAR's own physical range_min (0.1m per
# plugins.xacro's gpu_lidar <range><min>). Must stay below
# STOP_DISTANCE_M or the stop condition could never trigger.
DEFAULT_MIN_VALID_RANGE_M = 0.12
# Must be > DEFAULT_MIN_VALID_RANGE_M - real obstacles closer than this
# (but beyond the self-detection band above) trigger a stop.
DEFAULT_STOP_DISTANCE_M = 0.35
# Front arc spans the wrap-around point (angle 0) of a 360-sample,
# 0..2pi scan (REP-103: index 0 = angle_min = straight ahead, increasing
# counterclockwise) - see rambla_description/urdf/plugins.xacro's gpu_lidar
# config. +/-30 degrees either side of straight ahead.
DEFAULT_FRONT_ARC_DEG = 30
# gz-sim's contact sensor (update_rate=50Hz, see plugins.xacro) only
# publishes Contacts messages while contact is actually ongoing - confirmed
# live (2026-07-12) that once the robot backs away, the topic goes silent
# instead of publishing a final empty/false message. A boolean latched
# purely from the last received message therefore never clears. Bumper
# contact is instead treated as "current" for this long after the last
# true reading - several multiples of the ~20ms inter-message gap seen
# during real sustained contact, so an occasional dropped message doesn't
# spuriously clear it, while still releasing promptly (well under human
# reaction time) once contact truly ends.
DEFAULT_BUMPER_CONTACT_TIMEOUT_S = 0.25


class SafetyMonitor:

    def __init__(self, stop_distance_m=DEFAULT_STOP_DISTANCE_M,
                 front_arc_deg=DEFAULT_FRONT_ARC_DEG,
                 min_valid_range_m=DEFAULT_MIN_VALID_RANGE_M,
                 bumper_contact_timeout_s=DEFAULT_BUMPER_CONTACT_TIMEOUT_S,
                 clock=None):
        self._stop_distance_m = stop_distance_m
        self._front_arc_deg = front_arc_deg
        self._min_valid_range_m = min_valid_range_m
        self._bumper_contact_timeout_s = bumper_contact_timeout_s
        self._clock = clock or time.monotonic
        self._bumper_left_contact_until = None
        self._bumper_right_contact_until = None
        self._min_front_range = None

    def on_scan(self, ranges, angle_min, angle_increment):
        self._min_front_range = self._min_front_arc_range(
            ranges, angle_min, angle_increment)

    def on_bumper_left(self, has_contact):
        self._bumper_left_contact_until = self._contact_deadline(has_contact)

    def on_bumper_right(self, has_contact):
        self._bumper_right_contact_until = self._contact_deadline(has_contact)

    def _contact_deadline(self, has_contact):
        return self._clock() + self._bumper_contact_timeout_s if has_contact else None

    def is_blocked(self):
        now = self._clock()
        if self._bumper_left_contact_until is not None and now < self._bumper_left_contact_until:
            return True
        if self._bumper_right_contact_until is not None and now < self._bumper_right_contact_until:
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
