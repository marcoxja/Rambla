"""Pure, rclpy-independent state machine for the M5 Phase 5 "cautious
disambiguation" behavior: AMCL's own particle filter often can't
disambiguate multiple pose hypotheses from a single stationary scan in a
symmetric space - it needs motion to accumulate distinguishing bearings
between landmarks. Kept as a plain class (no ROS message types, no rclpy)
so it's unit testable without a running node - same shape as
rambla_safety's ControlAuthority/SafetyMonitor and this package's own
localization_state.LocalizationMonitor.

This is deliberately minimal - not a navigation/path-planning behavior. It
commands nothing but a slow, bounded in-place rotation and (from the
second attempt on) a short straight-line translation, and only while
localization hasn't converged and the robot is genuinely stationary.

Rotation alone accumulates distinguishing *bearings* from a single vantage
point, but a symmetric multi-room map can still leave several pose
hypotheses equally consistent with every bearing observed from that one
point (M7 Phase 2.5's live cold_start trace against map_v002: rotation-only
reached CONVERGING in only 1 of 9 GLOBAL cycles). Translation actually
moves the vantage point, which a purely rotating scan can never do -
leaving/re-entering a room changes which walls/landmarks are even visible,
which is what breaks that aliasing. Translation is left out of the first
attempt on purpose: it's extra motion/time cost that most convergences
don't need, so only attempts 2+ pay for it, after rotation-only has already
had one clean shot from the arming pose.

State machine:
  IDLE -> PROBING (attempt 1) or TRANSLATING (attempt 2+): once
    /localization_status reads GLOBAL/CONVERGING (on_localization_status)
    and the robot has reported near-zero velocity (on_velocity)
    continuously for grace_period_s - real /odometry/filtered velocity, not
    just "no teleop": /control_authority flipping back to AUTO only means
    no teleop command for rambla_safety's own manual_timeout_s (0.4s), the
    robot may still be physically coasting.
  TRANSLATING: commands a slow straight-line drive (linear_vel_mps, zero
    angular) for translate_duration_s, only on attempts_used >= 1 (the 2nd
    attempt onward). Collision-safe for free: this behavior's own output
    still flows through (M7) behavior_supervisor -> /cmd_vel_raw ->
    rambla_safety, whose collision reflex stops the robot before a wall
    exactly like it would for any other AUTO command - no obstacle check
    needed here.
  TRANSLATING -> PROBING: after translate_duration_s, stop translating and
    start rotating from the new vantage point.
  PROBING: commands a slow in-place rotation (angular_vel_rad_s, zero
    linear always) for attempt_duration_s - long enough to exceed
    amcl.yaml's update_min_a (0.2 rad) so AMCL actually resamples.
  PROBING -> OBSERVING: after attempt_duration_s, stop and hold for
    observe_duration_s so AMCL can process the new scans and
    localization_monitor can reclassify before the next attempt.
  OBSERVING -> PROBING (attempt 1 only) or TRANSLATING (attempt 2+):
    status is still GLOBAL/CONVERGING and attempts_used hasn't reached
    max_attempts - starts another attempt.
  OBSERVING -> EXHAUSTED: attempts_used has reached max_attempts - stop and
    stay put ("stay put and stay LOST/DEGRADED," never open-ended
    searching) until re-armed. Translation does not grow this budget - it's
    folded into the same bounded max_attempts count, not an extra phase on
    top of it.
  PROBING/OBSERVING/TRANSLATING -> IDLE: status leaves {GLOBAL, CONVERGING}
    for any reason (USABLE = success, or DEGRADED/LOST/UNINITIALIZED) -
    stop immediately.
  * -> IDLE, attempts_used reset to 0: on_localization_status observes a
    *fresh* edge into GLOBAL (previous status was something else). This is
    exactly when localization_monitor has just issued a
    /reinitialize_global_localization call (startup or post-kidnap
    recovery), so every new localization attempt gets its own bounded
    attempt budget, including re-arming from EXHAUSTED - and starts back at
    attempt 1 (rotation-only), not wherever the previous run left off.
"""

from rambla_localization.localization_state import CONVERGING, GLOBAL

IDLE = 'IDLE'
TRANSLATING = 'TRANSLATING'
PROBING = 'PROBING'
OBSERVING = 'OBSERVING'
EXHAUSTED = 'EXHAUSTED'

ACTIVE_STATUSES = frozenset({GLOBAL, CONVERGING})

DEFAULT_GRACE_PERIOD_S = 2.0
DEFAULT_ANGULAR_VEL_RAD_S = 0.3
DEFAULT_ATTEMPT_DURATION_S = 3.0
DEFAULT_OBSERVE_DURATION_S = 2.0
DEFAULT_MAX_ATTEMPTS = 4
DEFAULT_STATIONARY_LINEAR_THRESHOLD_MPS = 0.02
DEFAULT_STATIONARY_ANGULAR_THRESHOLD_RADPS = 0.02
# Matches verify_localization.py's own measure_delta_accuracy default
# linear_mps - an already-proven-gentle scripted speed in this codebase.
DEFAULT_LINEAR_VEL_MPS = 0.15
DEFAULT_TRANSLATE_DURATION_S = 2.0


class ProbeBehavior:

    def __init__(
        self,
        clock,
        grace_period_s=DEFAULT_GRACE_PERIOD_S,
        angular_vel_rad_s=DEFAULT_ANGULAR_VEL_RAD_S,
        attempt_duration_s=DEFAULT_ATTEMPT_DURATION_S,
        observe_duration_s=DEFAULT_OBSERVE_DURATION_S,
        max_attempts=DEFAULT_MAX_ATTEMPTS,
        stationary_linear_threshold_mps=DEFAULT_STATIONARY_LINEAR_THRESHOLD_MPS,
        stationary_angular_threshold_radps=DEFAULT_STATIONARY_ANGULAR_THRESHOLD_RADPS,
        linear_vel_mps=DEFAULT_LINEAR_VEL_MPS,
        translate_duration_s=DEFAULT_TRANSLATE_DURATION_S,
    ):
        self._clock = clock
        self._grace_period_s = grace_period_s
        self._angular_vel_rad_s = angular_vel_rad_s
        self._attempt_duration_s = attempt_duration_s
        self._observe_duration_s = observe_duration_s
        self._max_attempts = max_attempts
        self._stationary_linear_threshold_mps = stationary_linear_threshold_mps
        self._stationary_angular_threshold_radps = stationary_angular_threshold_radps
        self._linear_vel_mps = linear_vel_mps
        self._translate_duration_s = translate_duration_s

        self._mode = IDLE
        self._status = None
        self._attempts_used = 0
        self._stationary_since = None
        self._phase_start_time = None

    @property
    def mode(self):
        return self._mode

    @property
    def attempts_used(self):
        return self._attempts_used

    def on_localization_status(self, status):
        fresh_global = status == GLOBAL and self._status != GLOBAL
        self._status = status
        if fresh_global:
            self._attempts_used = 0
            self._mode = IDLE
            self._phase_start_time = None

    def on_velocity(self, linear_mps, angular_radps):
        moving = (
            abs(linear_mps) > self._stationary_linear_threshold_mps
            or abs(angular_radps) > self._stationary_angular_threshold_radps
        )
        if moving:
            self._stationary_since = None
        elif self._stationary_since is None:
            self._stationary_since = self._clock()

    def _is_stationary_for_grace_period(self):
        if self._stationary_since is None:
            return False
        return self._clock() - self._stationary_since >= self._grace_period_s

    def _next_attempt_mode(self):
        # Attempt 1 (attempts_used == 0) is rotation-only, preserving the
        # original cheap common case; attempt 2+ leads with a translation
        # to actually change the vantage point - see module docstring.
        return TRANSLATING if self._attempts_used >= 1 else PROBING

    def tick(self):
        """Call once per node tick - only while /control_authority is AUTO.
        The wrapper freezes this state machine entirely during MANUAL by
        simply not calling tick() (same as traversal_node not calling
        WanderBehavior.next_command() while MANUAL): a MANUAL interruption
        means a human is physically driving the robot, which invalidates
        whatever attempt was in progress anyway, so cutting the phase short
        on resume (elapsed time will already exceed the phase's duration)
        is reasonable, not a bug. Returns (linear, angular) to publish on
        /cmd_vel_raw - linear is nonzero only during TRANSLATING, angular is
        nonzero only during PROBING; never both at once."""
        now = self._clock()
        active = self._status in ACTIVE_STATUSES

        if self._mode in (TRANSLATING, PROBING, OBSERVING) and not active:
            self._mode = IDLE
            self._phase_start_time = None

        if self._mode == IDLE:
            if active and self._is_stationary_for_grace_period():
                self._mode = self._next_attempt_mode()
                self._phase_start_time = now
                if self._mode == TRANSLATING:
                    return self._linear_vel_mps, 0.0
                return 0.0, self._angular_vel_rad_s
            return 0.0, 0.0

        if self._mode == TRANSLATING:
            if now - self._phase_start_time >= self._translate_duration_s:
                self._mode = PROBING
                self._phase_start_time = now
                return 0.0, self._angular_vel_rad_s
            return self._linear_vel_mps, 0.0

        if self._mode == PROBING:
            if now - self._phase_start_time >= self._attempt_duration_s:
                self._mode = OBSERVING
                self._phase_start_time = now
                return 0.0, 0.0
            return 0.0, self._angular_vel_rad_s

        if self._mode == OBSERVING:
            if now - self._phase_start_time >= self._observe_duration_s:
                self._attempts_used += 1
                if self._attempts_used >= self._max_attempts:
                    self._mode = EXHAUSTED
                    self._phase_start_time = None
                else:
                    self._mode = self._next_attempt_mode()
                    self._phase_start_time = now
            return 0.0, 0.0

        return 0.0, 0.0  # EXHAUSTED
