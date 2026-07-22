"""Pure, rclpy-independent AMCL confidence state machine + recovery trigger.

Kept as a plain class (no ROS message types, no rclpy) so it's unit
testable without a running node - same shape as rambla_safety's
ControlAuthority and SafetyMonitor.

AMCL's own recovery_alpha_slow/fast (amcl.yaml) is a passive mitigation
against particle deprivation, not a guaranteed kidnapped-robot detector.
This class is the explicit mechanism M5_PLAN.md's Phase 4 calls for: it
classifies /particle_cloud spread and /amcl_pose covariance into a
confidence state, and tells the node wrapper when to call
/reinitialize_global_localization - both on startup and whenever LOST
persists past a bounded duration - so the robot recovers after being
moved without any operator action.

State machine:
  UNINITIALIZED -> GLOBAL: once the node wrapper calls request_reinit()
    for the first time (map_server/amcl active, first reinit issued).
  GLOBAL -> CONVERGING: particle spread narrows below converging_spread_m
    for the first time since the last reinit (still broad, no longer
    uniform-full-map).
  GLOBAL/CONVERGING -> USABLE: both particle spread and /amcl_pose
    covariance (xy, yaw) drop within their usable_* bounds.
  USABLE -> DEGRADED: any usable bound is exceeded again (spread growing /
    re-clustering) - the first sign something may be wrong.
  DEGRADED -> USABLE: bounds recovered.
  DEGRADED -> LOST: a hard ceiling is exceeded (spread or xy covariance),
    or no /particle_cloud or /amcl_pose update has arrived within
    stale_timeout_s (this staleness check is universal - it applies from
    any state except UNINITIALIZED) - EXCEPT while the robot is
    intentionally stationary (on_velocity) and the filter had already
    reached CONVERGING/USABLE/DEGRADED, where stale_timeout_s is replaced
    by the much longer stationary_stale_timeout_s. AMCL's own motion
    gating (amcl.yaml's update_min_d/a) means it simply stops publishing
    while the robot isn't moving - M7 Phase 2.5 found the short
    stale_timeout_s alone was misreading that silence as LOST every
    ~30s and destructively reiniting a filter that was actually
    converging fine, just quiet. This narrows the staleness trigger, not
    kidnap detection: a filter that never converged (still GLOBAL), or
    one that blew a hard ceiling while actually moving, is untouched by
    the extension and still goes LOST on the short timeout; a filter
    stale for longer than stationary_stale_timeout_s still goes LOST too
    (a real long-baseline stall) - the extension only holds the last
    belief steady through one probe EXHAUSTED cycle instead of wiping it.
  LOST -> GLOBAL: only via request_reinit(), which the node wrapper calls
    once tick() reports LOST has persisted for lost_duration_s and
    reinit_cooldown_s has elapsed since the last request.

GLOBAL/CONVERGING deliberately have no hard-ceiling check of their own:
right after a reinit, a full-map-uniform spread is expected and correct,
not a failure signal. The hard ceiling only means something once spread
was once low (USABLE/DEGRADED) and grows back - that's the actual kidnap
signal.
"""

UNINITIALIZED = 'UNINITIALIZED'
GLOBAL = 'GLOBAL'
CONVERGING = 'CONVERGING'
USABLE = 'USABLE'
DEGRADED = 'DEGRADED'
LOST = 'LOST'

# States worth holding steady through a stationary staleness gap rather than
# wiping via LOST->reinit - GLOBAL is deliberately excluded: a filter that
# hasn't narrowed at all yet has no partial belief worth protecting, and
# should keep using the short stale_timeout_s so a probe stuck in GLOBAL
# still gets re-armed promptly.
HOLD_ELIGIBLE_STATES = frozenset({CONVERGING, USABLE, DEGRADED})

DEFAULT_CONVERGING_SPREAD_M = 2.0
DEFAULT_USABLE_SPREAD_M = 0.5
DEFAULT_LOST_SPREAD_M = 3.0
DEFAULT_USABLE_COV_XY_M = 0.25
DEFAULT_LOST_COV_XY_M = 1.0
DEFAULT_USABLE_COV_YAW_RAD = 0.3
DEFAULT_STALE_TIMEOUT_S = 5.0
DEFAULT_STATIONARY_STALE_TIMEOUT_S = 60.0
DEFAULT_LOST_DURATION_S = 5.0
DEFAULT_REINIT_COOLDOWN_S = 10.0
DEFAULT_STATIONARY_LINEAR_THRESHOLD_MPS = 0.02
DEFAULT_STATIONARY_ANGULAR_THRESHOLD_RADPS = 0.02


class LocalizationMonitor:

    def __init__(
        self,
        clock,
        converging_spread_m=DEFAULT_CONVERGING_SPREAD_M,
        usable_spread_m=DEFAULT_USABLE_SPREAD_M,
        lost_spread_m=DEFAULT_LOST_SPREAD_M,
        usable_cov_xy_m=DEFAULT_USABLE_COV_XY_M,
        lost_cov_xy_m=DEFAULT_LOST_COV_XY_M,
        usable_cov_yaw_rad=DEFAULT_USABLE_COV_YAW_RAD,
        stale_timeout_s=DEFAULT_STALE_TIMEOUT_S,
        stationary_stale_timeout_s=DEFAULT_STATIONARY_STALE_TIMEOUT_S,
        lost_duration_s=DEFAULT_LOST_DURATION_S,
        reinit_cooldown_s=DEFAULT_REINIT_COOLDOWN_S,
        stationary_linear_threshold_mps=DEFAULT_STATIONARY_LINEAR_THRESHOLD_MPS,
        stationary_angular_threshold_radps=DEFAULT_STATIONARY_ANGULAR_THRESHOLD_RADPS,
    ):
        self._clock = clock
        self._converging_spread_m = converging_spread_m
        self._usable_spread_m = usable_spread_m
        self._lost_spread_m = lost_spread_m
        self._usable_cov_xy_m = usable_cov_xy_m
        self._lost_cov_xy_m = lost_cov_xy_m
        self._usable_cov_yaw_rad = usable_cov_yaw_rad
        self._stale_timeout_s = stale_timeout_s
        self._stationary_stale_timeout_s = stationary_stale_timeout_s
        self._lost_duration_s = lost_duration_s
        self._reinit_cooldown_s = reinit_cooldown_s
        self._stationary_linear_threshold_mps = stationary_linear_threshold_mps
        self._stationary_angular_threshold_radps = stationary_angular_threshold_radps

        self._state = UNINITIALIZED
        self._spread_m = None
        self._cov_xy_m = None
        self._cov_yaw_rad = None
        self._last_particle_time = None
        self._last_pose_time = None
        self._lost_since = None
        self._last_reinit_time = None
        # Assume moving until told otherwise: on_velocity may not have been
        # called yet (no /odometry/filtered message received), and the safe
        # default is the short stale_timeout_s - i.e. no behavior change
        # for any caller that never wires up on_velocity at all.
        self._moving = True

    @property
    def state(self):
        return self._state

    def on_velocity(self, linear_mps, angular_radps):
        """Feed real /odometry/filtered velocity (not e.g. /control_authority
        going AUTO, which only means no teleop command for rambla_safety's
        manual_timeout_s - the robot may still be physically coasting) -
        same source and thresholds as probe_behavior.ProbeBehavior.
        on_velocity, kept as a separate, independent check here rather than
        importing from that module to avoid a cross-file coupling between
        these two pure state machines."""
        self._moving = (
            abs(linear_mps) > self._stationary_linear_threshold_mps
            or abs(angular_radps) > self._stationary_angular_threshold_radps
        )

    def on_particle_cloud(self, spread_m):
        self._spread_m = spread_m
        self._last_particle_time = self._clock()
        self._recompute()

    def on_amcl_pose(self, cov_xy_m, cov_yaw_rad):
        self._cov_xy_m = cov_xy_m
        self._cov_yaw_rad = cov_yaw_rad
        self._last_pose_time = self._clock()
        self._recompute()

    def tick(self):
        """Call once per monitor cycle. Re-checks staleness even with no
        new messages, and returns (state, should_reinit): should_reinit
        is True exactly when LOST has persisted for lost_duration_s and
        reinit_cooldown_s has elapsed since the last request - the caller
        is then expected to actually issue the service call and follow up
        with request_reinit()."""
        self._recompute()
        if self._state != LOST or self._lost_since is None:
            return self._state, False
        now = self._clock()
        sustained = now - self._lost_since >= self._lost_duration_s
        cooled_down = (
            self._last_reinit_time is None
            or now - self._last_reinit_time >= self._reinit_cooldown_s
        )
        return self._state, sustained and cooled_down

    def request_reinit(self):
        """Call once the node has actually issued (or is about to issue)
        the /reinitialize_global_localization request - on startup once
        map_server/amcl are active, or whenever tick() signals a
        sustained-LOST recovery. Optimistically resets tracking to GLOBAL:
        the request is fire-and-forget, matching AMCL's own service
        semantics, and the state machine re-derives CONVERGING/USABLE from
        the next fresh /particle_cloud and /amcl_pose messages.

        Also resets the staleness clock (_last_particle_time/
        _last_pose_time) to now - waiting for the first post-reinit
        message is not itself "stale", it's just not usable yet (already
        covered by _is_within_usable_bounds's own None checks)."""
        self._state = GLOBAL
        self._spread_m = None
        self._cov_xy_m = None
        self._cov_yaw_rad = None
        self._lost_since = None
        now = self._clock()
        self._last_particle_time = now
        self._last_pose_time = now
        self._last_reinit_time = now

    def _is_within_usable_bounds(self):
        return (
            self._spread_m is not None
            and self._spread_m <= self._usable_spread_m
            and self._cov_xy_m is not None
            and self._cov_xy_m <= self._usable_cov_xy_m
            and self._cov_yaw_rad is not None
            and self._cov_yaw_rad <= self._usable_cov_yaw_rad
        )

    def _effective_stale_timeout_s(self):
        # Hold a converged-ish belief through AMCL's own motion-gated
        # silence instead of racing the short timeout - see the module
        # docstring's DEGRADED -> LOST note. GLOBAL is excluded via
        # HOLD_ELIGIBLE_STATES: no partial belief exists yet there, so a
        # stuck-in-GLOBAL probe should still re-arm promptly.
        if not self._moving and self._state in HOLD_ELIGIBLE_STATES:
            return self._stationary_stale_timeout_s
        return self._stale_timeout_s

    def _is_stale(self):
        # Both timestamps are always set by request_reinit() before the
        # state can leave UNINITIALIZED (the only state _is_stale is
        # skipped for), so neither is ever None here.
        now = self._clock()
        timeout = self._effective_stale_timeout_s()
        return (
            now - self._last_particle_time > timeout
            or now - self._last_pose_time > timeout
        )

    def _enter_lost(self):
        if self._state != LOST:
            self._lost_since = self._clock()
        self._state = LOST

    def _recompute(self):
        if self._state == UNINITIALIZED:
            return  # waiting for request_reinit() to be called on startup

        if self._is_stale():
            self._enter_lost()
            return

        usable = self._is_within_usable_bounds()

        if self._state == GLOBAL:
            if usable:
                self._state = USABLE
            elif (
                self._spread_m is not None
                and self._spread_m <= self._converging_spread_m
            ):
                self._state = CONVERGING
        elif self._state == CONVERGING:
            if usable:
                self._state = USABLE
            elif (
                self._spread_m is not None
                and self._spread_m > self._lost_spread_m
            ):
                self._enter_lost()
        elif self._state == USABLE:
            if not usable:
                self._state = DEGRADED
        elif self._state == DEGRADED:
            if usable:
                self._state = USABLE
            elif (
                (self._spread_m is not None
                 and self._spread_m > self._lost_spread_m)
                or (self._cov_xy_m is not None
                    and self._cov_xy_m > self._lost_cov_xy_m)
            ):
                self._enter_lost()
        # LOST only leaves via request_reinit()
