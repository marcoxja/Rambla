"""Pure, rclpy-independent behavior/mode arbitration - the M7 Layer 2 sole
/cmd_vel_raw writer. Muxes the autonomous velocity-intent sources (NAV /
PROBE / MAP) into one and centralizes the /localization_status == USABLE
gate that would otherwise force every map-relative behavior to re-implement
it independently (see control-arbitration-layers.md's "why this centralizes
gating" section).

Kept as a plain class (no ROS message types, no rclpy) so it's unit
testable without a running node - same shape as rambla_safety's
ControlAuthority and this repo's LocalizationMonitor/ProbeBehavior.

MANUAL is never re-decided here - rambla_safety's ControlAuthority (Layer 1)
remains the sole owner of who wins; AUTO/MANUAL string values are imported
from there rather than duplicated. /control_authority is consulted only to
pause this layer's own behavior (Option A, control-arbitration-layers.md
"MANUAL takeover: pause, then re-evaluate on resume"): hold zero and
preempt/cancel any in-flight goal so Nav2 doesn't keep servoing against a
robot a human is driving. On return to AUTO the supervisor does not
auto-resume the pre-takeover goal - it holds no un-cancellable mission
state, and waits for a fresh on_goal_pose(). Deciding continue-vs-re-evaluate
after a takeover is a future Layer-3 (mission) responsibility.

Mode selection each tick (highest to lowest priority):
  MANUAL -> IDLE, zero, cancel any active goal, drop any pending goal.
  AUTO, active goal + status == USABLE -> NAV, forward /cmd_vel_nav.
  AUTO, active goal + status != USABLE -> cancel the goal (once this edge),
    then re-evaluate mode below from the same, now-cleared state.
  AUTO, status in {GLOBAL, CONVERGING} -> PROBE, forward /cmd_vel_probe -
    same pre-USABLE re-localization statuses probe_behavior.ProbeBehavior
    itself treats as ACTIVE_STATUSES.
  AUTO, status == USABLE, a pending goal exists -> dispatch it (becomes the
    active goal), NAV, forward /cmd_vel_nav.
  AUTO, a map request is active (dormant in M7 - rambla_traversal stays on
    its own wander.launch.py, not wired live; kept as the documented future
    fold-in point, control-arbitration-layers.md's "Add a Behavior" row) ->
    MAP, forward /cmd_vel_map.
  otherwise -> IDLE, zero.

A goal is re-armable only via a fresh on_goal_pose() call, and only takes
effect once dispatched - one cancelled by MANUAL or by leaving USABLE never
auto-replays (same anti-replay discipline as ControlAuthority's
stale-command handling). on_goal_finished() (the action client's result
callback, success or failure alike) likewise clears the active goal so a
completed run doesn't wedge the mux in NAV forever - distinguishing success
from failure for reporting is Phase 4's concern, not this mux's.
"""
from rambla_localization.localization_state import CONVERGING, GLOBAL, USABLE
from rambla_safety.control_authority import AUTO, MANUAL

IDLE = 'IDLE'
MAP = 'MAP'
NAV = 'NAV'
PROBE = 'PROBE'

DISPATCH_GOAL = 'dispatch_goal'
CANCEL_GOAL = 'cancel_goal'
NO_ACTION = 'none'

PRE_USABLE_STATUSES = frozenset({GLOBAL, CONVERGING})

ZERO_CMD = (0.0, 0.0)


class BehaviorSupervisor:

    def __init__(self, clock):
        self._clock = clock
        self._mode = IDLE
        self._last_transition_time = None

        self._authority_mode = AUTO
        self._loc_status = None

        self._pending_goal = None
        self._active_goal = None

        self._nav_cmd = None
        self._probe_cmd = None
        self._map_cmd = None
        self._map_requested = False

    @property
    def mode(self):
        return self._mode

    @property
    def last_transition_time(self):
        return self._last_transition_time

    def on_control_authority(self, mode):
        self._authority_mode = mode

    def on_localization_status(self, status):
        self._loc_status = status

    def on_goal_pose(self, goal):
        """goal is an opaque token to this pure class (the node wrapper's
        actual PoseStamped) - only its presence/absence and dispatch timing
        matter here."""
        self._pending_goal = goal

    def on_goal_finished(self):
        """Call once the action client's result callback fires, regardless
        of SUCCEEDED/CANCELED/ABORTED - see module docstring."""
        self._active_goal = None

    def on_cmd_vel_nav(self, linear, angular):
        self._nav_cmd = (linear, angular)

    def on_cmd_vel_probe(self, linear, angular):
        self._probe_cmd = (linear, angular)

    def on_cmd_vel_map(self, linear, angular):
        self._map_cmd = (linear, angular)

    def on_map_request(self, active):
        """Future fold-in point (dormant in M7 - see module docstring): a
        Layer-3/operator signal that MAP mode should drive when nothing
        higher-priority claims the tick."""
        self._map_requested = active

    def tick(self):
        """Call once per publish cycle. Returns (mode, (linear, angular),
        action) where action is one of DISPATCH_GOAL/CANCEL_GOAL/NO_ACTION -
        the caller is expected to actually send/cancel the NavigateToPose
        goal when action is not NO_ACTION."""
        action = NO_ACTION

        if self._authority_mode == MANUAL:
            if self._active_goal is not None:
                action = CANCEL_GOAL
            self._active_goal = None
            self._pending_goal = None
            self._set_mode(IDLE)
            return self._mode, ZERO_CMD, action

        if self._active_goal is not None and self._loc_status != USABLE:
            action = CANCEL_GOAL
            self._active_goal = None

        if self._active_goal is not None:
            self._set_mode(NAV)
            return self._mode, self._cmd_or_zero(self._nav_cmd), action

        if self._loc_status in PRE_USABLE_STATUSES:
            self._set_mode(PROBE)
            return self._mode, self._cmd_or_zero(self._probe_cmd), action

        if self._loc_status == USABLE and self._pending_goal is not None:
            self._active_goal = self._pending_goal
            self._pending_goal = None
            action = DISPATCH_GOAL
            self._set_mode(NAV)
            return self._mode, self._cmd_or_zero(self._nav_cmd), action

        if self._map_requested:
            self._set_mode(MAP)
            return self._mode, self._cmd_or_zero(self._map_cmd), action

        self._set_mode(IDLE)
        return self._mode, ZERO_CMD, action

    def _set_mode(self, mode):
        if mode != self._mode:
            self._mode = mode
            self._last_transition_time = self._clock()

    @staticmethod
    def _cmd_or_zero(cmd):
        return cmd if cmd is not None else ZERO_CMD
