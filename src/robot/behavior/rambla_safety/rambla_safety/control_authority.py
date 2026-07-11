"""Pure, rclpy-independent AUTO/MANUAL command-authority arbitration.

Kept as a plain class (no ROS message types, no rclpy) so it's unit
testable without a running node - same shape as safety_monitor.SafetyMonitor
and rambla_control_panel's DeadmanTimer.

Resolves P1-3 (see .claude/internal-docs/audits/2026-07-08-accepted-
stabilization-findings.md Sec.5): exactly one source is ever selected for
/cmd_vel at a time, so autonomous (/cmd_vel_raw) and manual
(/cmd_vel_teleop) commands can never fight each other on the same topic.

Mode transitions:
  - AUTO -> MANUAL: the instant any manual command arrives. Manual takeover
    is immediate and exclusive - it does not wait for AUTO to yield.
  - MANUAL -> AUTO: only once no manual command has arrived for
    manual_timeout_s (mirrors DeadmanTimer's own timeout pattern -
    rambla_control_panel stops publishing /cmd_vel_teleop entirely once its
    local deadman goes inactive, so "no message" here means "genuinely
    disconnected/released", not "commanding zero while still connected").

Stale-command safety: auto commands are frozen (ignored) for the entire
time the mode is MANUAL, and both stored commands are zeroed exactly on
their own mode's exit transition. This guarantees that whichever source
was NOT selected can never be replayed as a stale leftover the instant
the mode flips back - a resuming source must publish a fresh command
before its value is used again.
"""

AUTO = 'AUTO'
MANUAL = 'MANUAL'

# Comfortably larger than rambla_control_panel's own DEADMAN_TIMEOUT_S
# (0.3s) plus its 20Hz (50ms) publish period, so this never times out
# before control_panel's own deadman has already stopped publishing.
DEFAULT_MANUAL_TIMEOUT_S = 0.4


class ControlAuthority:

    def __init__(self, manual_timeout_s=DEFAULT_MANUAL_TIMEOUT_S, clock=None):
        self._manual_timeout_s = manual_timeout_s
        self._clock = clock
        self._mode = AUTO
        self._last_manual_seen = None
        self._auto_cmd = (0.0, 0.0)
        self._manual_cmd = (0.0, 0.0)

    def on_auto_cmd(self, linear, angular):
        # Ignored entirely while MANUAL - not just unused - so an autonomous
        # behavior that keeps publishing through a takeover can't leave a
        # stale command sitting ready to replay the moment MANUAL releases.
        if self._mode == AUTO:
            self._auto_cmd = (linear, angular)

    def on_manual_cmd(self, linear, angular):
        self._last_manual_seen = self._clock()
        if self._mode == AUTO:
            self._mode = MANUAL
            self._auto_cmd = (0.0, 0.0)
        self._manual_cmd = (linear, angular)

    def tick(self):
        """Call once per publish cycle. Returns (mode, (linear, angular)):
        the currently-selected source and its command, after checking
        whether MANUAL has timed out back to AUTO."""
        if self._mode == MANUAL:
            timed_out = (
                self._last_manual_seen is None
                or self._clock() - self._last_manual_seen > self._manual_timeout_s
            )
            if timed_out:
                self._mode = AUTO
                self._manual_cmd = (0.0, 0.0)

        if self._mode == MANUAL:
            return MANUAL, self._manual_cmd
        return AUTO, self._auto_cmd

    @property
    def mode(self):
        return self._mode
