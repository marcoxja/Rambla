"""Standalone drive-command watchdog, decoupled from rclpy/asyncio so it's
unit-testable with a fake clock (see test/test_deadman.py). A dropped
websocket connection (closed tab, lost wifi) must not leave /cmd_vel
republishing the last nonzero command forever - the same failure mode as
the ros2 topic pub workaround this control panel replaces.
"""

DEADMAN_TIMEOUT_S = 0.3


class DeadmanTimer:
    """Tracks the last drive command and whether it's still fresh enough
    to keep republishing, per a caller-supplied monotonic clock function.
    """

    def __init__(self, timeout_s=DEADMAN_TIMEOUT_S, clock=None):
        self._timeout_s = timeout_s
        self._clock = clock
        self._linear = 0.0
        self._angular = 0.0
        self._last_update = None
        self._tripped = False

    def on_command(self, linear, angular):
        self._linear = linear
        self._angular = angular
        self._last_update = self._clock()
        self._tripped = False

    def on_disconnect(self):
        self._trip()

    def _trip(self):
        self._linear = 0.0
        self._angular = 0.0
        self._tripped = True

    def command(self):
        """Returns (linear, angular) to publish right now: the last received
        command if still fresh, otherwise zero. Trips (and stays tripped
        until the next on_command) the first time it goes stale, so callers
        can tell a fresh zero-because-tripped state from a fresh
        zero-because-user-centered-the-stick state if they need to.
        """
        if self._tripped:
            return 0.0, 0.0
        if self._last_update is None:
            return 0.0, 0.0
        if self._clock() - self._last_update > self._timeout_s:
            self._trip()
            return 0.0, 0.0
        return self._linear, self._angular

    @property
    def is_tripped(self):
        return self._tripped

    @property
    def is_active(self):
        """True only while a command has been received and hasn't gone
        stale - i.e. there's a live operator behind the current command,
        as opposed to a fresh zero from either startup or a trip. Lets a
        caller (e.g. control_panel_node's cmd_vel publisher) distinguish
        "genuinely under manual control" from "silent/never connected"
        without inspecting the command value itself.
        """
        return self._last_update is not None and not self._tripped
