"""Exercises BehaviorSupervisor's mode-mux/USABLE-gate/goal-dispatch state
machine in isolation, with a fake clock - same style as
rambla_localization's test_probe_behavior.py and rambla_safety's
test_control_authority.py.
"""
import unittest

from rambla_localization.localization_state import (
    CONVERGING,
    DEGRADED,
    GLOBAL,
    USABLE,
)
from rambla_safety.control_authority import AUTO, MANUAL

from rambla_navigation.supervisor_state import (
    BehaviorSupervisor,
    CANCEL_GOAL,
    DISPATCH_GOAL,
    IDLE,
    MAP,
    NAV,
    NO_ACTION,
    PROBE,
)


class FakeClock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_supervisor(clock=None):
    return BehaviorSupervisor(clock=clock or FakeClock())


class TestBehaviorSupervisor(unittest.TestCase):

    def test_starts_idle_with_zero_cmd(self):
        sup = make_supervisor()
        mode, cmd, action = sup.tick()
        self.assertEqual(mode, IDLE)
        self.assertEqual(cmd, (0.0, 0.0))
        self.assertEqual(action, NO_ACTION)

    def test_pre_usable_selects_probe_and_forwards_verbatim(self):
        sup = make_supervisor()
        sup.on_localization_status(GLOBAL)
        sup.on_cmd_vel_probe(0.1, 0.3)
        mode, cmd, action = sup.tick()
        self.assertEqual(mode, PROBE)
        self.assertEqual(cmd, (0.1, 0.3))
        self.assertEqual(action, NO_ACTION)

    def test_probe_missing_cmd_holds_zero(self):
        sup = make_supervisor()
        sup.on_localization_status(CONVERGING)
        mode, cmd, action = sup.tick()
        self.assertEqual(mode, PROBE)
        self.assertEqual(cmd, (0.0, 0.0))

    def test_goal_held_pending_usable_then_dispatched_exactly_once(self):
        sup = make_supervisor()
        sup.on_localization_status(GLOBAL)
        sup.on_goal_pose('goal-1')
        mode, _, action = sup.tick()
        self.assertEqual(mode, PROBE)
        self.assertEqual(action, NO_ACTION)  # held, not dispatched pre-USABLE

        sup.on_localization_status(USABLE)
        sup.on_cmd_vel_nav(0.15, 0.0)
        mode, cmd, action = sup.tick()
        self.assertEqual(mode, NAV)
        self.assertEqual(cmd, (0.15, 0.0))
        self.assertEqual(action, DISPATCH_GOAL)

        mode, _, action = sup.tick()  # repeated USABLE tick
        self.assertEqual(mode, NAV)
        self.assertEqual(action, NO_ACTION)  # not re-dispatched

    def test_nav_forwards_cmd_vel_nav_verbatim(self):
        sup = make_supervisor()
        sup.on_localization_status(USABLE)
        sup.on_goal_pose('goal-1')
        sup.on_cmd_vel_nav(0.12, -0.2)
        sup.tick()  # dispatches
        mode, cmd, _ = sup.tick()
        self.assertEqual(mode, NAV)
        self.assertEqual(cmd, (0.12, -0.2))

    def test_status_leaves_usable_mid_nav_cancels_and_holds(self):
        sup = make_supervisor()
        sup.on_localization_status(USABLE)
        sup.on_goal_pose('goal-1')
        sup.on_cmd_vel_nav(0.15, 0.0)
        sup.tick()
        self.assertEqual(sup.mode, NAV)

        sup.on_localization_status(DEGRADED)
        mode, cmd, action = sup.tick()
        self.assertEqual(action, CANCEL_GOAL)
        self.assertNotEqual(mode, NAV)
        self.assertEqual(cmd, (0.0, 0.0))

        # NAV does not re-arm just because USABLE returns without a fresh
        # goal - the stale goal never auto-replays.
        sup.on_localization_status(USABLE)
        mode, _, action = sup.tick()
        self.assertNotEqual(mode, NAV)
        self.assertEqual(action, NO_ACTION)

    def test_manual_forces_idle_zero_and_preempts_running_goal(self):
        sup = make_supervisor()
        sup.on_localization_status(USABLE)
        sup.on_goal_pose('goal-1')
        sup.on_cmd_vel_nav(0.15, 0.0)
        sup.tick()
        self.assertEqual(sup.mode, NAV)

        sup.on_control_authority(MANUAL)
        mode, cmd, action = sup.tick()
        self.assertEqual(mode, IDLE)
        self.assertEqual(cmd, (0.0, 0.0))
        self.assertEqual(action, CANCEL_GOAL)

        # return to AUTO: does not auto-resume the old goal
        sup.on_control_authority(AUTO)
        mode, _, action = sup.tick()
        self.assertEqual(mode, IDLE)
        self.assertEqual(action, NO_ACTION)

    def test_manual_preempts_even_a_merely_pending_goal(self):
        sup = make_supervisor()
        sup.on_localization_status(GLOBAL)
        sup.on_goal_pose('goal-1')  # pending only, pre-USABLE - never dispatched
        sup.on_control_authority(MANUAL)
        mode, _, action = sup.tick()
        self.assertEqual(mode, IDLE)
        self.assertEqual(action, NO_ACTION)  # nothing was dispatched yet

        sup.on_control_authority(AUTO)
        sup.on_localization_status(USABLE)  # now usable, but goal was dropped
        mode, _, action = sup.tick()
        self.assertNotEqual(mode, NAV)
        self.assertEqual(action, NO_ACTION)

    def test_goal_finished_clears_active_goal_and_falls_back_to_idle(self):
        sup = make_supervisor()
        sup.on_localization_status(USABLE)
        sup.on_goal_pose('goal-1')
        sup.tick()
        self.assertEqual(sup.mode, NAV)

        sup.on_goal_finished()
        mode, cmd, action = sup.tick()
        self.assertEqual(mode, IDLE)
        self.assertEqual(cmd, (0.0, 0.0))
        self.assertEqual(action, NO_ACTION)

    def test_map_mode_is_the_dormant_fold_in_point(self):
        sup = make_supervisor()
        sup.on_map_request(True)
        sup.on_cmd_vel_map(0.1, 0.1)
        mode, cmd, _ = sup.tick()
        self.assertEqual(mode, MAP)
        self.assertEqual(cmd, (0.1, 0.1))

        # pre-USABLE re-localization still wins over a mapping request
        sup.on_localization_status(GLOBAL)
        mode, _, _ = sup.tick()
        self.assertEqual(mode, PROBE)

    def test_never_blends_two_sources(self):
        sup = make_supervisor()
        sup.on_localization_status(USABLE)
        sup.on_goal_pose('goal-1')
        sup.on_cmd_vel_nav(0.15, 0.0)
        sup.on_cmd_vel_probe(0.3, 0.3)  # stale value from before USABLE
        sup.tick()  # dispatches
        mode, cmd, _ = sup.tick()
        self.assertEqual(mode, NAV)
        self.assertEqual(cmd, (0.15, 0.0))  # never the probe value

    def test_last_transition_time_tracks_clock_on_mode_change(self):
        clock = FakeClock(t=10.0)
        sup = make_supervisor(clock=clock)
        sup.tick()  # IDLE -> IDLE, no transition
        self.assertIsNone(sup.last_transition_time)

        clock.advance(5.0)
        sup.on_localization_status(GLOBAL)
        sup.tick()  # IDLE -> PROBE
        self.assertEqual(sup.last_transition_time, 15.0)


if __name__ == '__main__':
    unittest.main()
