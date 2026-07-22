"""Exercises ProbeBehavior's grace-period/rotation/observe/bounded-attempts
state machine in isolation, with a fake clock - same style as
test_localization_state.py and rambla_safety's test_control_authority.py.
"""
import unittest

from rambla_localization.localization_state import (
    CONVERGING,
    DEGRADED,
    GLOBAL,
    LOST,
    UNINITIALIZED,
    USABLE,
)
from rambla_localization.probe_behavior import (
    EXHAUSTED,
    IDLE,
    OBSERVING,
    PROBING,
    TRANSLATING,
    ProbeBehavior,
)


class FakeClock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_probe(clock=None, **kwargs):
    kwargs.setdefault('grace_period_s', 1.0)
    kwargs.setdefault('angular_vel_rad_s', 0.3)
    kwargs.setdefault('attempt_duration_s', 1.0)
    kwargs.setdefault('observe_duration_s', 1.0)
    kwargs.setdefault('max_attempts', 2)
    kwargs.setdefault('stationary_linear_threshold_mps', 0.02)
    kwargs.setdefault('stationary_angular_threshold_radps', 0.02)
    # Same magnitude as the other phase durations so timing across mixed
    # PROBING/OBSERVING/TRANSLATING sequences stays easy to reason about in
    # whole tick-equivalent steps.
    kwargs.setdefault('linear_vel_mps', 0.1)
    kwargs.setdefault('translate_duration_s', 1.0)
    return ProbeBehavior(clock=clock or FakeClock(), **kwargs)


class TestProbeBehavior(unittest.TestCase):

    def test_stays_idle_while_inactive_status(self):
        for status in (UNINITIALIZED, USABLE, DEGRADED, LOST):
            clock = FakeClock()
            probe = make_probe(clock=clock)
            probe.on_localization_status(status)
            probe.on_velocity(0.0, 0.0)
            clock.advance(5.0)
            linear, angular = probe.tick()
            self.assertEqual(probe.mode, IDLE)
            self.assertEqual((linear, angular), (0.0, 0.0))

    def test_stays_idle_while_moving_even_if_global(self):
        clock = FakeClock()
        probe = make_probe(clock=clock)
        probe.on_localization_status(GLOBAL)
        probe.on_velocity(0.5, 0.0)
        clock.advance(5.0)
        linear, angular = probe.tick()
        self.assertEqual(probe.mode, IDLE)
        self.assertEqual((linear, angular), (0.0, 0.0))

    def test_enters_probing_after_grace_period_stationary(self):
        clock = FakeClock()
        probe = make_probe(clock=clock)
        probe.on_localization_status(GLOBAL)
        probe.on_velocity(0.0, 0.0)

        clock.advance(0.5)
        probe.tick()
        self.assertEqual(probe.mode, IDLE)  # grace period (1.0s) not yet up

        clock.advance(0.6)  # total 1.1s stationary
        linear, angular = probe.tick()
        self.assertEqual(probe.mode, PROBING)
        self.assertEqual(linear, 0.0)
        self.assertGreater(angular, 0.0)

    def test_repeated_global_status_does_not_reset_attempts(self):
        clock = FakeClock()
        probe = make_probe(clock=clock)
        probe.on_localization_status(GLOBAL)
        probe.on_velocity(0.0, 0.0)
        clock.advance(1.1)
        probe.tick()
        self.assertEqual(probe.mode, PROBING)

        probe.on_localization_status(GLOBAL)  # repeated - not a fresh edge
        self.assertEqual(probe.mode, PROBING)

    def test_cycles_probing_observing_while_unconverged(self):
        clock = FakeClock()
        probe = make_probe(clock=clock)
        probe.on_localization_status(GLOBAL)
        probe.on_velocity(0.0, 0.0)
        clock.advance(1.1)
        probe.tick()
        self.assertEqual(probe.mode, PROBING)

        probe.on_localization_status(CONVERGING)
        clock.advance(1.1)  # exceeds attempt_duration_s (1.0)
        probe.tick()
        self.assertEqual(probe.mode, OBSERVING)

        clock.advance(1.1)  # exceeds observe_duration_s (1.0)
        probe.tick()
        # Attempt 2 leads with TRANSLATING, not PROBING - see Task 2.
        self.assertEqual(probe.mode, TRANSLATING)
        self.assertEqual(probe.attempts_used, 1)

    def test_stops_on_reaching_usable_mid_attempt(self):
        clock = FakeClock()
        probe = make_probe(clock=clock)
        probe.on_localization_status(GLOBAL)
        probe.on_velocity(0.0, 0.0)
        clock.advance(1.1)
        probe.tick()
        self.assertEqual(probe.mode, PROBING)

        probe.on_localization_status(USABLE)
        linear, angular = probe.tick()
        self.assertEqual(probe.mode, IDLE)
        self.assertEqual((linear, angular), (0.0, 0.0))

    def test_reaches_exhausted_after_max_attempts_and_stays_zero(self):
        # grace(1) + attempt1(rotate 1 + observe 1) + attempt2(translate 1 +
        # rotate 1 + observe 1) = 6 tick-equivalent steps - one more than
        # before TRANSLATING existed, since attempt 2 now has an extra phase.
        clock = FakeClock()
        probe = make_probe(clock=clock, max_attempts=2)
        probe.on_localization_status(GLOBAL)
        probe.on_velocity(0.0, 0.0)
        for _ in range(6):
            clock.advance(1.1)
            probe.tick()
        self.assertEqual(probe.mode, EXHAUSTED)
        self.assertEqual(probe.attempts_used, 2)

        clock.advance(10.0)
        linear, angular = probe.tick()
        self.assertEqual(probe.mode, EXHAUSTED)
        self.assertEqual((linear, angular), (0.0, 0.0))

    def test_fresh_global_edge_rearms_from_exhausted(self):
        clock = FakeClock()
        probe = make_probe(clock=clock, max_attempts=2)
        probe.on_localization_status(GLOBAL)
        probe.on_velocity(0.0, 0.0)
        for _ in range(6):
            clock.advance(1.1)
            probe.tick()
        self.assertEqual(probe.mode, EXHAUSTED)

        probe.on_localization_status(LOST)
        probe.on_localization_status(GLOBAL)  # fresh reinit
        self.assertEqual(probe.mode, IDLE)
        self.assertEqual(probe.attempts_used, 0)

    def test_rearm_restarts_at_rotation_only_not_translating(self):
        # Re-arming must go back to attempt 1's cheap rotation-only path,
        # not resume wherever the exhausted run left off.
        clock = FakeClock()
        probe = make_probe(clock=clock, max_attempts=2)
        probe.on_localization_status(GLOBAL)
        probe.on_velocity(0.0, 0.0)
        for _ in range(6):
            clock.advance(1.1)
            probe.tick()
        self.assertEqual(probe.mode, EXHAUSTED)

        probe.on_localization_status(LOST)
        probe.on_localization_status(GLOBAL)  # fresh reinit
        clock.advance(1.1)
        linear, angular = probe.tick()
        self.assertEqual(probe.mode, PROBING)
        self.assertEqual(linear, 0.0)
        self.assertGreater(angular, 0.0)

    # --- M7 Phase 2.5 Task 2: translation on attempt 2+ ---------------------

    def test_first_attempt_is_rotation_only(self):
        clock = FakeClock()
        probe = make_probe(clock=clock)
        probe.on_localization_status(GLOBAL)
        probe.on_velocity(0.0, 0.0)
        clock.advance(1.1)
        linear, angular = probe.tick()
        self.assertEqual(probe.mode, PROBING)
        self.assertEqual(linear, 0.0)
        self.assertGreater(angular, 0.0)

    def test_second_attempt_starts_with_translating(self):
        clock = FakeClock()
        probe = make_probe(clock=clock)
        probe.on_localization_status(GLOBAL)
        probe.on_velocity(0.0, 0.0)
        clock.advance(1.1)
        probe.tick()
        self.assertEqual(probe.mode, PROBING)

        probe.on_localization_status(CONVERGING)
        clock.advance(1.1)  # exceeds attempt_duration_s -> OBSERVING
        probe.tick()
        self.assertEqual(probe.mode, OBSERVING)

        clock.advance(1.1)  # exceeds observe_duration_s -> next attempt
        probe.tick()
        self.assertEqual(probe.mode, TRANSLATING)
        self.assertEqual(probe.attempts_used, 1)

        # The command for the new phase is emitted starting the following
        # tick - same convention as every other phase transition here.
        clock.advance(0.1)
        linear, angular = probe.tick()
        self.assertEqual(probe.mode, TRANSLATING)
        self.assertGreater(linear, 0.0)
        self.assertEqual(angular, 0.0)

    def test_translating_transitions_to_probing_after_duration(self):
        clock = FakeClock()
        probe = make_probe(clock=clock)
        probe.on_localization_status(GLOBAL)
        probe.on_velocity(0.0, 0.0)
        clock.advance(1.1)
        probe.tick()  # attempt 1: PROBING
        probe.on_localization_status(CONVERGING)
        clock.advance(1.1)
        probe.tick()  # OBSERVING
        clock.advance(1.1)
        probe.tick()  # attempt 2: TRANSLATING
        self.assertEqual(probe.mode, TRANSLATING)

        clock.advance(1.1)  # exceeds translate_duration_s (1.0)
        linear, angular = probe.tick()
        self.assertEqual(probe.mode, PROBING)
        self.assertEqual(linear, 0.0)
        self.assertGreater(angular, 0.0)

    def test_translating_stops_immediately_on_leaving_active_status(self):
        clock = FakeClock()
        probe = make_probe(clock=clock)
        probe.on_localization_status(GLOBAL)
        probe.on_velocity(0.0, 0.0)
        clock.advance(1.1)
        probe.tick()
        probe.on_localization_status(CONVERGING)
        clock.advance(1.1)
        probe.tick()  # OBSERVING
        clock.advance(1.1)
        probe.tick()  # TRANSLATING
        self.assertEqual(probe.mode, TRANSLATING)

        probe.on_localization_status(USABLE)
        linear, angular = probe.tick()
        self.assertEqual(probe.mode, IDLE)
        self.assertEqual((linear, angular), (0.0, 0.0))

    def test_translating_never_combines_linear_and_angular(self):
        clock = FakeClock()
        probe = make_probe(clock=clock)
        probe.on_localization_status(GLOBAL)
        probe.on_velocity(0.0, 0.0)
        clock.advance(1.1)
        probe.tick()
        probe.on_localization_status(CONVERGING)
        clock.advance(1.1)
        probe.tick()  # OBSERVING
        clock.advance(1.1)
        probe.tick()  # TRANSLATING
        clock.advance(0.1)
        linear, angular = probe.tick()
        self.assertNotEqual(linear, 0.0)
        self.assertEqual(angular, 0.0)


if __name__ == '__main__':
    unittest.main()
