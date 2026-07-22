"""Exercises LocalizationMonitor's confidence state machine in isolation,
with a fake clock - same style as rambla_safety's test_control_authority.py
and test_safety_monitor.py.
"""
import unittest

from rambla_localization.localization_state import (
    CONVERGING,
    DEGRADED,
    GLOBAL,
    LOST,
    UNINITIALIZED,
    USABLE,
    LocalizationMonitor,
)


class FakeClock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


def make_monitor(clock=None, **kwargs):
    return LocalizationMonitor(
        clock=clock or FakeClock(),
        converging_spread_m=2.0,
        usable_spread_m=0.5,
        lost_spread_m=3.0,
        usable_cov_xy_m=0.25,
        lost_cov_xy_m=1.0,
        usable_cov_yaw_rad=0.3,
        stale_timeout_s=5.0,
        stationary_stale_timeout_s=30.0,
        lost_duration_s=5.0,
        reinit_cooldown_s=10.0,
        **kwargs)


class TestLocalizationMonitor(unittest.TestCase):

    def test_starts_uninitialized(self):
        monitor = make_monitor()
        self.assertEqual(monitor.state, UNINITIALIZED)

    def test_request_reinit_moves_to_global(self):
        monitor = make_monitor()
        monitor.request_reinit()
        self.assertEqual(monitor.state, GLOBAL)

    def test_global_stays_global_with_wide_spread(self):
        monitor = make_monitor()
        monitor.request_reinit()
        monitor.on_particle_cloud(5.0)
        monitor.on_amcl_pose(2.0, 1.0)
        self.assertEqual(monitor.state, GLOBAL)

    def test_global_narrows_to_converging(self):
        monitor = make_monitor()
        monitor.request_reinit()
        monitor.on_particle_cloud(1.5)
        monitor.on_amcl_pose(0.8, 0.5)
        self.assertEqual(monitor.state, CONVERGING)

    def test_converging_reaches_usable(self):
        monitor = make_monitor()
        monitor.request_reinit()
        monitor.on_particle_cloud(1.5)
        monitor.on_amcl_pose(0.8, 0.5)
        self.assertEqual(monitor.state, CONVERGING)
        monitor.on_particle_cloud(0.2)
        monitor.on_amcl_pose(0.1, 0.05)
        self.assertEqual(monitor.state, USABLE)

    def test_global_can_jump_straight_to_usable(self):
        monitor = make_monitor()
        monitor.request_reinit()
        monitor.on_particle_cloud(0.2)
        monitor.on_amcl_pose(0.1, 0.05)
        self.assertEqual(monitor.state, USABLE)

    def test_usable_degrades_when_spread_grows(self):
        monitor = make_monitor()
        monitor.request_reinit()
        monitor.on_particle_cloud(0.2)
        monitor.on_amcl_pose(0.1, 0.05)
        self.assertEqual(monitor.state, USABLE)
        monitor.on_particle_cloud(0.8)
        self.assertEqual(monitor.state, DEGRADED)

    def test_degraded_recovers_to_usable(self):
        monitor = make_monitor()
        monitor.request_reinit()
        monitor.on_particle_cloud(0.2)
        monitor.on_amcl_pose(0.1, 0.05)
        monitor.on_particle_cloud(0.8)
        self.assertEqual(monitor.state, DEGRADED)
        monitor.on_particle_cloud(0.2)
        self.assertEqual(monitor.state, USABLE)

    def test_degraded_hits_lost_on_hard_ceiling(self):
        monitor = make_monitor()
        monitor.request_reinit()
        monitor.on_particle_cloud(0.2)
        monitor.on_amcl_pose(0.1, 0.05)
        monitor.on_particle_cloud(0.8)
        self.assertEqual(monitor.state, DEGRADED)
        monitor.on_particle_cloud(3.5)  # past lost_spread_m
        self.assertEqual(monitor.state, LOST)

    def test_converging_hits_lost_on_hard_ceiling(self):
        monitor = make_monitor()
        monitor.request_reinit()
        monitor.on_particle_cloud(1.5)
        monitor.on_amcl_pose(0.8, 0.5)
        self.assertEqual(monitor.state, CONVERGING)
        monitor.on_particle_cloud(3.5)
        self.assertEqual(monitor.state, LOST)

    def test_global_wide_spread_does_not_trigger_lost(self):
        # GLOBAL's expected full-map spread must not itself read as LOST -
        # only USABLE/DEGRADED regressing into a hard ceiling should.
        monitor = make_monitor()
        monitor.request_reinit()
        monitor.on_particle_cloud(9.0)
        monitor.on_amcl_pose(4.0, 2.0)
        self.assertEqual(monitor.state, GLOBAL)

    def test_stale_data_triggers_lost(self):
        clock = FakeClock()
        monitor = make_monitor(clock=clock)
        monitor.request_reinit()
        monitor.on_particle_cloud(0.2)
        monitor.on_amcl_pose(0.1, 0.05)
        self.assertEqual(monitor.state, USABLE)
        clock.advance(5.1)
        state, _ = monitor.tick()
        self.assertEqual(state, LOST)

    def test_tick_does_not_request_reinit_before_lost_duration(self):
        clock = FakeClock()
        monitor = make_monitor(clock=clock)
        monitor.request_reinit()
        monitor.on_particle_cloud(0.2)
        monitor.on_amcl_pose(0.1, 0.05)
        monitor.on_particle_cloud(0.8)
        monitor.on_particle_cloud(3.5)  # -> LOST
        self.assertEqual(monitor.state, LOST)
        clock.advance(4.9)
        state, should_reinit = monitor.tick()
        self.assertEqual(state, LOST)
        self.assertFalse(should_reinit)

    def test_tick_requests_reinit_after_sustained_lost(self):
        clock = FakeClock()
        monitor = make_monitor(clock=clock)
        monitor.request_reinit()
        monitor.on_particle_cloud(0.2)
        monitor.on_amcl_pose(0.1, 0.05)
        monitor.on_particle_cloud(0.8)
        monitor.on_particle_cloud(3.5)  # -> LOST (lost_since=0, last_reinit=0)
        # Must clear both the lost_duration_s (5.0) AND reinit_cooldown_s
        # (10.0) windows - both started at t=0 here, so 10.1s covers both.
        clock.advance(10.1)
        state, should_reinit = monitor.tick()
        self.assertEqual(state, LOST)
        self.assertTrue(should_reinit)

    def test_reinit_cooldown_blocks_immediate_retrigger(self):
        clock = FakeClock()
        monitor = make_monitor(clock=clock)
        monitor.request_reinit()  # last_reinit_time = 0
        clock.advance(2.0)  # lost transition below happens at t=2.0
        monitor.on_particle_cloud(0.2)
        monitor.on_amcl_pose(0.1, 0.05)
        monitor.on_particle_cloud(0.8)
        monitor.on_particle_cloud(3.5)  # -> LOST, lost_since=2.0
        clock.advance(5.0)  # t=7.0: sustained (7-2>=5) but cooldown (7-0<10)
        state, should_reinit = monitor.tick()
        self.assertEqual(state, LOST)
        self.assertFalse(should_reinit)

    # --- M7 Phase 2.5 Task 1: stationary staleness hold ---------------------

    def test_stationary_converging_holds_through_short_staleness(self):
        # AMCL motion-gated silence past the short stale_timeout_s must not
        # wipe a partially-converged filter while the robot is genuinely
        # stationary - this is the destructive-reinit-thrash fix.
        clock = FakeClock()
        monitor = make_monitor(clock=clock)
        monitor.request_reinit()
        monitor.on_particle_cloud(1.5)
        monitor.on_amcl_pose(0.8, 0.5)
        self.assertEqual(monitor.state, CONVERGING)
        monitor.on_velocity(0.0, 0.0)
        clock.advance(5.1)  # past stale_timeout_s, well under stationary_stale_timeout_s
        state, should_reinit = monitor.tick()
        self.assertEqual(state, CONVERGING)
        self.assertFalse(should_reinit)

    def test_stationary_usable_holds_through_short_staleness(self):
        clock = FakeClock()
        monitor = make_monitor(clock=clock)
        monitor.request_reinit()
        monitor.on_particle_cloud(0.2)
        monitor.on_amcl_pose(0.1, 0.05)
        self.assertEqual(monitor.state, USABLE)
        monitor.on_velocity(0.0, 0.0)
        clock.advance(5.1)
        state, should_reinit = monitor.tick()
        self.assertEqual(state, USABLE)
        self.assertFalse(should_reinit)

    def test_moving_still_uses_short_stale_timeout(self):
        # Genuine LOST detection while actually moving must be untouched -
        # the hold only applies to intentional stationary silence.
        clock = FakeClock()
        monitor = make_monitor(clock=clock)
        monitor.request_reinit()
        monitor.on_particle_cloud(0.2)
        monitor.on_amcl_pose(0.1, 0.05)
        self.assertEqual(monitor.state, USABLE)
        monitor.on_velocity(0.3, 0.0)
        clock.advance(5.1)
        state, _ = monitor.tick()
        self.assertEqual(state, LOST)

    def test_stationary_global_still_uses_short_stale_timeout(self):
        # GLOBAL has no partial belief worth protecting - a probe stuck in
        # GLOBAL should still re-arm promptly via the short timeout.
        clock = FakeClock()
        monitor = make_monitor(clock=clock)
        monitor.request_reinit()
        monitor.on_particle_cloud(9.0)
        monitor.on_amcl_pose(4.0, 2.0)
        self.assertEqual(monitor.state, GLOBAL)
        monitor.on_velocity(0.0, 0.0)
        clock.advance(5.1)
        state, _ = monitor.tick()
        self.assertEqual(state, LOST)

    def test_stationary_long_stall_still_goes_lost(self):
        # The hold is bounded, not indefinite - a real long-baseline stall
        # still resolves to LOST (and eventually reinit) once staleness
        # exceeds stationary_stale_timeout_s.
        clock = FakeClock()
        monitor = make_monitor(clock=clock)
        monitor.request_reinit()
        monitor.on_particle_cloud(1.5)
        monitor.on_amcl_pose(0.8, 0.5)
        self.assertEqual(monitor.state, CONVERGING)
        monitor.on_velocity(0.0, 0.0)
        clock.advance(30.1)  # past stationary_stale_timeout_s
        state, _ = monitor.tick()
        self.assertEqual(state, LOST)

    def test_stationary_hold_ends_once_moving_again(self):
        # Motion resuming mid-hold must drop back to the short timeout
        # immediately, not linger on the extended one.
        clock = FakeClock()
        monitor = make_monitor(clock=clock)
        monitor.request_reinit()
        monitor.on_particle_cloud(0.2)
        monitor.on_amcl_pose(0.1, 0.05)
        self.assertEqual(monitor.state, USABLE)
        monitor.on_velocity(0.0, 0.0)
        clock.advance(5.1)
        state, _ = monitor.tick()
        self.assertEqual(state, USABLE)  # still held
        monitor.on_velocity(0.3, 0.0)
        clock.advance(0.1)
        state, _ = monitor.tick()
        self.assertEqual(state, LOST)  # now stale past the short timeout too

    def test_stationary_hold_tolerates_sub_threshold_velocity_noise(self):
        # Odometry noise below the configured threshold must still read as
        # "stationary" - matches probe_behavior.ProbeBehavior's own
        # threshold semantics.
        clock = FakeClock()
        monitor = make_monitor(clock=clock)
        monitor.request_reinit()
        monitor.on_particle_cloud(0.2)
        monitor.on_amcl_pose(0.1, 0.05)
        self.assertEqual(monitor.state, USABLE)
        monitor.on_velocity(0.01, 0.01)  # under the 0.02 default thresholds
        clock.advance(5.1)
        state, _ = monitor.tick()
        self.assertEqual(state, USABLE)

    def test_request_reinit_after_lost_returns_to_global(self):
        clock = FakeClock()
        monitor = make_monitor(clock=clock)
        monitor.request_reinit()
        monitor.on_particle_cloud(0.2)
        monitor.on_amcl_pose(0.1, 0.05)
        monitor.on_particle_cloud(0.8)
        monitor.on_particle_cloud(3.5)  # -> LOST
        clock.advance(10.1)
        _, should_reinit = monitor.tick()
        self.assertTrue(should_reinit)
        monitor.request_reinit()
        self.assertEqual(monitor.state, GLOBAL)


if __name__ == '__main__':
    unittest.main()
