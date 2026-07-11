"""Exercises ControlAuthority's AUTO/MANUAL arbitration logic in isolation,
with a fake clock - same style as rambla_control_panel's test_deadman.py
and this package's test_safety_monitor.py.
"""
import unittest

from rambla_safety.control_authority import AUTO, MANUAL, ControlAuthority


class FakeClock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class TestControlAuthority(unittest.TestCase):

    def test_defaults_to_auto_with_zero_command(self):
        authority = ControlAuthority(clock=FakeClock())
        self.assertEqual(authority.tick(), (AUTO, (0.0, 0.0)))

    def test_auto_cmd_passes_through_in_auto(self):
        authority = ControlAuthority(clock=FakeClock())
        authority.on_auto_cmd(0.15, 0.0)
        self.assertEqual(authority.tick(), (AUTO, (0.15, 0.0)))

    def test_manual_cmd_takes_over_immediately(self):
        authority = ControlAuthority(clock=FakeClock())
        authority.on_auto_cmd(0.15, 0.0)
        authority.on_manual_cmd(0.5, -0.3)
        self.assertEqual(authority.tick(), (MANUAL, (0.5, -0.3)))

    def test_auto_cmd_frozen_while_manual(self):
        authority = ControlAuthority(clock=FakeClock())
        authority.on_manual_cmd(0.5, -0.3)
        authority.on_auto_cmd(0.9, 0.9)  # traversal kept publishing, ignored
        mode, (linear, angular) = authority.tick()
        self.assertEqual(mode, MANUAL)
        self.assertEqual((linear, angular), (0.5, -0.3))

    def test_auto_cmd_cleared_on_takeover_not_replayed_immediately(self):
        clock = FakeClock()
        authority = ControlAuthority(clock=clock, manual_timeout_s=0.4)
        authority.on_auto_cmd(0.9, 0.0)  # last auto command before takeover
        authority.on_manual_cmd(0.0, 0.0)
        clock.advance(0.41)  # manual releases without ever sending another cmd
        mode, (linear, angular) = authority.tick()
        self.assertEqual(mode, AUTO)
        self.assertEqual(
            (linear, angular), (0.0, 0.0),
            'stale pre-takeover auto command must not replay on release')

    def test_manual_times_out_back_to_auto(self):
        clock = FakeClock()
        authority = ControlAuthority(clock=clock, manual_timeout_s=0.4)
        authority.on_manual_cmd(0.5, 0.0)
        clock.advance(0.41)
        self.assertEqual(authority.tick(), (AUTO, (0.0, 0.0)))

    def test_manual_stays_engaged_within_timeout(self):
        clock = FakeClock()
        authority = ControlAuthority(clock=clock, manual_timeout_s=0.4)
        authority.on_manual_cmd(0.5, 0.0)
        clock.advance(0.1)
        self.assertEqual(authority.tick(), (MANUAL, (0.5, 0.0)))

    def test_auto_resumes_with_fresh_command_after_manual_release(self):
        clock = FakeClock()
        authority = ControlAuthority(clock=clock, manual_timeout_s=0.4)
        authority.on_manual_cmd(0.5, 0.0)
        clock.advance(0.41)
        authority.tick()  # times out to AUTO
        authority.on_auto_cmd(0.15, 0.0)  # traversal resumes publishing
        self.assertEqual(authority.tick(), (AUTO, (0.15, 0.0)))

    def test_manual_re_engages_immediately_after_a_prior_release(self):
        clock = FakeClock()
        authority = ControlAuthority(clock=clock, manual_timeout_s=0.4)
        authority.on_manual_cmd(0.5, 0.0)
        clock.advance(0.41)
        authority.tick()  # released back to AUTO
        authority.on_manual_cmd(0.2, 0.2)
        self.assertEqual(authority.tick(), (MANUAL, (0.2, 0.2)))


if __name__ == '__main__':
    unittest.main()
