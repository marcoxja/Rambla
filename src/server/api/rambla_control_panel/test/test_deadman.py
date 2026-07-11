"""Deliberately not a general test framework: exercises DeadmanTimer's
timeout logic in isolation with a fake clock, per this repo's preference
for small targeted tests (see rambla_sim/test/test_smoke_topics.py).
"""
import unittest

from rambla_control_panel.deadman import DeadmanTimer


class FakeClock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class TestDeadmanTimer(unittest.TestCase):

    def test_returns_zero_before_any_command(self):
        clock = FakeClock()
        timer = DeadmanTimer(timeout_s=0.3, clock=clock)
        self.assertEqual(timer.command(), (0.0, 0.0))

    def test_returns_last_command_while_fresh(self):
        clock = FakeClock()
        timer = DeadmanTimer(timeout_s=0.3, clock=clock)
        timer.on_command(0.5, -0.2)
        clock.advance(0.1)
        self.assertEqual(timer.command(), (0.5, -0.2))

    def test_trips_to_zero_after_timeout(self):
        clock = FakeClock()
        timer = DeadmanTimer(timeout_s=0.3, clock=clock)
        timer.on_command(0.5, -0.2)
        clock.advance(0.31)
        self.assertEqual(timer.command(), (0.0, 0.0))
        self.assertTrue(timer.is_tripped)

    def test_stays_zero_after_tripping_until_new_command(self):
        clock = FakeClock()
        timer = DeadmanTimer(timeout_s=0.3, clock=clock)
        timer.on_command(0.5, -0.2)
        clock.advance(0.31)
        timer.command()
        clock.advance(0.01)
        self.assertEqual(timer.command(), (0.0, 0.0))

    def test_recovers_on_new_command_after_tripping(self):
        clock = FakeClock()
        timer = DeadmanTimer(timeout_s=0.3, clock=clock)
        timer.on_command(0.5, -0.2)
        clock.advance(0.31)
        timer.command()
        timer.on_command(0.1, 0.1)
        self.assertEqual(timer.command(), (0.1, 0.1))
        self.assertFalse(timer.is_tripped)

    def test_disconnect_zeroes_immediately(self):
        clock = FakeClock()
        timer = DeadmanTimer(timeout_s=0.3, clock=clock)
        timer.on_command(0.5, -0.2)
        timer.on_disconnect()
        self.assertEqual(timer.command(), (0.0, 0.0))
        self.assertTrue(timer.is_tripped)

    def test_not_active_before_any_command(self):
        clock = FakeClock()
        timer = DeadmanTimer(timeout_s=0.3, clock=clock)
        self.assertFalse(timer.is_active)

    def test_active_while_fresh(self):
        clock = FakeClock()
        timer = DeadmanTimer(timeout_s=0.3, clock=clock)
        timer.on_command(0.0, 0.0)
        clock.advance(0.1)
        timer.command()
        self.assertTrue(timer.is_active)

    def test_not_active_after_timeout(self):
        clock = FakeClock()
        timer = DeadmanTimer(timeout_s=0.3, clock=clock)
        timer.on_command(0.5, -0.2)
        clock.advance(0.31)
        timer.command()
        self.assertFalse(timer.is_active)

    def test_not_active_after_disconnect(self):
        clock = FakeClock()
        timer = DeadmanTimer(timeout_s=0.3, clock=clock)
        timer.on_command(0.5, -0.2)
        timer.on_disconnect()
        self.assertFalse(timer.is_active)

    def test_active_again_after_recovering_from_timeout(self):
        clock = FakeClock()
        timer = DeadmanTimer(timeout_s=0.3, clock=clock)
        timer.on_command(0.5, -0.2)
        clock.advance(0.31)
        timer.command()
        timer.on_command(0.1, 0.1)
        self.assertTrue(timer.is_active)


if __name__ == '__main__':
    unittest.main()
