"""Exercises SafetyMonitor's collision decision logic in isolation, with
synthetic LaserScan-shaped inputs (plain lists, no rclpy/message types) -
same style as rambla_control_panel's test_deadman.py.
"""
import math
import unittest

from rambla_safety.safety_monitor import SafetyMonitor


def make_ranges(value, count=360):
    return [value] * count


class FakeClock:
    def __init__(self, t=0.0):
        self.t = t

    def __call__(self):
        return self.t

    def advance(self, dt):
        self.t += dt


class TestSafetyMonitor(unittest.TestCase):

    def test_unblocked_with_no_input(self):
        monitor = SafetyMonitor()
        self.assertFalse(monitor.is_blocked())
        self.assertEqual(monitor.filter_cmd(0.5, 0.0), (0.5, 0.0))

    def test_clear_scan_does_not_block(self):
        monitor = SafetyMonitor(stop_distance_m=0.35)
        monitor.on_scan(make_ranges(5.0), angle_min=0.0, angle_increment=2 * math.pi / 360)
        self.assertFalse(monitor.is_blocked())

    def test_close_range_directly_ahead_blocks(self):
        monitor = SafetyMonitor(stop_distance_m=0.35, min_valid_range_m=0.3)
        ranges = make_ranges(5.0)
        ranges[0] = 0.32  # angle_min = 0 = straight ahead, above the configured floor
        monitor.on_scan(ranges, angle_min=0.0, angle_increment=2 * math.pi / 360)
        self.assertTrue(monitor.is_blocked())

    def test_close_range_behind_does_not_block(self):
        monitor = SafetyMonitor(stop_distance_m=0.35, front_arc_deg=30)
        ranges = make_ranges(5.0)
        ranges[180] = 0.32  # directly behind (pi rad from front)
        monitor.on_scan(ranges, angle_min=0.0, angle_increment=2 * math.pi / 360)
        self.assertFalse(monitor.is_blocked())

    def test_close_range_at_wraparound_seam_blocks(self):
        # Index 359 is one increment before the 2pi wrap, i.e. just to one
        # side of straight-ahead - must be treated as "front", not ignored
        # because it's near the end of the array.
        monitor = SafetyMonitor(stop_distance_m=0.35, front_arc_deg=30)
        ranges = make_ranges(5.0)
        ranges[359] = 0.32
        monitor.on_scan(ranges, angle_min=0.0, angle_increment=2 * math.pi / 360)
        self.assertTrue(monitor.is_blocked())

    def test_range_exactly_at_threshold_does_not_block(self):
        monitor = SafetyMonitor(stop_distance_m=0.35)
        ranges = make_ranges(5.0)
        ranges[0] = 0.35
        monitor.on_scan(ranges, angle_min=0.0, angle_increment=2 * math.pi / 360)
        self.assertFalse(monitor.is_blocked())

    def test_nan_and_zero_ranges_ignored(self):
        monitor = SafetyMonitor(stop_distance_m=0.35)
        ranges = make_ranges(5.0)
        ranges[0] = float('nan')
        ranges[1] = 0.0
        monitor.on_scan(ranges, angle_min=0.0, angle_increment=2 * math.pi / 360)
        self.assertFalse(monitor.is_blocked())

    def test_close_range_below_valid_floor_ignored(self):
        # Ranges below min_valid_range_m are treated as invalid (originally
        # added for a self-detection artifact that M1's sensor-placement fix
        # resolved at the geometry level - see DEFAULT_MIN_VALID_RANGE_M's
        # docstring; this floor remains as a generic near-range guard, and
        # this test exercises that filtering logic in the abstract).
        monitor = SafetyMonitor(stop_distance_m=0.35, min_valid_range_m=0.3)
        ranges = make_ranges(5.0)
        ranges[0] = 0.25
        ranges[1] = 0.26
        ranges[2] = 0.27
        monitor.on_scan(ranges, angle_min=0.0, angle_increment=2 * math.pi / 360)
        self.assertFalse(monitor.is_blocked())

    def test_real_obstacle_beyond_valid_floor_blocks(self):
        monitor = SafetyMonitor(stop_distance_m=0.35, min_valid_range_m=0.3)
        ranges = make_ranges(5.0)
        ranges[0] = 0.25  # below the valid floor, ignored
        ranges[1] = 0.31  # real obstacle, closer than stop_distance_m
        monitor.on_scan(ranges, angle_min=0.0, angle_increment=2 * math.pi / 360)
        self.assertTrue(monitor.is_blocked())

    def test_bumper_left_contact_blocks(self):
        monitor = SafetyMonitor()
        monitor.on_bumper_left(True)
        self.assertTrue(monitor.is_blocked())

    def test_bumper_right_contact_blocks(self):
        monitor = SafetyMonitor()
        monitor.on_bumper_right(True)
        self.assertTrue(monitor.is_blocked())

    def test_bumper_contact_clears_when_released(self):
        monitor = SafetyMonitor()
        monitor.on_bumper_left(True)
        self.assertTrue(monitor.is_blocked())
        monitor.on_bumper_left(False)
        self.assertFalse(monitor.is_blocked())

    def test_bumper_contact_expires_without_a_release_message(self):
        # gz-sim's real contact sensor never sends an explicit "contact
        # ended" message (see DEFAULT_BUMPER_CONTACT_TIMEOUT_S) - so a
        # stuck-forever block must clear on its own once the timeout
        # elapses with no fresh True reading, not just on an explicit
        # on_bumper_left(False) call.
        clock = FakeClock()
        monitor = SafetyMonitor(bumper_contact_timeout_s=0.25, clock=clock)
        monitor.on_bumper_left(True)
        self.assertTrue(monitor.is_blocked())
        clock.advance(0.1)
        self.assertTrue(monitor.is_blocked())
        clock.advance(0.2)
        self.assertFalse(monitor.is_blocked())

    def test_repeated_contact_messages_keep_extending_the_deadline(self):
        clock = FakeClock()
        monitor = SafetyMonitor(bumper_contact_timeout_s=0.25, clock=clock)
        monitor.on_bumper_left(True)
        clock.advance(0.2)
        monitor.on_bumper_left(True)  # sustained contact, e.g. next 50Hz message
        clock.advance(0.2)
        self.assertTrue(monitor.is_blocked())

    def test_filter_cmd_zeroes_forward_linear_when_blocked(self):
        monitor = SafetyMonitor()
        monitor.on_bumper_left(True)
        self.assertEqual(monitor.filter_cmd(0.5, 0.3), (0.0, 0.3))

    def test_filter_cmd_allows_reverse_when_blocked(self):
        monitor = SafetyMonitor()
        monitor.on_bumper_left(True)
        self.assertEqual(monitor.filter_cmd(-0.3, 0.0), (-0.3, 0.0))

    def test_filter_cmd_allows_rotation_when_blocked(self):
        monitor = SafetyMonitor()
        monitor.on_bumper_left(True)
        self.assertEqual(monitor.filter_cmd(0.0, 0.8), (0.0, 0.8))

    def test_filter_cmd_passes_through_when_clear(self):
        monitor = SafetyMonitor()
        self.assertEqual(monitor.filter_cmd(0.5, -0.2), (0.5, -0.2))


if __name__ == '__main__':
    unittest.main()
