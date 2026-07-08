"""Exercises WanderBehavior's reactive steering decisions in isolation,
with synthetic LaserScan-shaped inputs - same style as rambla_safety's
test_safety_monitor.py.
"""
import math
import unittest

from rambla_traversal.wander_behavior import WanderBehavior


def make_ranges(value, count=360):
    return [value] * count


class TestWanderBehavior(unittest.TestCase):

    def test_drives_forward_when_clear(self):
        behavior = WanderBehavior(forward_speed=0.15, turn_threshold_m=0.6)
        ranges = make_ranges(5.0)
        linear, angular = behavior.next_command(
            ranges, angle_min=0.0, angle_increment=2 * math.pi / 360)
        self.assertEqual(linear, 0.15)
        self.assertEqual(angular, 0.0)

    def test_turns_when_front_is_blocked(self):
        behavior = WanderBehavior(turn_threshold_m=0.6)
        ranges = make_ranges(5.0)
        ranges[0] = 0.3  # directly ahead, inside turn_threshold_m
        linear, angular = behavior.next_command(
            ranges, angle_min=0.0, angle_increment=2 * math.pi / 360)
        self.assertEqual(linear, 0.0)
        self.assertNotEqual(angular, 0.0)

    def test_turns_toward_more_open_side_left(self):
        behavior = WanderBehavior(turn_threshold_m=0.6)
        ranges = make_ranges(5.0)
        ranges[0] = 0.3  # front blocked
        # Right side (angle -90deg, index ~270) tight, left side open.
        for i in range(250, 291):
            ranges[i] = 0.3
        _, angular = behavior.next_command(
            ranges, angle_min=0.0, angle_increment=2 * math.pi / 360)
        self.assertGreater(angular, 0.0)  # positive = turn left

    def test_turns_toward_more_open_side_right(self):
        behavior = WanderBehavior(turn_threshold_m=0.6)
        ranges = make_ranges(5.0)
        ranges[0] = 0.3  # front blocked
        # Left side (angle +90deg, index ~90) tight, right side open.
        for i in range(70, 111):
            ranges[i] = 0.3
        _, angular = behavior.next_command(
            ranges, angle_min=0.0, angle_increment=2 * math.pi / 360)
        self.assertLess(angular, 0.0)  # negative = turn right

    def test_resumes_forward_once_front_clears(self):
        behavior = WanderBehavior(turn_threshold_m=0.6)
        blocked_ranges = make_ranges(5.0)
        blocked_ranges[0] = 0.3
        behavior.next_command(blocked_ranges, angle_min=0.0, angle_increment=2 * math.pi / 360)

        clear_ranges = make_ranges(5.0)
        linear, angular = behavior.next_command(
            clear_ranges, angle_min=0.0, angle_increment=2 * math.pi / 360)
        self.assertGreater(linear, 0.0)
        self.assertEqual(angular, 0.0)

    def test_keeps_turning_same_direction_until_clear(self):
        # Once a turn direction is chosen, it shouldn't flip-flop every
        # tick while still blocked, even if relative clearance readings
        # wobble slightly - avoids oscillating in place.
        behavior = WanderBehavior(turn_threshold_m=0.6)
        ranges = make_ranges(5.0)
        ranges[0] = 0.3
        for i in range(250, 291):
            ranges[i] = 0.3  # right tighter -> should pick left (positive)
        _, first_angular = behavior.next_command(
            ranges, angle_min=0.0, angle_increment=2 * math.pi / 360)

        # Second tick: still blocked, but now left/right clearance is equal -
        # a naive re-evaluation might pick right instead.
        ranges2 = make_ranges(5.0)
        ranges2[0] = 0.3
        _, second_angular = behavior.next_command(
            ranges2, angle_min=0.0, angle_increment=2 * math.pi / 360)
        self.assertEqual(first_angular, second_angular)

    def test_handles_empty_ranges(self):
        behavior = WanderBehavior()
        linear, angular = behavior.next_command([], angle_min=0.0, angle_increment=0.0)
        self.assertEqual(linear, behavior._forward_speed)
        self.assertEqual(angular, 0.0)

    def test_self_detection_range_does_not_trigger_turn(self):
        # Confirmed live in sim: the LiDAR sees its own front bumper at
        # ~0.24-0.27m even with nothing else nearby - must drive forward,
        # not spin in place at spawn (see WanderBehavior's
        # DEFAULT_MIN_VALID_RANGE_M docstring).
        behavior = WanderBehavior(turn_threshold_m=0.6, min_valid_range_m=0.3)
        ranges = make_ranges(5.0)
        ranges[0] = 0.25
        ranges[1] = 0.26
        ranges[2] = 0.27
        linear, angular = behavior.next_command(
            ranges, angle_min=0.0, angle_increment=2 * math.pi / 360)
        self.assertGreater(linear, 0.0)
        self.assertEqual(angular, 0.0)


if __name__ == '__main__':
    unittest.main()
