#!/usr/bin/env python3
"""Wheel encoder 속도 계산 계약."""

import math
import unittest

from stier_slam_core.wheel_encoder import WheelEncoderEstimator


class WheelEncoderEstimatorTest(unittest.TestCase):
    def setUp(self):
        self.estimator = WheelEncoderEstimator(
            wheel_radius_m=0.2,
            left_joint_name="rear_left_wheel",
            right_joint_name="rear_right_wheel",
            left_sign=1.0,
            right_sign=1.0,
            max_abs_wheel_speed_rad_s=20.0,
        )

    def test_uses_named_joint_velocity_and_wheel_radius(self):
        speed = self.estimator.update(
            10.0,
            ["rear_right_wheel", "rear_left_wheel"],
            [],
            [4.0, 2.0],
        )

        self.assertAlmostEqual(speed, 0.6)

    def test_falls_back_to_position_difference_after_baseline(self):
        first = self.estimator.update(
            10.0,
            ["rear_left_wheel", "rear_right_wheel"],
            [1.0, 2.0],
            [],
        )
        second = self.estimator.update(
            10.5,
            ["rear_left_wheel", "rear_right_wheel"],
            [2.0, 3.5],
            [],
        )

        self.assertIsNone(first)
        self.assertAlmostEqual(second, 0.5)

    def test_applies_configured_wheel_direction_signs(self):
        estimator = WheelEncoderEstimator(
            0.2, "left", "right", -1.0, 1.0, 20.0
        )

        speed = estimator.update(1.0, ["left", "right"], [], [-2.0, 2.0])

        self.assertAlmostEqual(speed, 0.4)

    def test_velocity_only_sample_invalidates_old_position_baseline(self):
        names = ["rear_left_wheel", "rear_right_wheel"]
        self.assertIsNone(self.estimator.update(1.0, names, [1.0, 1.0], []))
        self.assertAlmostEqual(self.estimator.update(2.0, names, [], [2.0, 2.0]), 0.4)

        self.assertIsNone(self.estimator.update(3.0, names, [5.0, 5.0], []))
        self.assertAlmostEqual(
            self.estimator.update(4.0, names, [6.0, 6.0], []),
            0.2,
        )

    def test_rejects_duplicate_or_missing_joint_names(self):
        for names in (["rear_left_wheel"], ["rear_left_wheel", "rear_left_wheel"]):
            with self.subTest(names=names), self.assertRaises(ValueError):
                self.estimator.update(1.0, names, [], [1.0] * len(names))

    def test_rejects_non_monotonic_time_non_finite_values_and_excess_speed(self):
        self.estimator.update(
            1.0, ["rear_left_wheel", "rear_right_wheel"], [], [1.0, 1.0]
        )
        invalid_samples = (
            (1.0, [1.0, 1.0]),
            (2.0, [math.nan, 1.0]),
            (2.0, [21.0, 1.0]),
        )
        for stamp, velocities in invalid_samples:
            with self.subTest(stamp=stamp, velocities=velocities), self.assertRaises(ValueError):
                self.estimator.update(
                    stamp,
                    ["rear_left_wheel", "rear_right_wheel"],
                    [],
                    velocities,
                )


if __name__ == "__main__":
    unittest.main()
