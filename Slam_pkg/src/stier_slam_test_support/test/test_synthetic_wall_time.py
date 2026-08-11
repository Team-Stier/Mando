#!/usr/bin/env python3
"""Wall-time mode must timestamp synthetic data near the current ROS epoch."""

import importlib.util
import math
from pathlib import Path
import threading
import time
import unittest

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "synthetic_sensor_node.py"
SPEC = importlib.util.spec_from_file_location("synthetic_sensor_node_under_test", SCRIPT_PATH)
SYNTHETIC_MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(SYNTHETIC_MODULE)


class SyntheticWallTimeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("synthetic_wall_time_contract_test", anonymous=True)

    def test_publish_rate_is_finite_positive_and_bounded(self):
        for invalid in (0.0, -1.0, math.nan, 1000.01, True):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    SYNTHETIC_MODULE._validated_publish_rate(invalid)
        self.assertEqual(SYNTHETIC_MODULE._validated_publish_rate(20.0), 20.0)

    def test_cycle_is_even_and_boolean_parameters_are_strict(self):
        for invalid in (1, 3, 5, True, 4.0):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    SYNTHETIC_MODULE._validated_cycle_samples(invalid)
        self.assertEqual(SYNTHETIC_MODULE._validated_cycle_samples(40), 40)
        with self.assertRaises(ValueError):
            SYNTHETIC_MODULE._validated_boolean("false", "~publish_clock")
        self.assertFalse(SYNTHETIC_MODULE._validated_boolean(False, "~publish_clock"))

    def test_trajectory_parameters_reject_unsafe_geometry_and_accept_boundary(self):
        self.assertEqual(
            SYNTHETIC_MODULE._validated_trajectory_parameters(20.0, 40),
            (20.0, 40),
        )
        half_cycle = 40 // 2
        maximum_safe_pose = (
            SYNTHETIC_MODULE.MAP_FREE_X_MAX_M
            - SYNTHETIC_MODULE.TEMPORARY_OBSTACLE_RANGE_M
            - SYNTHETIC_MODULE.TEMPORARY_OBSTACLE_FREE_MARGIN_M
        )
        boundary_rate = (
            (half_cycle - 1) * SYNTHETIC_MODULE.SYNTHETIC_SPEED_MPS
            / maximum_safe_pose
        )
        self.assertEqual(
            SYNTHETIC_MODULE._validated_trajectory_parameters(boundary_rate, 40),
            (boundary_rate, 40),
        )
        self.assertEqual(
            SYNTHETIC_MODULE._validated_trajectory_parameters(1000.0, 7802),
            (1000.0, 7802),
        )

        invalid_pairs = (
            (1.0, 40),
            (20.0, 4000),
            (20.0, 10 ** 400),
            (1000.0, 7804),
            (boundary_rate * (1.0 - 1.0e-6), 40),
            (math.nan, 40),
            (True, 40),
            (20.0, True),
            (20.0, 40.0),
        )
        for publish_rate_hz, cycle_samples in invalid_pairs:
            with self.subTest(
                    publish_rate_hz=publish_rate_hz,
                    cycle_samples=cycle_samples):
                with self.assertRaises(ValueError):
                    SYNTHETIC_MODULE._validated_trajectory_parameters(
                        publish_rate_hz, cycle_samples
                    )

    def test_node_init_rejects_unsafe_parameter_combinations_before_publishers(self):
        for publish_rate_hz, cycle_samples in ((1.0, 40), (20.0, 4000)):
            with self.subTest(
                    publish_rate_hz=publish_rate_hz,
                    cycle_samples=cycle_samples):
                rospy.set_param("~publish_rate_hz", publish_rate_hz)
                rospy.set_param("~cycle_samples", cycle_samples)
                with self.assertRaisesRegex(
                        ValueError, "unsafe synthetic trajectory geometry"):
                    SYNTHETIC_MODULE.SyntheticSensorNode()
        rospy.delete_param("~publish_rate_hz")
        rospy.delete_param("~cycle_samples")

    def test_every_allowed_phase_and_beam_stays_inside_scan_range(self):
        maximum_safe_pose = (
            SYNTHETIC_MODULE.MAP_FREE_X_MAX_M
            - SYNTHETIC_MODULE.TEMPORARY_OBSTACLE_RANGE_M
            - SYNTHETIC_MODULE.TEMPORARY_OBSTACLE_FREE_MARGIN_M
        )
        boundary_rate = (
            (40 // 2 - 1) * SYNTHETIC_MODULE.SYNTHETIC_SPEED_MPS
            / maximum_safe_pose
        )
        for publish_rate_hz, cycle_samples in ((20.0, 40), (boundary_rate, 40)):
            with self.subTest(
                    publish_rate_hz=publish_rate_hz,
                    cycle_samples=cycle_samples):
                SYNTHETIC_MODULE._validated_trajectory_parameters(
                    publish_rate_hz, cycle_samples
                )
                for phase in range(cycle_samples):
                    pose_x = SYNTHETIC_MODULE._synthetic_pose_x(
                        phase, publish_rate_hz, cycle_samples
                    )
                    self.assertGreaterEqual(pose_x, 0.0)
                    for obstacle_active in (False, True):
                        ranges = SYNTHETIC_MODULE._synthetic_scan_ranges(
                            pose_x, obstacle_active
                        )
                        self.assertEqual(
                            len(ranges), SYNTHETIC_MODULE.SCAN_BEAM_COUNT
                        )
                        self.assertTrue(all(
                            math.isfinite(distance)
                            and SYNTHETIC_MODULE.SCAN_RANGE_MIN_M <= distance
                            <= SYNTHETIC_MODULE.SCAN_RANGE_MAX_M
                            for distance in ranges
                        ))
                        angle_increment = (
                            2.0 * math.pi / SYNTHETIC_MODULE.SCAN_BEAM_COUNT
                        )
                        for beam_index, distance in enumerate(ranges):
                            angle = -math.pi + beam_index * angle_increment
                            endpoint_x = pose_x + distance * math.cos(angle)
                            endpoint_y = distance * math.sin(angle)
                            self.assertLessEqual(
                                abs(endpoint_x),
                                SYNTHETIC_MODULE.STATIC_WALL_X_M + 1.0e-9,
                            )
                            self.assertLessEqual(
                                abs(endpoint_y),
                                SYNTHETIC_MODULE.STATIC_WALL_Y_M + 1.0e-9,
                            )
                            if (
                                    obstacle_active
                                    and abs(SYNTHETIC_MODULE._wrapped_angle(angle))
                                    <= SYNTHETIC_MODULE.TEMPORARY_OBSTACLE_HALF_ANGLE_RAD):
                                self.assertLessEqual(
                                    endpoint_x,
                                    SYNTHETIC_MODULE.MAP_FREE_X_MAX_M
                                    - SYNTHETIC_MODULE.TEMPORARY_OBSTACLE_FREE_MARGIN_M
                                    + 1.0e-9,
                                )
                            if not obstacle_active:
                                self.assertTrue(
                                    math.isclose(
                                        abs(endpoint_x),
                                        SYNTHETIC_MODULE.STATIC_WALL_X_M,
                                        abs_tol=1.0e-9,
                                    )
                                    or math.isclose(
                                        abs(endpoint_y),
                                        SYNTHETIC_MODULE.STATIC_WALL_Y_M,
                                        abs_tol=1.0e-9,
                                    )
                                )

    def test_wall_schedule_skips_overrun_and_recovers_without_time_reversal(self):
        next_index, next_deadline = SYNTHETIC_MODULE._next_wall_schedule(
            current_sample_index=0,
            monotonic_origin_sec=100.0,
            previous_monotonic_sec=100.0,
            current_monotonic_sec=100.3,
            period_sec=0.05,
        )
        self.assertEqual(next_index, 6)
        self.assertAlmostEqual(next_deadline, 100.3)

        recovered_index, recovered_deadline = SYNTHETIC_MODULE._next_wall_schedule(
            current_sample_index=next_index,
            monotonic_origin_sec=100.0,
            previous_monotonic_sec=100.3,
            current_monotonic_sec=100.301,
            period_sec=0.05,
        )
        self.assertEqual(recovered_index, 7)
        self.assertAlmostEqual(recovered_deadline, 100.35)
        self.assertGreater(recovered_index, next_index)
        self.assertGreater(recovered_deadline, next_deadline)

        invalid_calls = (
            dict(
                current_sample_index=6,
                monotonic_origin_sec=100.0,
                previous_monotonic_sec=100.3,
                current_monotonic_sec=100.2,
                period_sec=0.05,
            ),
            dict(
                current_sample_index=6,
                monotonic_origin_sec=100.0,
                previous_monotonic_sec=100.3,
                current_monotonic_sec=math.nan,
                period_sec=0.05,
            ),
            dict(
                current_sample_index=True,
                monotonic_origin_sec=100.0,
                previous_monotonic_sec=100.0,
                current_monotonic_sec=100.0,
                period_sec=0.05,
            ),
        )
        for arguments in invalid_calls:
            with self.subTest(arguments=arguments):
                with self.assertRaises(ValueError):
                    SYNTHETIC_MODULE._next_wall_schedule(**arguments)

    def test_wall_time_mode_uses_current_monotonic_ros_epoch(self):
        condition = threading.Condition()
        samples = []

        def callback(message):
            with condition:
                samples.append((message, rospy.Time.now().to_sec(), time.monotonic()))
                condition.notify_all()

        subscriber = rospy.Subscriber(
            "/slam/input/vehicle_speed",
            TwistWithCovarianceStamped,
            callback,
            queue_size=50,
        )
        deadline = time.monotonic() + 5.0
        with condition:
            while len(samples) < 40 and time.monotonic() < deadline:
                condition.wait(timeout=min(0.1, deadline - time.monotonic()))
        subscriber.unregister()

        self.assertGreaterEqual(len(samples), 40)
        stamps = [sample[0].header.stamp.to_nsec() for sample in samples[:40]]
        self.assertTrue(all(later > earlier for earlier, later in zip(stamps, stamps[1:])))
        self.assertTrue(all(
            (later - earlier) % 50000000 == 0
            for earlier, later in zip(stamps, stamps[1:])
        ))
        self.assertGreaterEqual(
            (len(samples[:40]) - 1)
            / (samples[39][2] - samples[0][2]),
            18.0,
        )
        for message, receipt_ros_sec, _ in samples[:40]:
            self.assertLess(abs(message.header.stamp.to_sec() - receipt_ros_sec), 0.10)

        published_types = dict(rospy.get_published_topics("/"))
        self.assertNotIn("/clock", published_types)


if __name__ == "__main__":
    import rostest

    rostest.rosrun(
        "stier_slam_test_support",
        "synthetic_wall_time",
        SyntheticWallTimeTest,
    )
