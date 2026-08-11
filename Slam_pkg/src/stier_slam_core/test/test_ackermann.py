import math
import unittest

from stier_slam_core.ackermann import AckermannIntegrator, NonMonotonicTimeError


class AckermannIntegratorTest(unittest.TestCase):
    def test_straight_motion_integrates_distance_after_initial_sample(self):
        """A missing time delta must not advance the initial pose."""
        integrator = AckermannIntegrator(0.32, 0.6, 8.0)

        integrator.update(1.0, 1.0, 0.0)
        state = integrator.update(2.0, 1.0, 0.0)

        self.assertAlmostEqual(state.x, 1.0)
        self.assertAlmostEqual(state.y, 0.0)
        self.assertAlmostEqual(state.yaw, 0.0)
        self.assertAlmostEqual(state.linear_speed, 1.0)
        self.assertAlmostEqual(state.yaw_rate, 0.0)

    def test_turn_uses_midpoint_heading_and_ackermann_yaw_rate(self):
        """Using the final heading for translation would produce a different arc."""
        integrator = AckermannIntegrator(0.32, 0.6, 8.0)

        integrator.update(10.0, 1.0, 0.2)
        state = integrator.update(11.0, 1.0, 0.2)

        self.assertAlmostEqual(state.x, 0.95026, places=5)
        self.assertAlmostEqual(state.y, 0.31147, places=5)
        self.assertAlmostEqual(state.yaw, 0.63347, places=5)
        self.assertAlmostEqual(
            state.yaw_rate, 1.0 * math.tan(0.2) / 0.32, places=12
        )

    def test_reverse_motion_uses_signed_speed(self):
        """Replacing signed velocity with magnitude would move this pose forward."""
        integrator = AckermannIntegrator(0.32, 0.6, 8.0)

        integrator.update(3.0, -1.5, 0.0)
        state = integrator.update(5.0, -1.5, 0.0)

        self.assertAlmostEqual(state.x, -3.0)
        self.assertAlmostEqual(state.linear_speed, -1.5)

    def test_time_reversal_is_rejected(self):
        """Accepting an earlier sample would integrate an invalid negative delta."""
        integrator = AckermannIntegrator(0.32, 0.6, 8.0)

        integrator.update(1.0, 1.0, 0.0)
        integrator.update(2.0, 1.0, 0.0)

        with self.assertRaises(NonMonotonicTimeError):
            integrator.update(1.5, 1.0, 0.0)

    def test_reset_starts_a_new_epoch_from_the_origin(self):
        """A rosbag epoch reset must discard both prior time and prior pose."""
        integrator = AckermannIntegrator(0.32, 0.6, 8.0)
        integrator.update(90.0, 1.0, 0.0)
        prior = integrator.update(91.0, 1.0, 0.0)
        self.assertAlmostEqual(prior.x, 1.0)

        integrator.reset()
        restarted = integrator.update(10.0, -0.5, 0.0)

        self.assertEqual(
            (restarted.x, restarted.y, restarted.yaw),
            (0.0, 0.0, 0.0),
        )
        self.assertEqual(restarted.stamp, 10.0)
        self.assertEqual(restarted.linear_speed, -0.5)

    def test_out_of_range_speed_and_steering_are_rejected(self):
        """Silently clamping limits would conceal an invalid vehicle input."""
        integrator = AckermannIntegrator(0.32, 0.6, 8.0)

        with self.assertRaises(ValueError):
            integrator.update(1.0, 8.01, 0.0)
        with self.assertRaises(ValueError):
            integrator.update(1.0, 0.0, 0.61)


if __name__ == "__main__":
    unittest.main()
