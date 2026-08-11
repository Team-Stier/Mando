import importlib.util
import math
from pathlib import Path
import sys
import threading
import types
import unittest


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "vehicle_odometry_node.py"


class _Object:
    pass


class _Stamp:
    def __init__(self, seconds, nanoseconds=None):
        self._seconds = seconds
        self._nanoseconds = (
            int(round(seconds * 1.0e9))
            if nanoseconds is None
            else nanoseconds
        )

    def to_sec(self):
        return self._seconds

    def to_nsec(self):
        return self._nanoseconds


def _stamp_from_sec(seconds):
    """Match rospy.Time.from_sec's float truncation for regression coverage."""
    whole_seconds = int(seconds)
    nanoseconds = int((seconds - whole_seconds) * 1.0e9)
    return _Stamp(
        seconds,
        nanoseconds=whole_seconds * 1000000000 + nanoseconds,
    )


class _TwistWithCovarianceStamped:
    def __init__(self, stamp, speed, covariance):
        self.header = _Object()
        self.header.stamp = _Stamp(stamp)
        self.twist = _Object()
        self.twist.twist = _Object()
        self.twist.twist.linear = _Object()
        self.twist.twist.linear.x = speed
        self.twist.covariance = covariance


class _Odometry:
    def __init__(self):
        self.header = _Object()
        self.pose = _Object()
        self.pose.pose = _Object()
        self.pose.pose.position = _Object()
        self.pose.pose.orientation = _Object()
        self.twist = _Object()
        self.twist.twist = _Object()
        self.twist.twist.linear = _Object()
        self.twist.twist.angular = _Object()


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Ros:
    def __init__(self):
        self.subscriptions = []
        self.publisher = _Publisher()
        self.warnings = []
        self.now_sec = 1.0
        self.now_values = None
        self.params = {
            "/stier_slam/topics/inputs/vehicle_speed": "/slam/input/vehicle_speed",
            "/stier_slam/topics/inputs/steering_angle": "/slam/input/steering_angle",
            "/stier_slam/topics/processed/wheel_odometry": "/slam/odometry/wheel",
            "~wheelbase_m": 0.32,
            "~max_abs_steering_rad": 0.6,
            "~max_abs_speed_mps": 8.0,
            "~control_pair_timeout_sec": 0.10,
            "~pose_covariance": [0.0] * 36,
            "~twist_covariance": [0.75] * 36,
        }

    def get_param(self, name, default=None):
        return self.params.get(name, default)

    def Publisher(self, *args, **kwargs):
        return self.publisher

    def Subscriber(self, topic, message_type, callback, **kwargs):
        self.subscriptions.append((topic, message_type, callback))

    def logwarn(self, *args):
        self.warnings.append(args)

    def logwarn_throttle(self, *args):
        self.warnings.append(args)

    def now(self):
        if self.now_values is not None:
            return _Stamp(next(self.now_values))
        return _Stamp(self.now_sec)


class _ReverseAcquisitionLock:
    """Force the older callback to enter only after the newer callback exits."""

    def __init__(self):
        self._lock = threading.RLock()
        self.older_waiting = threading.Event()
        self.newer_done = threading.Event()

    def __enter__(self):
        if threading.current_thread().name == "older_receipt":
            self.older_waiting.set()
            if not self.newer_done.wait(timeout=2.0):
                raise RuntimeError("newer callback did not acquire first")
        self._lock.acquire()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self._lock.release()
        if threading.current_thread().name == "newer_receipt":
            self.newer_done.set()


class VehicleOdometryNodeTest(unittest.TestCase):
    def setUp(self):
        self._saved_modules = {
            name: sys.modules.get(name)
            for name in ("rospy", "geometry_msgs", "geometry_msgs.msg", "nav_msgs", "nav_msgs.msg",
                         "std_msgs", "std_msgs.msg", "vehicle_odometry_node_under_test")
        }
        self.ros = _Ros()
        rospy = types.ModuleType("rospy")
        rospy.get_param = self.ros.get_param
        rospy.Publisher = self.ros.Publisher
        rospy.Subscriber = self.ros.Subscriber
        rospy.logwarn = self.ros.logwarn
        rospy.logwarn_throttle = self.ros.logwarn_throttle
        rospy.Time = types.SimpleNamespace(
            from_sec=_stamp_from_sec,
            now=self.ros.now,
        )
        geometry_msgs = types.ModuleType("geometry_msgs")
        geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
        geometry_msgs_msg.TwistWithCovarianceStamped = _TwistWithCovarianceStamped
        nav_msgs = types.ModuleType("nav_msgs")
        nav_msgs_msg = types.ModuleType("nav_msgs.msg")
        nav_msgs_msg.Odometry = _Odometry
        std_msgs = types.ModuleType("std_msgs")
        std_msgs_msg = types.ModuleType("std_msgs.msg")
        std_msgs_msg.Float64 = type("Float64", (), {})
        sys.modules.update({
            "rospy": rospy,
            "geometry_msgs": geometry_msgs,
            "geometry_msgs.msg": geometry_msgs_msg,
            "nav_msgs": nav_msgs,
            "nav_msgs.msg": nav_msgs_msg,
            "std_msgs": std_msgs,
            "std_msgs.msg": std_msgs_msg,
        })
        spec = importlib.util.spec_from_file_location("vehicle_odometry_node_under_test", SCRIPT_PATH)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.node = self.module.VehicleOdometryNode()

    def tearDown(self):
        for name, module in self._saved_modules.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

    def _set_fresh_steering(self):
        self.node._on_steering(types.SimpleNamespace(data=0.0))

    def _speed_message(self, stamp, speed=1.0, covariance=None):
        return _TwistWithCovarianceStamped(stamp, speed, covariance or [0.25] * 36)

    def _at(self, receipt_time):
        self.ros.now_sec = receipt_time

    def test_subscribes_to_stamped_twist_vehicle_speed(self):
        """Subscribing to Float64 would reject compliant normalized speed publishers."""
        vehicle_speed = next(item for item in self.ros.subscriptions if item[0] == "/slam/input/vehicle_speed")

        self.assertIs(vehicle_speed[1], _TwistWithCovarianceStamped)

    def test_uses_central_topic_parameters_for_subscribers(self):
        self.ros.params["/stier_slam/topics/inputs/vehicle_speed"] = "/custom/speed"
        self.ros.params["/stier_slam/topics/inputs/steering_angle"] = "/custom/steering"

        self.module.VehicleOdometryNode()

        self.assertEqual([item[0] for item in self.ros.subscriptions[-2:]], [
            "/custom/steering", "/custom/speed",
        ])

    def test_publishes_source_stamp_and_configured_twist_covariance(self):
        """Letting sensor covariance override YAML would change the odometry contract."""
        covariance = [0.25] * 36
        self._set_fresh_steering()
        self.node._on_speed(self._speed_message(42.5, covariance=covariance))

        published = self.ros.publisher.messages[0]
        self.assertEqual(published.header.stamp.to_sec(), 42.5)
        self.assertEqual(published.twist.covariance, [0.75] * 36)

    def test_preserves_exact_source_stamp_nanoseconds(self):
        """Converting through float seconds must not shift the source stamp by 1 ns."""
        self._set_fresh_steering()
        self.node._on_speed(self._speed_message(100.05))

        published = self.ros.publisher.messages[0]
        self.assertEqual(published.header.stamp.to_nsec(), 100050000000)

    def test_rejects_invalid_configured_twist_covariance(self):
        """A malformed YAML covariance would make published uncertainty invalid."""
        self.ros.params["~twist_covariance"] = [0.0] * 35

        with self.assertRaises(ValueError):
            self.module.VehicleOdometryNode()

    def test_rejects_invalid_control_pair_timeout(self):
        """An unbounded or non-finite pending speed could pair with unrelated steering."""
        for invalid in (0.0, -0.1, math.nan, 1.01):
            with self.subTest(invalid=invalid):
                self.ros.params["~control_pair_timeout_sec"] = invalid
                with self.assertRaises(ValueError):
                    self.module.VehicleOdometryNode()

    def test_rejects_non_finite_speed(self):
        """A NaN speed must not become an odometry update."""
        self._set_fresh_steering()

        self.node._on_speed(self._speed_message(10.0, speed=math.nan))

        self.assertEqual(self.ros.publisher.messages, [])

    def test_rejects_zero_source_stamp(self):
        """An unstamped speed message cannot establish a replay-safe pose time."""
        self._set_fresh_steering()

        self.node._on_speed(self._speed_message(0.0))

        self.assertEqual(self.ros.publisher.messages, [])

    def test_rejects_out_of_order_source_stamp(self):
        """A later receipt time must not hide a source-time reversal."""
        self._set_fresh_steering()
        self.node._on_speed(self._speed_message(20.0))
        self._set_fresh_steering()
        self.node._on_speed(self._speed_message(19.0))

        self.assertEqual(len(self.ros.publisher.messages), 1)

    def test_speed_arriving_before_steering_is_paired_once_within_timeout(self):
        """Independent TCPROS threads must not discard a speed that arrives first."""
        self._at(10.00)
        self.node._on_speed(self._speed_message(42.5))
        self.assertEqual(self.ros.publisher.messages, [])

        self._at(10.02)
        self.node._on_steering(types.SimpleNamespace(data=0.0))

        self.assertEqual(len(self.ros.publisher.messages), 1)
        self.assertEqual(self.ros.publisher.messages[0].header.stamp.to_sec(), 42.5)

    def test_each_steering_and_speed_sample_is_consumed_at_most_once(self):
        """Reusing steering would violate the locked fresh-pair contract."""
        self._at(20.00)
        self._set_fresh_steering()
        self._at(20.01)
        self.node._on_speed(self._speed_message(50.0))
        self._at(20.02)
        self.node._on_speed(self._speed_message(50.1))
        self.assertEqual(len(self.ros.publisher.messages), 1)

        self._at(20.03)
        self._set_fresh_steering()

        self.assertEqual(len(self.ros.publisher.messages), 2)
        self.assertEqual(self.ros.publisher.messages[1].header.stamp.to_sec(), 50.1)

    def test_new_speed_supersedes_an_unpaired_pending_speed(self):
        """A growing speed backlog could later integrate stale measurements."""
        self._at(30.00)
        self.node._on_speed(self._speed_message(60.0))
        self._at(30.01)
        self.node._on_speed(self._speed_message(60.1))
        self._at(30.02)
        self._set_fresh_steering()

        self.assertEqual(len(self.ros.publisher.messages), 1)
        self.assertEqual(self.ros.publisher.messages[0].header.stamp.to_sec(), 60.1)

    def test_new_steering_supersedes_an_unpaired_pending_steering(self):
        """The symmetric steering slot must retain only the newest sample."""
        self._at(35.00)
        self.node._on_steering(types.SimpleNamespace(data=0.1))
        self._at(35.01)
        self.node._on_steering(types.SimpleNamespace(data=0.2))
        self._at(35.02)
        self.node._on_speed(self._speed_message(65.0))

        self.assertEqual(len(self.ros.publisher.messages), 1)
        expected_yaw_rate = math.tan(0.2) / 0.32
        self.assertAlmostEqual(
            self.ros.publisher.messages[0].twist.twist.angular.z,
            expected_yaw_rate,
        )
        self.assertTrue(any(
            "replacing unpaired steering" in str(warning)
            for warning in self.ros.warnings
        ))

    def test_stale_pending_speed_is_not_paired_with_late_steering(self):
        """Late steering must not retroactively assign a control to an old speed."""
        self._at(40.00)
        self.node._on_speed(self._speed_message(70.0))
        self._at(40.11)
        self._set_fresh_steering()
        self.assertEqual(self.ros.publisher.messages, [])

        self._at(40.12)
        self.node._on_speed(self._speed_message(70.1))

        self.assertEqual(len(self.ros.publisher.messages), 1)
        self.assertEqual(self.ros.publisher.messages[0].header.stamp.to_sec(), 70.1)

    def test_stale_pending_steering_is_not_paired_with_late_speed(self):
        """A late speed must expire old steering and wait for a new sample."""
        self._at(45.00)
        self.node._on_steering(types.SimpleNamespace(data=0.1))
        self._at(45.11)
        self.node._on_speed(self._speed_message(75.0))
        self.assertEqual(self.ros.publisher.messages, [])

        self._at(45.12)
        self.node._on_steering(types.SimpleNamespace(data=0.2))

        self.assertEqual(len(self.ros.publisher.messages), 1)
        self.assertAlmostEqual(
            self.ros.publisher.messages[0].twist.twist.angular.z,
            math.tan(0.2) / 0.32,
        )
        self.assertTrue(any(
            warning[-1] == "steering"
            and "expired unpaired %s sample" in warning
            for warning in self.ros.warnings
        ))

    def test_receipt_clock_reversal_clears_old_pending_speed(self):
        """A rosbag loop must not pair pre-loop speed with post-loop steering."""
        self._at(80.00)
        self.node._on_speed(self._speed_message(90.0))
        self._at(5.00)
        self._set_fresh_steering()
        self.assertEqual(self.ros.publisher.messages, [])

        self._at(5.01)
        self.node._on_speed(self._speed_message(10.0))

        self.assertEqual(len(self.ros.publisher.messages), 1)
        self.assertEqual(self.ros.publisher.messages[0].header.stamp.to_sec(), 10.0)

    def test_receipt_clock_reversal_resets_integrator_for_new_source_epoch(self):
        """A rosbag loop must restart pose and accept the lower source stamp."""
        self._at(80.00)
        self._set_fresh_steering()
        self._at(80.01)
        self.node._on_speed(self._speed_message(90.0))
        self.assertEqual(len(self.ros.publisher.messages), 1)

        self._at(5.00)
        self._set_fresh_steering()
        self._at(5.01)
        self.node._on_speed(self._speed_message(10.0))

        self.assertEqual(len(self.ros.publisher.messages), 2)
        restarted = self.ros.publisher.messages[1]
        self.assertEqual(restarted.header.stamp.to_sec(), 10.0)
        self.assertEqual(
            (
                restarted.pose.pose.position.x,
                restarted.pose.pose.position.y,
                restarted.pose.pose.orientation.z,
                restarted.pose.pose.orientation.w,
            ),
            (0.0, 0.0, 0.0, 1.0),
        )

    def test_concurrent_callbacks_publish_one_pair_without_reuse(self):
        """Callback concurrency must not integrate one message more than once."""
        barrier = threading.Barrier(3)

        def send_speed():
            barrier.wait()
            self.node._on_speed(self._speed_message(100.0))

        def send_steering():
            barrier.wait()
            self.node._on_steering(types.SimpleNamespace(data=0.0))

        speed_thread = threading.Thread(target=send_speed)
        steering_thread = threading.Thread(target=send_steering)
        speed_thread.start()
        steering_thread.start()
        barrier.wait()
        speed_thread.join()
        steering_thread.join()

        self.assertEqual(len(self.ros.publisher.messages), 1)

    def test_receipt_time_is_captured_in_pair_lock_acquisition_order(self):
        """Inverse callback scheduling must not look like a ROS clock reversal."""
        reverse_lock = _ReverseAcquisitionLock()
        self.node._pair_lock = reverse_lock
        self.ros.now_values = iter((10.02, 10.03))

        older = threading.Thread(
            name="older_receipt",
            target=lambda: self.node._on_steering(types.SimpleNamespace(data=0.0)),
        )
        newer = threading.Thread(
            name="newer_receipt",
            target=lambda: self.node._on_speed(self._speed_message(100.0)),
        )
        older.start()
        self.assertTrue(reverse_lock.older_waiting.wait(timeout=2.0))
        newer.start()
        older.join(timeout=2.0)
        newer.join(timeout=2.0)

        self.assertFalse(older.is_alive())
        self.assertFalse(newer.is_alive())
        self.assertEqual(len(self.ros.publisher.messages), 1)
        self.assertFalse(any("clock reversed" in str(warning) for warning in self.ros.warnings))


if __name__ == "__main__":
    unittest.main()
