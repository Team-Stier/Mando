import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest

from stier_slam_core.freshness import (
    FUTURE,
    MISSING,
    OK,
    STALE,
    TIME_REVERSED,
    FreshnessMonitor,
    SensorConfig,
)


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "sensor_health_node.py"


class FreshnessMonitorTest(unittest.TestCase):
    def setUp(self):
        self.monitor = FreshnessMonitor(
            {
                "scan": SensorConfig(timeout_sec=1.0, has_header_stamp=True),
                "rtk_status": SensorConfig(timeout_sec=0.5, has_header_stamp=False),
            },
            future_tolerance_sec=0.2,
        )

    def test_reports_missing_before_the_first_sample(self):
        """Removing initial health visibility would hide an unplugged required sensor."""
        health = self.monitor.evaluate(5.0)["scan"]

        self.assertEqual(health.state, MISSING)
        self.assertEqual(health.reason, "missing")
        self.assertIsNone(health.source_age_sec)
        self.assertIsNone(health.receipt_age_sec)
        self.assertIsNone(health.rate_hz)

    def test_reports_stamped_ok_with_distinct_source_and_receipt_ages(self):
        """Using receipt time as a source stamp would conceal upstream timestamp delay."""
        self.monitor.update("scan", header_stamp=10.0, receipt_time=10.25)

        health = self.monitor.evaluate(10.5)["scan"]

        self.assertEqual(health.state, OK)
        self.assertEqual(health.reason, "ok")
        self.assertAlmostEqual(health.source_age_sec, 0.5)
        self.assertAlmostEqual(health.receipt_age_sec, 0.25)
        self.assertIsNone(health.rate_hz)
        self.assertEqual(health.last_header_stamp, 10.0)
        self.assertEqual(health.last_receipt_time, 10.25)

    def test_is_stale_at_the_exact_configured_source_timeout(self):
        """Changing >= to > would leave a sensor healthy for one timeout boundary."""
        self.monitor.update("scan", header_stamp=10.0, receipt_time=10.0)

        health = self.monitor.evaluate(11.0)["scan"]

        self.assertEqual(health.state, STALE)
        self.assertEqual(health.reason, "source_age_stale")
        self.assertEqual(health.source_age_sec, 1.0)

    def test_marks_receipt_transport_delay_stale_even_with_a_fresh_source_stamp(self):
        """Ignoring receipt age would let a delayed buffered message refresh health."""
        self.monitor.update("scan", header_stamp=10.0, receipt_time=9.0)

        health = self.monitor.evaluate(10.0)["scan"]

        self.assertEqual(health.state, STALE)
        self.assertEqual(health.reason, "receipt_age_stale")
        self.assertEqual(health.source_age_sec, 0.0)
        self.assertEqual(health.receipt_age_sec, 1.0)

    def test_future_requires_strictly_more_than_the_tolerance(self):
        """Changing the future comparison boundary would reject a permitted clock skew."""
        self.monitor.update("scan", header_stamp=10.2, receipt_time=10.0)
        at_boundary = self.monitor.evaluate(10.0)["scan"]

        self.monitor.update("scan", header_stamp=11.200001, receipt_time=11.0)
        beyond_boundary = self.monitor.evaluate(11.0)["scan"]

        self.assertEqual(at_boundary.state, OK)
        self.assertEqual(beyond_boundary.state, FUTURE)
        self.assertEqual(beyond_boundary.reason, "source_stamp_future")

    def test_unstamped_input_has_no_fabricated_source_stamp_and_uses_receipt_age(self):
        """Synthesizing a header for DiagnosticStatus would misrepresent the ROS contract."""
        self.monitor.update("rtk_status", header_stamp=None, receipt_time=20.0)

        healthy = self.monitor.evaluate(20.49)["rtk_status"]
        stale = self.monitor.evaluate(20.5)["rtk_status"]

        self.assertEqual(healthy.state, OK)
        self.assertIsNone(healthy.source_age_sec)
        self.assertIsNone(healthy.last_header_stamp)
        self.assertEqual(stale.state, STALE)
        self.assertEqual(stale.reason, "receipt_age_stale")

    def test_rejects_a_future_receipt_before_any_evaluation_clock_history_exists(self):
        """Treating a future receipt as OK would hide a /clock-domain mismatch on first tick."""
        self.monitor.update("rtk_status", header_stamp=None, receipt_time=10.0)

        health = self.monitor.evaluate(9.7)["rtk_status"]

        self.assertEqual(health.state, FUTURE)
        self.assertEqual(health.reason, "receipt_time_future")
        self.assertAlmostEqual(health.receipt_age_sec, -0.3)

    def test_header_reversal_requires_a_later_monotonic_source_sample_to_recover(self):
        """Accepting an out-of-order header would corrupt rate and bag replay state."""
        self.monitor.update("scan", header_stamp=10.0, receipt_time=10.0)
        self.monitor.update("scan", header_stamp=9.0, receipt_time=10.1)

        reversed_health = self.monitor.evaluate(10.1)["scan"]
        self.monitor.update("scan", header_stamp=11.0, receipt_time=11.0)
        recovered_health = self.monitor.evaluate(11.0)["scan"]

        self.assertEqual(reversed_health.state, TIME_REVERSED)
        self.assertEqual(reversed_health.reason, "header_time_reversed")
        self.assertEqual(recovered_health.state, OK)
        self.assertEqual(recovered_health.rate_hz, 1.0)

    def test_evaluation_clock_reversal_resets_samples_until_new_monotonic_data_arrives(self):
        """Keeping pre-seek samples after /clock reversal would falsely report bag health."""
        self.monitor.update("scan", header_stamp=10.0, receipt_time=10.0)
        self.assertEqual(self.monitor.evaluate(10.0)["scan"].state, OK)

        reversed_health = self.monitor.evaluate(9.0)["scan"]
        self.monitor.update("scan", header_stamp=9.5, receipt_time=9.5)
        recovered = self.monitor.evaluate(9.5)["scan"]

        self.assertEqual(reversed_health.state, TIME_REVERSED)
        self.assertEqual(reversed_health.reason, "evaluation_time_reversed")
        self.assertEqual(recovered.state, OK)

    def test_rejects_invalid_or_unknown_updates_without_partial_mutation(self):
        """Mutating before validation would let malformed callbacks refresh diagnostics."""
        self.monitor.update("scan", header_stamp=2.0, receipt_time=2.0)
        baseline = self.monitor.evaluate(2.0)["scan"]

        invalid_updates = (
            ("unknown", 3.0, 3.0),
            ("scan", math.nan, 3.0),
            ("scan", 3.0, math.inf),
            ("scan", -1.0, 3.0),
            ("scan", 3.0, -1.0),
            ("scan", None, 3.0),
            ("rtk_status", 3.0, 3.0),
        )
        for name, header_stamp, receipt_time in invalid_updates:
            with self.subTest(name=name, header_stamp=header_stamp, receipt_time=receipt_time):
                with self.assertRaises((KeyError, ValueError)):
                    self.monitor.update(name, header_stamp, receipt_time)
                health = self.monitor.evaluate(2.0)["scan"]
                self.assertEqual(health.last_header_stamp, baseline.last_header_stamp)
                self.assertEqual(health.last_receipt_time, baseline.last_receipt_time)
                self.assertEqual(health.state, baseline.state)

    def test_rejects_zero_stamped_source_without_mutating_existing_health(self):
        """Accepting source zero would make replay origin look like a valid sensor sample."""
        self.monitor.update("scan", header_stamp=2.0, receipt_time=2.0)

        with self.assertRaises(ValueError):
            self.monitor.update("scan", header_stamp=0.0, receipt_time=3.0)
        health = self.monitor.evaluate(2.0)["scan"]

        self.assertEqual(health.state, OK)
        self.assertEqual(health.last_header_stamp, 2.0)
        self.assertEqual(health.last_receipt_time, 2.0)

    def test_unstamped_receipt_zero_is_valid_at_ros_time_zero(self):
        """Rejecting receipt zero would break /use_sim_time startup for headerless contracts."""
        self.monitor.update("rtk_status", header_stamp=None, receipt_time=0.0)

        health = self.monitor.evaluate(0.0)["rtk_status"]

        self.assertEqual(health.state, OK)
        self.assertEqual(health.last_receipt_time, 0.0)

    def test_equal_unstamped_receipt_tick_remains_healthy_without_an_infinite_rate(self):
        """Treating equal callback ticks as reversal would reject valid same-clock messages."""
        self.monitor.update("rtk_status", header_stamp=None, receipt_time=5.0)
        self.monitor.update("rtk_status", header_stamp=None, receipt_time=5.0)

        health = self.monitor.evaluate(5.0)["rtk_status"]

        self.assertEqual(health.state, OK)
        self.assertEqual(health.last_receipt_time, 5.0)
        self.assertTrue(health.rate_hz is None or math.isfinite(health.rate_hz))

    def test_strictly_backward_unstamped_receipt_requires_later_recovery(self):
        """Allowing an earlier receipt to replace state would corrupt headerless rate history."""
        self.monitor.update("rtk_status", header_stamp=None, receipt_time=5.0)
        self.monitor.update("rtk_status", header_stamp=None, receipt_time=4.0)

        reversed_health = self.monitor.evaluate(5.0)["rtk_status"]
        self.monitor.update("rtk_status", header_stamp=None, receipt_time=5.0)
        equal_health = self.monitor.evaluate(5.0)["rtk_status"]
        self.monitor.update("rtk_status", header_stamp=None, receipt_time=6.0)
        recovered_health = self.monitor.evaluate(6.0)["rtk_status"]

        self.assertEqual(reversed_health.state, TIME_REVERSED)
        self.assertEqual(reversed_health.reason, "receipt_time_reversed")
        self.assertEqual(equal_health.state, TIME_REVERSED)
        self.assertEqual(recovered_health.state, OK)
        self.assertTrue(math.isfinite(recovered_health.rate_hz))

    def test_validates_configuration_and_rate_is_always_finite(self):
        """Invalid timeouts or a zero interval must not produce a NaN diagnostic rate."""
        invalid_configurations = (
            ({"scan": SensorConfig(0.0, True)}, 0.1),
            ({"scan": SensorConfig(1.0, True)}, -0.1),
            ({"scan": SensorConfig(math.nan, True)}, 0.1),
        )
        for sensors, tolerance in invalid_configurations:
            with self.subTest(sensors=sensors, tolerance=tolerance):
                with self.assertRaises(ValueError):
                    FreshnessMonitor(sensors, tolerance)

        self.monitor.update("scan", 10.0, 10.0)
        self.monitor.update("scan", 11.0, 11.0)
        rate = self.monitor.evaluate(11.0)["scan"].rate_hz
        self.assertTrue(math.isfinite(rate))
        self.assertGreater(rate, 0.0)


class _Stamp:
    def __init__(self, seconds):
        self._seconds = seconds

    def to_sec(self):
        return self._seconds


class _Header:
    def __init__(self, stamp):
        self.stamp = _Stamp(stamp)


class _LaserScan:
    def __init__(self, stamp):
        self.header = _Header(stamp)


class _Imu(_LaserScan):
    pass


class _NavSatFix(_LaserScan):
    pass


class _TwistWithCovarianceStamped(_LaserScan):
    pass


class _Image(_LaserScan):
    pass


class _Float64:
    pass


class _DiagnosticStatus:
    OK = 0
    WARN = 1
    ERROR = 2

    def __init__(self):
        self.level = None
        self.name = ""
        self.hardware_id = ""
        self.message = ""
        self.values = []


class _DiagnosticArray:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None)
        self.status = []


class _KeyValue:
    def __init__(self):
        self.key = ""
        self.value = ""


class _LockProbe:
    def __init__(self):
        self.owned = False

    def __enter__(self):
        if self.owned:
            raise AssertionError("unexpected non-reentrant lock probe")
        self.owned = True
        return self

    def __exit__(self, _exc_type, _exc_value, _traceback):
        self.owned = False


class _Publisher:
    def __init__(self, topic, message_type):
        self.topic = topic
        self.message_type = message_type
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Ros:
    def __init__(self):
        self.now = 100.0
        self.publishers = []
        self.subscriptions = []
        self.timers = []
        self.params = {
            "/stier_slam/topics/inputs": {
                "scan_raw": "/slam/input/scan_raw",
                "imu": "/slam/input/imu",
                "gnss_fix": "/slam/input/gnss/fix",
                "gnss_rtk_status": "/slam/input/gnss/rtk_status",
                "vehicle_speed": "/slam/input/vehicle_speed",
                "steering_angle": "/slam/input/steering_angle",
                "camera_front_image": "/slam/input/camera/front/image_raw",
            },
            "/stier_slam/topics/outputs/diagnostics": "/slam/diagnostics",
            "~sensors": {
                "scan_raw": {"timeout_sec": 1.0, "has_header_stamp": True},
                "imu": {"timeout_sec": 1.0, "has_header_stamp": True},
                "gnss_fix": {"timeout_sec": 1.0, "has_header_stamp": True},
                "gnss_rtk_status": {"timeout_sec": 1.0, "has_header_stamp": False},
                "vehicle_speed": {"timeout_sec": 1.0, "has_header_stamp": True},
                "steering_angle": {"timeout_sec": 1.0, "has_header_stamp": False},
                "camera_front_image": {"timeout_sec": 1.0, "has_header_stamp": True},
            },
            "~future_tolerance_sec": 0.2,
            "~publish_rate_hz": 5.0,
        }

    def get_param(self, name):
        return self.params[name]

    def Publisher(self, topic, message_type, **kwargs):
        publisher = _Publisher(topic, message_type)
        self.publishers.append(publisher)
        return publisher

    def Subscriber(self, topic, message_type, callback, **kwargs):
        self.subscriptions.append((topic, message_type, callback))

    def Timer(self, duration, callback, **kwargs):
        timer = types.SimpleNamespace(duration=duration, callback=callback, kwargs=kwargs)
        self.timers.append(timer)
        return timer

    def logwarn_throttle(self, *args):
        pass


class SensorHealthNodeTest(unittest.TestCase):
    def setUp(self):
        module_names = (
            "rospy", "sensor_msgs", "sensor_msgs.msg", "geometry_msgs", "geometry_msgs.msg",
            "std_msgs", "std_msgs.msg", "diagnostic_msgs", "diagnostic_msgs.msg",
            "sensor_health_node_under_test",
        )
        self._saved_modules = {name: sys.modules.get(name) for name in module_names}
        self.ros = _Ros()
        rospy = types.ModuleType("rospy")
        rospy.get_param = self.ros.get_param
        rospy.Publisher = self.ros.Publisher
        rospy.Subscriber = self.ros.Subscriber
        rospy.Timer = self.ros.Timer
        rospy.Duration = lambda seconds: seconds
        rospy.Time = types.SimpleNamespace(
            now=lambda: _Stamp(self.ros.now), from_sec=lambda seconds: _Stamp(seconds)
        )
        rospy.logwarn_throttle = self.ros.logwarn_throttle
        sensor_msgs = types.ModuleType("sensor_msgs")
        sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
        sensor_msgs_msg.LaserScan = _LaserScan
        sensor_msgs_msg.Imu = _Imu
        sensor_msgs_msg.NavSatFix = _NavSatFix
        sensor_msgs_msg.Image = _Image
        geometry_msgs = types.ModuleType("geometry_msgs")
        geometry_msgs_msg = types.ModuleType("geometry_msgs.msg")
        geometry_msgs_msg.TwistWithCovarianceStamped = _TwistWithCovarianceStamped
        std_msgs = types.ModuleType("std_msgs")
        std_msgs_msg = types.ModuleType("std_msgs.msg")
        std_msgs_msg.Float64 = _Float64
        diagnostic_msgs = types.ModuleType("diagnostic_msgs")
        diagnostic_msgs_msg = types.ModuleType("diagnostic_msgs.msg")
        diagnostic_msgs_msg.DiagnosticArray = _DiagnosticArray
        diagnostic_msgs_msg.DiagnosticStatus = _DiagnosticStatus
        diagnostic_msgs_msg.KeyValue = _KeyValue
        sys.modules.update({
            "rospy": rospy,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
            "geometry_msgs": geometry_msgs,
            "geometry_msgs.msg": geometry_msgs_msg,
            "std_msgs": std_msgs,
            "std_msgs.msg": std_msgs_msg,
            "diagnostic_msgs": diagnostic_msgs,
            "diagnostic_msgs.msg": diagnostic_msgs_msg,
        })
        spec = importlib.util.spec_from_file_location("sensor_health_node_under_test", SCRIPT_PATH)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.node = self.module.SensorHealthNode()

    def tearDown(self):
        for name, module in self._saved_modules.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

    def _diagnostics(self):
        publisher = next(item for item in self.ros.publishers if item.topic == "/slam/diagnostics")
        return publisher.messages[-1]

    def _require_time_now_inside_lock(self):
        probe = _LockProbe()
        self.node._lock = probe
        monitor = self.node._monitor
        original_update = monitor.update
        original_evaluate = monitor.evaluate

        def update(*args, **kwargs):
            self.assertTrue(probe.owned)
            return original_update(*args, **kwargs)

        def evaluate(*args, **kwargs):
            self.assertTrue(probe.owned)
            return original_evaluate(*args, **kwargs)

        monitor.update = update
        monitor.evaluate = evaluate

        def now():
            self.assertTrue(probe.owned)
            return _Stamp(self.ros.now)

        self.module.rospy.Time.now = now

    def test_uses_every_normalized_contract_topic_type_and_resettable_ros_timer(self):
        """Changing a subscription type or omitting a contract sensor breaks adapter compatibility."""
        subscriptions = {(topic, message_type) for topic, message_type, _ in self.ros.subscriptions}

        self.assertEqual(subscriptions, {
            ("/slam/input/scan_raw", _LaserScan),
            ("/slam/input/imu", _Imu),
            ("/slam/input/gnss/fix", _NavSatFix),
            ("/slam/input/gnss/rtk_status", _DiagnosticStatus),
            ("/slam/input/vehicle_speed", _TwistWithCovarianceStamped),
            ("/slam/input/steering_angle", _Float64),
            ("/slam/input/camera/front/image_raw", _Image),
        })
        self.assertEqual(len(self.ros.timers), 1)
        self.assertEqual(self.ros.timers[0].duration, 0.2)
        self.assertTrue(self.ros.timers[0].kwargs["reset"])

    def test_uses_central_topic_parameters_for_health_interfaces(self):
        custom = dict(self.ros.params["/stier_slam/topics/inputs"])
        custom["scan_raw"] = "/custom/scan"
        self.ros.params["/stier_slam/topics/inputs"] = custom
        self.ros.params["/stier_slam/topics/outputs/diagnostics"] = "/custom/health"

        self.module.SensorHealthNode()

        self.assertEqual(self.ros.subscriptions[-7][0], "/custom/scan")
        self.assertEqual(self.ros.publishers[-1].topic, "/custom/health")

    def test_publishes_each_sensor_and_deterministic_aggregate_without_replacing_source_stamp(self):
        """Replacing a header with receipt time would hide stale bag data in diagnostics."""
        self.node._on_stamped("scan_raw", _LaserScan(33.0))
        self.ros.now = 100.0
        self.node._on_timer(None)

        diagnostics = self._diagnostics()
        statuses = {status.name: status for status in diagnostics.status}
        scan = statuses["sensor_health/scan_raw"]
        aggregate = statuses["sensor_health/aggregate"]
        scan_values = {value.key: value.value for value in scan.values}

        self.assertEqual(len(diagnostics.status), 8)
        self.assertEqual(diagnostics.header.stamp.to_sec(), 100.0)
        self.assertEqual(scan.hardware_id, "stier_slam")
        self.assertEqual(scan.message, "source_age_stale")
        self.assertEqual(scan_values["last_header_stamp"], "33.0")
        self.assertEqual(scan_values["source_age_sec"], "67.0")
        self.assertEqual(aggregate.message, "MISSING")
        self.assertEqual(aggregate.level, _DiagnosticStatus.ERROR)

    def test_malformed_header_is_dropped_without_refreshing_or_crashing(self):
        """Letting NaN headers through would turn malformed input into a healthy sample."""
        self.node._on_stamped("imu", _Imu(math.nan))
        self.node._on_timer(None)

        statuses = {status.name: status for status in self._diagnostics().status}
        imu = statuses["sensor_health/imu"]
        values = {value.key: value.value for value in imu.values}

        self.assertEqual(imu.message, "missing")
        self.assertEqual(values["last_header_stamp"], "")

    def test_zero_stamped_header_is_dropped_without_refreshing(self):
        """Forwarding a zero source stamp would violate the replay-safe stamped contract."""
        self.node._on_stamped("imu", _Imu(0.0))
        self.node._on_timer(None)

        statuses = {status.name: status for status in self._diagnostics().status}
        values = {value.key: value.value for value in statuses["sensor_health/imu"].values}

        self.assertEqual(statuses["sensor_health/imu"].message, "missing")
        self.assertEqual(values["last_header_stamp"], "")

    def test_unstamped_contracts_are_recorded_at_receipt_time_only(self):
        """Giving Float64 a fake source timestamp would misstate its normalized message contract."""
        self.node._on_unstamped("steering_angle", _Float64())
        self.node._on_timer(None)

        statuses = {status.name: status for status in self._diagnostics().status}
        steering = statuses["sensor_health/steering_angle"]
        values = {value.key: value.value for value in steering.values}

        self.assertEqual(steering.message, "ok")
        self.assertEqual(values["source_age_sec"], "")
        self.assertEqual(values["last_header_stamp"], "")
        self.assertEqual(values["last_receipt_time"], "100.0")

    def test_missing_required_sensor_is_not_weaker_than_an_observed_stale_sensor(self):
        """Letting stale outrank missing would hide a disconnected required sensor in aggregate."""
        self.node._on_stamped("scan_raw", _LaserScan(1.0))
        self.node._on_timer(None)

        statuses = {status.name: status for status in self._diagnostics().status}
        missing = statuses["sensor_health/imu"]
        aggregate = statuses["sensor_health/aggregate"]

        self.assertEqual(missing.level, _DiagnosticStatus.ERROR)
        self.assertEqual(aggregate.message, "MISSING")
        self.assertEqual(aggregate.level, _DiagnosticStatus.ERROR)

    def test_stamped_receipt_time_is_captured_while_the_monitor_lock_is_owned(self):
        """Capturing receipt before locking permits an older callback time to commit late."""
        self._require_time_now_inside_lock()

        self.node._on_stamped("scan_raw", _LaserScan(100.0))

    def test_unstamped_receipt_time_is_captured_while_the_monitor_lock_is_owned(self):
        """Capturing headerless receipt before locking permits an older callback time to commit late."""
        self._require_time_now_inside_lock()

        self.node._on_unstamped("steering_angle", _Float64())

    def test_timer_time_is_captured_while_the_monitor_lock_is_owned(self):
        """Capturing timer time before locking lets a later callback be evaluated against old time."""
        self._require_time_now_inside_lock()

        self.node._on_timer(None)

    def test_timer_and_callback_interleave_linearizes_at_the_locked_ros_time(self):
        """A timer snapshot must not publish false future or reversal after a later callback commits."""
        self.node._monitor.evaluate(100.3)
        phase = {"first": True}

        def now():
            if phase["first"]:
                phase["first"] = False
                self.node._on_stamped("scan_raw", _LaserScan(100.3))
                return _Stamp(100.3 if self.node._lock._is_owned() else 100.0)
            return _Stamp(100.3)

        self.module.rospy.Time.now = now
        self.node._on_timer(None)

        statuses = {status.name: status for status in self._diagnostics().status}
        scan = statuses["sensor_health/scan_raw"]

        self.assertEqual(scan.message, "ok")
        self.assertNotEqual(scan.message, "evaluation_time_reversed")


if __name__ == "__main__":
    unittest.main()
