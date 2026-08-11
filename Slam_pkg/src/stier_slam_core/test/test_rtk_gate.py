import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest

from stier_slam_core.rtk_gate import RtkFix, RtkGate


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "rtk_gate_node.py"


def fix(stamp=10.0, latitude=37.0, longitude=127.0, altitude=42.0,
        covariance_xy_m2=0.25, nav_status=0):
    return RtkFix(stamp, latitude, longitude, altitude, covariance_xy_m2, nav_status)


class RtkGateTest(unittest.TestCase):
    def setUp(self):
        self.gate = RtkGate(
            max_covariance_xy_m2=1.0,
            max_speed_mps=5.0,
            innovation_margin_m=1.0,
        )

    def test_accepts_first_fresh_fix_with_valid_navsat_status(self):
        """Dropping a valid initial FIX would prevent graph initialization."""
        decision = self.gate.evaluate(fix(), "FIX")

        self.assertTrue(decision.accepted)
        self.assertEqual(decision.reason, "accepted")

    def test_rejects_float_and_no_fix_states(self):
        """Treating a non-FIX adapter state as precise RTK would corrupt global constraints."""
        for state in ("FLOAT", "NO_FIX"):
            with self.subTest(state=state):
                decision = self.gate.evaluate(fix(), state)

                self.assertFalse(decision.accepted)
                self.assertEqual(decision.reason, "rtk_not_fixed")

    def test_rejects_invalid_navsat_status(self):
        """A no-fix NavSat status must not enter even when the adapter says FIX."""
        decision = self.gate.evaluate(fix(nav_status=-1), "FIX")

        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "navsat_status_invalid")

    def test_rejects_non_positive_source_stamp_before_first_fix(self):
        """A zero or negative source stamp cannot establish replay-safe RTK history."""
        for stamp in (0.0, -0.01):
            with self.subTest(stamp=stamp):
                decision = self.gate.evaluate(fix(stamp=stamp), "FIX")

                self.assertFalse(decision.accepted)
                self.assertEqual(decision.reason, "source_stamp_invalid")

    def test_rejects_latitude_and_longitude_outside_geodetic_bounds(self):
        """Out-of-range coordinates cannot form an RTAB-Map global observation."""
        for latitude, longitude, reason in (
                (90.001, 127.0, "latitude_out_of_bounds"),
                (37.0, -180.001, "longitude_out_of_bounds")):
            with self.subTest(latitude=latitude, longitude=longitude):
                decision = self.gate.evaluate(fix(latitude=latitude, longitude=longitude), "FIX")

                self.assertFalse(decision.accepted)
                self.assertEqual(decision.reason, reason)

    def test_rejects_non_finite_and_excessive_covariance(self):
        """Unknown or overly broad GNSS uncertainty must not be advertised as RTK FIX."""
        for covariance_xy_m2, reason in (
                (math.nan, "covariance_not_finite"),
                (1.001, "covariance_too_large")):
            with self.subTest(covariance_xy_m2=covariance_xy_m2):
                decision = self.gate.evaluate(fix(covariance_xy_m2=covariance_xy_m2), "FIX")

                self.assertFalse(decision.accepted)
                self.assertEqual(decision.reason, reason)

    def test_rejects_non_monotonic_time_after_an_accepted_fix(self):
        """Using equal or reversed source time would make the innovation speed undefined."""
        self.assertTrue(self.gate.evaluate(fix(stamp=10.0), "FIX").accepted)

        decision = self.gate.evaluate(fix(stamp=10.0), "FIX")

        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "non_monotonic_time")

    def test_rejects_innovation_faster_than_speed_limit_plus_margin(self):
        """A GNSS jump beyond the physical innovation envelope must be withheld."""
        self.assertTrue(self.gate.evaluate(fix(stamp=10.0), "FIX").accepted)

        decision = self.gate.evaluate(
            fix(stamp=12.0, latitude=37.0, longitude=127.0002), "FIX"
        )

        self.assertFalse(decision.accepted)
        self.assertEqual(decision.reason, "innovation_too_large")

    def test_rejection_does_not_replace_the_last_accepted_fix(self):
        """Letting a rejected future jump advance history would hide a later time reversal."""
        self.assertTrue(self.gate.evaluate(fix(stamp=10.0), "FIX").accepted)
        self.assertFalse(self.gate.evaluate(
            fix(stamp=20.0, longitude=127.01), "FIX"
        ).accepted)

        decision = self.gate.evaluate(fix(stamp=11.0), "FIX")

        self.assertTrue(decision.accepted)
        self.assertEqual(decision.reason, "accepted")


class _Stamp:
    def __init__(self, seconds):
        self._seconds = seconds

    def to_sec(self):
        return self._seconds


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _DiagnosticStatus:
    OK = 0
    WARN = 1
    ERROR = 2

    def __init__(self):
        self.level = None
        self.name = ""
        self.message = ""
        self.values = []


class _KeyValue:
    def __init__(self):
        self.key = ""
        self.value = ""


class _NavSatFix:
    COVARIANCE_TYPE_UNKNOWN = 0
    COVARIANCE_TYPE_DIAGONAL_KNOWN = 2

    def __init__(self, stamp=10.0, status=0, latitude=37.0, longitude=127.0,
                 altitude=42.0, covariance=None,
                 covariance_type=COVARIANCE_TYPE_DIAGONAL_KNOWN):
        self.header = types.SimpleNamespace(stamp=_Stamp(stamp), frame_id="gnss_link")
        self.status = types.SimpleNamespace(status=status)
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude
        self.position_covariance = covariance or [0.25, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0]
        self.position_covariance_type = covariance_type


class _Ros:
    def __init__(self):
        self.now = 100.0
        self.publishers = []
        self.subscriptions = []
        self.params = {
            "/stier_slam/topics/inputs/gnss_fix": "/slam/input/gnss/fix",
            "/stier_slam/topics/inputs/gnss_rtk_status": "/slam/input/gnss/rtk_status",
            "/stier_slam/topics/processed/accepted_gnss": "/slam/gnss/fix_accepted",
            "/stier_slam/topics/outputs/rtk_diagnostics": "/slam/diagnostics/rtk_gate",
            "~max_covariance_xy_m2": 1.0,
            "~max_speed_mps": 5.0,
            "~innovation_margin_m": 1.0,
            "~rtk_state_max_age_sec": 0.5,
        }

    def get_param(self, name):
        return self.params[name]

    def Publisher(self, topic, message_type, **kwargs):
        publisher = _Publisher()
        publisher.topic = topic
        publisher.message_type = message_type
        self.publishers.append(publisher)
        return publisher

    def Subscriber(self, topic, message_type, callback, **kwargs):
        self.subscriptions.append((topic, message_type, callback))


class RtkGateNodeTest(unittest.TestCase):
    def setUp(self):
        module_names = (
            "rospy", "sensor_msgs", "sensor_msgs.msg", "diagnostic_msgs",
            "diagnostic_msgs.msg", "rtk_gate_node_under_test",
        )
        self._saved_modules = {name: sys.modules.get(name) for name in module_names}
        self.ros = _Ros()
        rospy = types.ModuleType("rospy")
        rospy.get_param = self.ros.get_param
        rospy.Publisher = self.ros.Publisher
        rospy.Subscriber = self.ros.Subscriber
        rospy.Time = types.SimpleNamespace(now=lambda: _Stamp(self.ros.now))
        sensor_msgs = types.ModuleType("sensor_msgs")
        sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
        sensor_msgs_msg.NavSatFix = _NavSatFix
        diagnostic_msgs = types.ModuleType("diagnostic_msgs")
        diagnostic_msgs_msg = types.ModuleType("diagnostic_msgs.msg")
        diagnostic_msgs_msg.DiagnosticStatus = _DiagnosticStatus
        diagnostic_msgs_msg.KeyValue = _KeyValue
        sys.modules.update({
            "rospy": rospy,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
            "diagnostic_msgs": diagnostic_msgs,
            "diagnostic_msgs.msg": diagnostic_msgs_msg,
        })
        spec = importlib.util.spec_from_file_location("rtk_gate_node_under_test", SCRIPT_PATH)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.node = self.module.RtkGateNode()

    def tearDown(self):
        for name, module in self._saved_modules.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

    def _status(self, state):
        return types.SimpleNamespace(message=state)

    def _publisher(self, topic):
        return next(publisher for publisher in self.ros.publishers if publisher.topic == topic)

    def _diagnostic_values(self):
        diagnostic = self._publisher("/slam/diagnostics/rtk_gate").messages[-1]
        return {value.key: value.value for value in diagnostic.values}

    def test_uses_central_topic_parameters_for_ros_interfaces(self):
        self.ros.params["/stier_slam/topics/inputs/gnss_fix"] = "/custom/fix"
        self.ros.params["/stier_slam/topics/inputs/gnss_rtk_status"] = "/custom/status"
        self.ros.params["/stier_slam/topics/processed/accepted_gnss"] = "/custom/accepted"
        self.ros.params["/stier_slam/topics/outputs/rtk_diagnostics"] = "/custom/diagnostics"

        self.module.RtkGateNode()

        self.assertEqual([item[0] for item in self.ros.subscriptions[-2:]], [
            "/custom/status", "/custom/fix",
        ])
        self.assertEqual([item.topic for item in self.ros.publishers[-2:]], [
            "/custom/accepted", "/custom/diagnostics",
        ])

    def test_accepts_only_a_fresh_fix_and_preserves_original_message_stamp(self):
        """Re-stamping or forwarding a stale adapter state would break replay safety."""
        self.node._on_rtk_status(self._status("FIX"))
        source = _NavSatFix(stamp=42.5)
        self.node._on_fix(source)

        accepted = self._publisher("/slam/gnss/fix_accepted").messages[0]
        diagnostic = self._publisher("/slam/diagnostics/rtk_gate").messages[-1]
        self.assertIsNot(accepted, source)
        self.assertEqual(accepted.header.stamp.to_sec(), 42.5)
        self.assertEqual(accepted.latitude, 37.0)
        self.assertEqual(accepted.position_covariance[4], 0.5)
        self.assertEqual(diagnostic.level, _DiagnosticStatus.OK)
        self.assertEqual(diagnostic.message, "accepted")

    def test_rejects_stale_rtk_state_with_error_diagnostic_and_counter(self):
        """A previously FIX adapter state must expire instead of authorizing later fixes."""
        self.node._on_rtk_status(self._status("FIX"))
        self.ros.now = 100.51
        self.node._on_fix(_NavSatFix())

        self.assertEqual(self._publisher("/slam/gnss/fix_accepted").messages, [])
        diagnostic = self._publisher("/slam/diagnostics/rtk_gate").messages[-1]
        self.assertEqual(diagnostic.level, _DiagnosticStatus.ERROR)
        self.assertEqual(diagnostic.message, "rtk_state_stale")
        self.assertEqual(self._diagnostic_values()["rejected_rtk_state_stale"], "1")

    def test_rejects_clock_reversal_as_stale_state_with_counter(self):
        """A negative ROS-time age must not retain a future cached FIX across a seek."""
        self.node._on_rtk_status(self._status("FIX"))
        self.ros.now = 99.99
        self.node._on_fix(_NavSatFix())

        self.assertEqual(self._publisher("/slam/gnss/fix_accepted").messages, [])
        diagnostic = self._publisher("/slam/diagnostics/rtk_gate").messages[-1]
        self.assertEqual(diagnostic.level, _DiagnosticStatus.ERROR)
        self.assertEqual(diagnostic.message, "rtk_state_stale")
        self.assertEqual(self._diagnostic_values()["rejected_rtk_state_stale"], "1")

    def test_reports_float_as_warn_and_no_fix_as_error(self):
        """Collapsing FLOAT and NO_FIX into one health level would hide RTK recovery state."""
        self.node._on_rtk_status(self._status("FLOAT"))
        self.node._on_fix(_NavSatFix())
        float_diagnostic = self._publisher("/slam/diagnostics/rtk_gate").messages[-1]

        self.node._on_rtk_status(self._status("NO_FIX"))
        self.node._on_fix(_NavSatFix(stamp=11.0))
        no_fix_diagnostic = self._publisher("/slam/diagnostics/rtk_gate").messages[-1]

        self.assertEqual(float_diagnostic.level, _DiagnosticStatus.WARN)
        self.assertEqual(float_diagnostic.message, "rtk_not_fixed")
        self.assertEqual(no_fix_diagnostic.level, _DiagnosticStatus.ERROR)
        self.assertEqual(no_fix_diagnostic.message, "rtk_not_fixed")
        values = self._diagnostic_values()
        self.assertEqual(values["rejected_rtk_not_fixed"], "2")

    def test_counts_policy_rejections_in_diagnostics(self):
        """Dropping covariance failures without a counter would make field diagnosis opaque."""
        self.node._on_rtk_status(self._status("FIX"))
        self.node._on_fix(_NavSatFix(covariance=[math.nan] + [0.0] * 8))

        diagnostic = self._publisher("/slam/diagnostics/rtk_gate").messages[-1]
        self.assertEqual(diagnostic.level, _DiagnosticStatus.WARN)
        self.assertEqual(diagnostic.message, "covariance_not_finite")
        values = self._diagnostic_values()
        self.assertEqual(values["rejected_covariance_not_finite"], "1")

    def test_rejects_invalid_source_stamp_before_publication(self):
        """A source stamp of zero must be counted instead of being forwarded as a first fix."""
        self.node._on_rtk_status(self._status("FIX"))
        self.node._on_fix(_NavSatFix(stamp=0.0))

        self.assertEqual(self._publisher("/slam/gnss/fix_accepted").messages, [])
        diagnostic = self._publisher("/slam/diagnostics/rtk_gate").messages[-1]
        self.assertEqual(diagnostic.message, "source_stamp_invalid")
        self.assertEqual(self._diagnostic_values()["rejected_source_stamp_invalid"], "1")

    def test_rejects_unknown_covariance_type_before_publication(self):
        """Zero values with unknown covariance type cannot be interpreted as precise RTK."""
        self.node._on_rtk_status(self._status("FIX"))
        self.node._on_fix(_NavSatFix(
            covariance=[0.0] * 9,
            covariance_type=_NavSatFix.COVARIANCE_TYPE_UNKNOWN,
        ))

        self.assertEqual(self._publisher("/slam/gnss/fix_accepted").messages, [])
        diagnostic = self._publisher("/slam/diagnostics/rtk_gate").messages[-1]
        self.assertEqual(diagnostic.message, "covariance_unknown")
        self.assertEqual(self._diagnostic_values()["rejected_covariance_unknown"], "1")

    def test_rejects_negative_xy_covariance_axis_before_publication(self):
        """Taking max(X,Y) must not conceal an impossible negative axis variance."""
        self.node._on_rtk_status(self._status("FIX"))
        self.node._on_fix(_NavSatFix(
            covariance=[-0.25, 0.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0.0, 1.0],
        ))

        self.assertEqual(self._publisher("/slam/gnss/fix_accepted").messages, [])
        diagnostic = self._publisher("/slam/diagnostics/rtk_gate").messages[-1]
        self.assertEqual(diagnostic.message, "covariance_negative")
        self.assertEqual(self._diagnostic_values()["rejected_covariance_negative"], "1")


if __name__ == "__main__":
    unittest.main()
