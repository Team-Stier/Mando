import math
import importlib.util
from pathlib import Path
import sys
import types
import unittest

from stier_slam_core.scan_leveling import (
    ExcessiveTiltError,
    QuaternionValidationError,
    ScanValidationError,
    level_scan,
    normalize_quaternion,
    validate_max_abs_tilt,
)


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "scan_leveler_node.py"


def quaternion_from_rpy(roll, pitch, yaw):
    """Return a ROS-order quaternion for a hand-specified RPY attitude."""
    roll_half = roll * 0.5
    pitch_half = pitch * 0.5
    yaw_half = yaw * 0.5
    return (
        math.sin(roll_half) * math.cos(pitch_half) * math.cos(yaw_half)
        - math.cos(roll_half) * math.sin(pitch_half) * math.sin(yaw_half),
        math.cos(roll_half) * math.sin(pitch_half) * math.cos(yaw_half)
        + math.sin(roll_half) * math.cos(pitch_half) * math.sin(yaw_half),
        math.cos(roll_half) * math.cos(pitch_half) * math.sin(yaw_half)
        - math.sin(roll_half) * math.sin(pitch_half) * math.cos(yaw_half),
        math.cos(roll_half) * math.cos(pitch_half) * math.cos(yaw_half)
        + math.sin(roll_half) * math.sin(pitch_half) * math.sin(yaw_half),
    )


class ScanLevelingTest(unittest.TestCase):
    def test_normalizes_scaled_quaternion(self):
        """Skipping normalization would turn an otherwise valid IMU attitude into a bad rotation."""
        normalized = normalize_quaternion(0.0, 0.0, 2.0, 2.0)

        self.assertAlmostEqual(normalized[0], 0.0, places=15)
        self.assertAlmostEqual(normalized[1], 0.0, places=15)
        self.assertAlmostEqual(normalized[2], math.sqrt(0.5), places=15)
        self.assertAlmostEqual(normalized[3], math.sqrt(0.5), places=15)

    def test_rejects_zero_or_non_finite_quaternion(self):
        """A zero or NaN attitude cannot level a scan safely."""
        with self.assertRaises(QuaternionValidationError):
            normalize_quaternion(0.0, 0.0, 0.0, 0.0)
        with self.assertRaises(QuaternionValidationError):
            normalize_quaternion(0.0, math.nan, 0.0, 1.0)

    def test_rejects_wrong_length_quaternion_with_typed_error(self):
        """Unpacking a truncated IMU quaternion must not leak an implementation TypeError."""
        with self.assertRaises(QuaternionValidationError):
            validate_max_abs_tilt((0.0, 0.0, 1.0), math.radians(10.0))

    def test_removes_imu_yaw_but_retains_pitch_leveling(self):
        """Keeping yaw would rotate a forward scan as the vehicle changes heading."""
        points = level_scan(
            [2.0],
            0.0,
            1.0,
            0.1,
            10.0,
            quaternion_from_rpy(0.0, math.radians(10.0), math.radians(90.0)),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            -1.0,
            1.0,
        )

        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], 2.0 * math.cos(math.radians(10.0)), places=6)
        self.assertAlmostEqual(points[0][1], 0.0, places=6)
        self.assertAlmostEqual(points[0][2], -2.0 * math.sin(math.radians(10.0)), places=6)

    def test_retains_roll_leveling_while_removing_yaw(self):
        """Dropping roll would leave this sideways range at the wrong height."""
        points = level_scan(
            [2.0],
            math.pi * 0.5,
            1.0,
            0.1,
            10.0,
            quaternion_from_rpy(math.radians(30.0), 0.0, math.radians(70.0)),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            -2.0,
            2.0,
        )

        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], 0.0, places=6)
        self.assertAlmostEqual(points[0][1], math.sqrt(3.0), places=6)
        self.assertAlmostEqual(points[0][2], 1.0, places=6)

    def test_combines_roll_and_pitch_in_rpy_order_without_yaw(self):
        """Reversing roll/pitch multiplication would miss this forward component."""
        points = level_scan(
            [2.0],
            math.pi * 0.5,
            1.0,
            0.1,
            10.0,
            quaternion_from_rpy(math.radians(30.0), math.radians(30.0), math.radians(60.0)),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            -2.0,
            2.0,
        )

        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], 0.5, places=6)
        self.assertAlmostEqual(points[0][1], math.sqrt(3.0), places=6)
        self.assertAlmostEqual(points[0][2], math.sqrt(3.0) * 0.5, places=6)

    def test_applies_lidar_to_base_mount_before_leveling(self):
        """Ignoring the mounting transform would publish a lidar-frame cloud as base_link."""
        points = level_scan(
            [1.0],
            0.0,
            1.0,
            0.1,
            10.0,
            (0.0, 0.0, 0.0, 1.0),
            (1.0, 2.0, 0.5, 0.0, 0.0, math.pi * 0.5),
            -1.0,
            1.0,
        )

        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], 1.0, places=6)
        self.assertAlmostEqual(points[0][1], 3.0, places=6)
        self.assertAlmostEqual(points[0][2], 0.5, places=6)

    def test_filters_corrected_points_outside_z_limits(self):
        """Filtering before attitude correction would retain this below-road point."""
        points = level_scan(
            [2.0],
            0.0,
            1.0,
            0.1,
            10.0,
            quaternion_from_rpy(0.0, math.radians(45.0), 0.0),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            -1.0,
            1.0,
        )

        self.assertEqual(points, [])

    def test_skips_isolated_non_finite_ranges(self):
        """Treating NaN or infinity as returns would create false obstacles."""
        points = level_scan(
            [math.nan, 2.0, math.inf],
            0.0,
            math.pi * 0.5,
            0.1,
            10.0,
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            -1.0,
            1.0,
        )

        self.assertEqual(len(points), 1)
        self.assertAlmostEqual(points[0][0], 0.0, places=6)
        self.assertAlmostEqual(points[0][1], 2.0, places=6)

    def test_rejects_all_non_finite_ranges(self):
        """An entirely invalid scan must be dropped rather than published as an empty obstacle set."""
        with self.assertRaises(ScanValidationError):
            level_scan(
                [math.nan, math.inf],
                0.0,
                1.0,
                0.1,
                10.0,
                (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                -1.0,
                1.0,
            )

    def test_rejects_finite_ranges_when_none_are_within_sensor_bounds(self):
        """Marking finite values valid before range filtering would publish an empty cloud."""
        for out_of_range in (0.09, 10.01):
            with self.subTest(out_of_range=out_of_range):
                with self.assertRaises(ScanValidationError):
                    level_scan(
                        [out_of_range],
                        0.0,
                        1.0,
                        0.1,
                        10.0,
                        (0.0, 0.0, 0.0, 1.0),
                        (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                        -1.0,
                        1.0,
                    )

    def test_rejects_mixed_nonfinite_and_out_of_range_returns(self):
        """No measurement is usable when every finite return is outside sensor bounds."""
        with self.assertRaises(ScanValidationError):
            level_scan(
                [math.nan, 0.09, 10.01, math.inf],
                0.0,
                1.0,
                0.1,
                10.0,
                (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
                -1.0,
                1.0,
            )

    def test_rejects_excessive_roll_or_pitch(self):
        """Publishing a steep scan would violate the configured 2D-SLAM tilt envelope."""
        with self.assertRaises(ExcessiveTiltError):
            validate_max_abs_tilt(
                quaternion_from_rpy(0.0, math.radians(11.0), 0.0),
                math.radians(10.0),
            )

    def test_rejects_invalid_scan_metadata_and_filter_limits(self):
        """Accepting zero angular steps or inverted bounds would fabricate geometry."""
        arguments = (
            [2.0],
            0.0,
            0.0,
            0.1,
            10.0,
            (0.0, 0.0, 0.0, 1.0),
            (0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
            -1.0,
            1.0,
        )
        with self.assertRaises(ScanValidationError):
            level_scan(*arguments)
        with self.assertRaises(ScanValidationError):
            level_scan(
                [2.0], 0.0, 1.0, 10.0, 0.1,
                (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), -1.0, 1.0,
            )
        with self.assertRaises(ScanValidationError):
            level_scan(
                [2.0], 0.0, 1.0, 0.1, 10.0,
                (0.0, 0.0, 0.0, 1.0),
                (0.0, 0.0, 0.0, 0.0, 0.0, 0.0), 1.0, -1.0,
            )


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _Ros:
    def __init__(self):
        self.errors = []
        self.publisher = _Publisher()
        self.params = {
            "/stier_slam/topics/inputs/scan_raw": "/slam/input/scan_raw",
            "/stier_slam/topics/inputs/imu": "/slam/input/imu",
            "/stier_slam/topics/processed/leveled_points": "/slam/scan/leveled_points",
            "~queue_size": 7,
            "~sync_slop_sec": 0.05,
            "~max_abs_tilt_rad": math.radians(10.0),
            "~mount_xyz_rpy": [0.0] * 6,
            "~min_z_m": -1.0,
            "~max_z_m": 1.0,
        }

    def get_param(self, name):
        return self.params[name]

    def Publisher(self, *args, **kwargs):
        return self.publisher

    def logerr_throttle(self, *args):
        self.errors.append(args)


class _Filters:
    def __init__(self):
        self.subscribers = []
        self.synchronizers = []

    def Subscriber(self, topic, message_type):
        subscriber = types.SimpleNamespace(topic=topic, message_type=message_type)
        self.subscribers.append(subscriber)
        return subscriber

    def ApproximateTimeSynchronizer(self, subscribers, queue_size, slop):
        synchronizer = types.SimpleNamespace(
            subscribers=subscribers, queue_size=queue_size, slop=slop, callback=None
        )
        synchronizer.registerCallback = lambda callback: setattr(synchronizer, "callback", callback)
        self.synchronizers.append(synchronizer)
        return synchronizer


class ScanLevelerNodeTest(unittest.TestCase):
    def setUp(self):
        module_names = (
            "rospy", "message_filters", "sensor_msgs", "sensor_msgs.msg",
            "sensor_msgs.point_cloud2", "std_msgs", "std_msgs.msg",
            "scan_leveler_node_under_test",
        )
        self._saved_modules = {name: sys.modules.get(name) for name in module_names}
        self.ros = _Ros()
        self.filters = _Filters()

        rospy = types.ModuleType("rospy")
        rospy.get_param = self.ros.get_param
        rospy.Publisher = self.ros.Publisher
        rospy.logerr_throttle = self.ros.logerr_throttle
        message_filters = types.ModuleType("message_filters")
        message_filters.Subscriber = self.filters.Subscriber
        message_filters.ApproximateTimeSynchronizer = self.filters.ApproximateTimeSynchronizer
        sensor_msgs = types.ModuleType("sensor_msgs")
        sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
        sensor_msgs_msg.Imu = type("Imu", (), {})
        sensor_msgs_msg.LaserScan = type("LaserScan", (), {})
        sensor_msgs_msg.PointCloud2 = type("PointCloud2", (), {})
        point_cloud2 = types.ModuleType("sensor_msgs.point_cloud2")
        point_cloud2.create_cloud_xyz32 = lambda header, points: types.SimpleNamespace(
            header=header, points=points
        )
        sensor_msgs.point_cloud2 = point_cloud2
        std_msgs = types.ModuleType("std_msgs")
        std_msgs_msg = types.ModuleType("std_msgs.msg")
        std_msgs_msg.Header = type("Header", (), {})
        sys.modules.update({
            "rospy": rospy,
            "message_filters": message_filters,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
            "sensor_msgs.point_cloud2": point_cloud2,
            "std_msgs": std_msgs,
            "std_msgs.msg": std_msgs_msg,
        })
        spec = importlib.util.spec_from_file_location("scan_leveler_node_under_test", SCRIPT_PATH)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.node = self.module.ScanLevelerNode()

    def tearDown(self):
        for name, module in self._saved_modules.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

    @staticmethod
    def _scan(stamp=42.5, frame_id="lidar_link"):
        return types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=stamp, frame_id=frame_id),
            ranges=[2.0], angle_min=0.0, angle_increment=1.0,
            range_min=0.1, range_max=10.0,
        )

    @staticmethod
    def _imu(quaternion=(0.0, 0.0, 0.0, 1.0), covariance=None):
        return types.SimpleNamespace(
            orientation=types.SimpleNamespace(
                x=quaternion[0], y=quaternion[1], z=quaternion[2], w=quaternion[3]
            ),
            orientation_covariance=[0.01] * 9 if covariance is None else covariance,
        )

    def test_uses_approximate_sync_for_normalized_scan_and_imu(self):
        """Independent subscribers would pair attitude with a different scan timestamp."""
        synchronizer = self.filters.synchronizers[0]

        self.assertEqual([item.topic for item in self.filters.subscribers], [
            "/slam/input/scan_raw", "/slam/input/imu",
        ])
        self.assertEqual(synchronizer.queue_size, 7)
        self.assertEqual(synchronizer.slop, 0.05)
        self.assertIs(synchronizer.callback.__self__, self.node)

    def test_uses_central_topic_parameters_for_synchronized_inputs(self):
        self.ros.params["/stier_slam/topics/inputs/scan_raw"] = "/custom/scan"
        self.ros.params["/stier_slam/topics/inputs/imu"] = "/custom/imu"

        self.module.ScanLevelerNode()

        self.assertEqual([item.topic for item in self.filters.subscribers[-2:]], [
            "/custom/scan", "/custom/imu",
        ])

    def test_publishes_base_link_cloud_with_scan_source_stamp(self):
        """Replacing source time or frame would break replay alignment and downstream TF contracts."""
        self.node._on_synced(self._scan(), self._imu())

        published = self.ros.publisher.messages[0]
        self.assertEqual(published.header.stamp, 42.5)
        self.assertEqual(published.header.frame_id, "base_link")
        self.assertEqual(published.points, [(2.0, 0.0, 0.0)])

    def test_drops_imu_with_explicitly_unavailable_orientation(self):
        """Using an IMU that marks orientation unavailable would invent a leveling attitude."""
        self.node._on_synced(self._scan(), self._imu(covariance=[-1.0] + [0.0] * 8))

        self.assertEqual(self.ros.publisher.messages, [])
        self.assertEqual(len(self.ros.errors), 1)

    def test_drops_scan_when_pitch_exceeds_configured_limit(self):
        """A scan beyond the tilt envelope must not enter planar SLAM."""
        self.node._on_synced(
            self._scan(), self._imu(quaternion_from_rpy(0.0, math.radians(11.0), 0.0))
        )

        self.assertEqual(self.ros.publisher.messages, [])
        self.assertEqual(len(self.ros.errors), 1)

    def test_drops_scan_with_invalid_source_metadata(self):
        """An unstamped or frame-less scan cannot be replayed or transformed safely."""
        self.node._on_synced(self._scan(stamp=math.nan), self._imu())
        self.node._on_synced(self._scan(frame_id=""), self._imu())

        self.assertEqual(self.ros.publisher.messages, [])
        self.assertEqual(len(self.ros.errors), 2)


if __name__ == "__main__":
    unittest.main()
