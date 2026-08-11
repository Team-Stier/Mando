import math
import importlib.util
from pathlib import Path
import struct
import sys
import threading
import types
import unittest

from stier_slam_core.grid_map import GridMap
from stier_slam_core.live_obstacles import LiveObstacleTracker


SCRIPT_PATH = Path(__file__).parents[1] / "scripts" / "live_obstacle_node.py"


def _capturing_thread(target):
    """Return a started-later thread whose uncaught exception stays assertable."""
    errors = []

    def run():
        try:
            target()
        except BaseException as error:
            errors.append(error)

    return threading.Thread(target=run), errors


class LiveObstacleTrackerTest(unittest.TestCase):
    def setUp(self):
        self.grid = GridMap(
            3,
            2,
            1.0,
            0.0,
            0.0,
            [0, 100, -1,
             0, 0, 0],
        )

    def test_tracks_only_free_cells_and_coalesces_duplicate_cells(self):
        """Removing static suppression would display walls and unmapped space as live obstacles."""
        tracker = LiveObstacleTracker(self.grid, ttl_sec=2.0, occupied_threshold=50)

        cells = tracker.update(
            10.0,
            [(0.1, 0.1), (0.9, 0.9), (1.1, 0.1), (2.1, 0.1)],
        )

        self.assertEqual(cells, {(0, 0)})
        self.assertEqual(tracker.snapshot(10.0), {(0, 0)})

    def test_expires_a_cell_at_its_exact_ttl_boundary(self):
        """Using a strict expiry comparison would retain an obstacle one timestamp too long."""
        tracker = LiveObstacleTracker(self.grid, ttl_sec=2.0, occupied_threshold=50)
        tracker.update(10.0, [(0.1, 0.1)])

        self.assertEqual(tracker.snapshot(11.999), {(0, 0)})
        self.assertEqual(tracker.snapshot(12.0), set())

    def test_ignores_off_map_and_invalid_points(self):
        """Passing invalid geometry to GridMap would make one malformed cloud abort valid detections."""
        tracker = LiveObstacleTracker(self.grid, ttl_sec=2.0, occupied_threshold=50)

        cells = tracker.update(
            10.0,
            [(-0.1, 0.1), (3.0, 0.1), (math.nan, 0.1), (0.1, math.inf), (0.1, 1.1)],
        )

        self.assertEqual(cells, {(0, 1)})

    def test_time_reversal_clears_cache_and_records_reason(self):
        """Keeping future bag-loop cells would show obstacles that never existed in the replayed time."""
        tracker = LiveObstacleTracker(self.grid, ttl_sec=2.0, occupied_threshold=50)
        tracker.update(10.0, [(0.1, 0.1)])

        self.assertEqual(tracker.snapshot(9.0), set())
        self.assertEqual(tracker.last_reset_reason, "time_reversal")

    def test_forward_snapshot_does_not_reject_a_delayed_measurement_stamp(self):
        """Sharing timer and cloud watermarks would clear valid delayed sensor data as a time reversal."""
        tracker = LiveObstacleTracker(self.grid, ttl_sec=20.0, occupied_threshold=50)
        tracker.update(10.0, [(0.1, 0.1)])

        self.assertEqual(tracker.snapshot(20.0), {(0, 0)})
        self.assertEqual(tracker.update(11.0, [(0.1, 1.1)]), {(0, 0), (0, 1)})
        self.assertIsNone(tracker.last_reset_reason)

    def test_source_stamp_reversal_clears_prior_observations(self):
        """A cloud source-time rewind must discard cells left from the prior bag epoch."""
        tracker = LiveObstacleTracker(self.grid, ttl_sec=20.0, occupied_threshold=50)
        tracker.update(10.0, [(0.1, 0.1)])

        self.assertEqual(tracker.update(9.0, []), set())
        self.assertEqual(tracker.last_reset_reason, "time_reversal")

    def test_rejects_oversized_input_and_cache_without_partial_state_change(self):
        """Accepting unbounded clouds or cache growth would make one sensor fault exhaust memory."""
        tracker = LiveObstacleTracker(
            self.grid,
            ttl_sec=2.0,
            occupied_threshold=50,
            max_input_points=1,
            max_tracked_cells=1,
        )

        with self.assertRaises(ValueError):
            tracker.update(10.0, [(0.1, 0.1), (0.1, 1.1)])
        self.assertEqual(tracker.snapshot(10.0), set())

        tracker.update(11.0, [(0.1, 0.1)])
        with self.assertRaises(ValueError):
            tracker.update(11.5, [(0.1, 1.1)])
        self.assertEqual(tracker.snapshot(11.5), {(0, 0)})

    def test_delayed_observation_at_snapshot_expiry_boundary_is_not_reinserted(self):
        """Ignoring a forward timer watermark would republish a cell already expired in ROS time."""
        tracker = LiveObstacleTracker(self.grid, ttl_sec=2.0, occupied_threshold=50)
        tracker.update(10.0, [(0.1, 0.1)])
        self.assertEqual(tracker.snapshot(20.0), set())

        self.assertEqual(tracker.update(18.0, [(0.1, 1.1)]), set())
        self.assertEqual(tracker.update(18.001, [(0.1, 1.1)]), {(0, 1)})

    def test_reversal_overflow_keeps_prior_cache_and_watermarks_unchanged(self):
        """Clearing before capacity validation would lose valid cells when a rewound cloud is oversized."""
        tracker = LiveObstacleTracker(
            self.grid, ttl_sec=20.0, occupied_threshold=50, max_tracked_cells=1
        )
        tracker.update(10.0, [(0.1, 0.1)])

        with self.assertRaises(ValueError):
            tracker.update(9.0, [(0.1, 1.1), (1.1, 1.1)])

        self.assertEqual(tracker.snapshot(10.0), {(0, 0)})
        self.assertIsNone(tracker.last_reset_reason)


class _Object:
    pass


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


class _Ros:
    def __init__(self):
        self.params = {
            "/stier_slam/topics/processed/leveled_points": "/slam/scan/leveled_points",
            "/stier_slam/topics/outputs/released_map": "/slam/map/released",
            "/stier_slam/topics/outputs/live_obstacle_points": "/slam/live_obstacles/points",
            "/stier_slam/topics/outputs/live_obstacle_grid": "/slam/live_obstacles/grid",
            "~map_frame": "map",
            "~ttl_sec": 2.0,
            "~occupied_threshold": 50,
            "~local_grid_width": 2,
            "~local_grid_height": 2,
            "~local_grid_resolution": 1.0,
            "~max_input_points": 10,
            "~max_tracked_cells": 10,
            "~publish_rate_hz": 10.0,
        }
        self.publishers = []
        self.subscriptions = []
        self.timers = []
        self.warnings = []

    def get_param(self, name):
        return self.params[name]

    def Publisher(self, topic, message_type, **kwargs):
        publisher = _Publisher()
        self.publishers.append((topic, message_type, kwargs, publisher))
        return publisher

    def Subscriber(self, topic, message_type, callback, **kwargs):
        self.subscriptions.append((topic, message_type, callback, kwargs))

    def Duration(self, seconds):
        return seconds

    def Timer(self, duration, callback, reset=False):
        self.timers.append((duration, callback, reset))

    def logwarn_throttle(self, *args):
        self.warnings.append(args)


class _TfBuffer:
    def __init__(self):
        self.calls = []
        self.transform = None
        self.before_lookup = None

    def lookup_transform(self, *args):
        self.calls.append(args)
        if self.before_lookup is not None:
            self.before_lookup()
        if self.transform is None:
            raise _LookupException("no transform")
        return self.transform


class _PointCloud2:
    pass


class _OccupancyGrid:
    def __init__(self):
        self.header = _Object()
        self.info = _Object()
        self.info.origin = _Object()
        self.info.origin.position = _Object()
        self.info.origin.orientation = _Object()
        self.data = []


class LiveObstacleNodeTest(unittest.TestCase):
    def setUp(self):
        module_names = (
            "rospy", "tf2_ros", "nav_msgs", "nav_msgs.msg", "sensor_msgs",
            "sensor_msgs.msg", "sensor_msgs.point_cloud2", "std_msgs", "std_msgs.msg",
            "live_obstacle_node_under_test",
        )
        self._saved_modules = {name: sys.modules.get(name) for name in module_names}
        self.ros = _Ros()
        self.tf_buffer = _TfBuffer()

        rospy = types.ModuleType("rospy")
        rospy.get_param = self.ros.get_param
        rospy.Publisher = self.ros.Publisher
        rospy.Subscriber = self.ros.Subscriber
        rospy.logwarn_throttle = self.ros.logwarn_throttle
        rospy.Duration = self.ros.Duration
        rospy.Timer = self.ros.Timer
        tf2_ros = types.ModuleType("tf2_ros")
        tf2_ros.Buffer = lambda: self.tf_buffer
        tf2_ros.TransformListener = lambda buffer: types.SimpleNamespace(buffer=buffer)
        tf2_ros.LookupException = _LookupException
        tf2_ros.ConnectivityException = _LookupException
        tf2_ros.ExtrapolationException = _LookupException
        nav_msgs = types.ModuleType("nav_msgs")
        nav_msgs_msg = types.ModuleType("nav_msgs.msg")
        nav_msgs_msg.OccupancyGrid = _OccupancyGrid
        sensor_msgs = types.ModuleType("sensor_msgs")
        sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
        sensor_msgs_msg.PointCloud2 = _PointCloud2
        point_cloud2 = types.ModuleType("sensor_msgs.point_cloud2")
        point_cloud2.read_points = lambda message, **kwargs: iter(message.points)
        point_cloud2.create_cloud_xyz32 = lambda header, points: types.SimpleNamespace(
            header=header, points=list(points)
        )
        sensor_msgs.point_cloud2 = point_cloud2
        std_msgs = types.ModuleType("std_msgs")
        std_msgs_msg = types.ModuleType("std_msgs.msg")
        std_msgs_msg.Header = type("Header", (), {})
        sys.modules.update({
            "rospy": rospy,
            "tf2_ros": tf2_ros,
            "nav_msgs": nav_msgs,
            "nav_msgs.msg": nav_msgs_msg,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
            "sensor_msgs.point_cloud2": point_cloud2,
            "std_msgs": std_msgs,
            "std_msgs.msg": std_msgs_msg,
        })
        spec = importlib.util.spec_from_file_location("live_obstacle_node_under_test", SCRIPT_PATH)
        self.module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(self.module)
        self.node = self.module.LiveObstacleNode()

    def tearDown(self):
        for name, module in self._saved_modules.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

    @staticmethod
    def _cloud(stamp, points, fields=None):
        return types.SimpleNamespace(
            header=types.SimpleNamespace(stamp=_Stamp(stamp), frame_id="lidar_link"),
            points=points,
            fields=fields if fields is not None else [
                types.SimpleNamespace(name="x", offset=0, datatype=7, count=1),
                types.SimpleNamespace(name="y", offset=4, datatype=7, count=1),
                types.SimpleNamespace(name="z", offset=8, datatype=7, count=1),
            ],
        )

    @staticmethod
    def _released_map():
        message = _OccupancyGrid()
        message.header.stamp = _Stamp(1.0)
        message.header.frame_id = "map"
        message.info.width = 3
        message.info.height = 2
        message.info.resolution = 1.0
        message.info.origin.position.x = 0.0
        message.info.origin.position.y = 0.0
        message.info.origin.position.z = 0.0
        message.info.origin.orientation.x = 0.0
        message.info.origin.orientation.y = 0.0
        message.info.origin.orientation.z = 0.0
        message.info.origin.orientation.w = 1.0
        message.data = [0, 100, -1, 0, 0, 0]
        return message

    @staticmethod
    def _translation_transform(x, y, z):
        return types.SimpleNamespace(
            transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(x=x, y=y, z=z),
                rotation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )

    def test_uses_central_topic_parameters_for_live_layer_interfaces(self):
        self.ros.params["/stier_slam/topics/processed/leveled_points"] = "/custom/cloud"
        self.ros.params["/stier_slam/topics/outputs/released_map"] = "/custom/map"
        self.ros.params["/stier_slam/topics/outputs/live_obstacle_points"] = "/custom/points"
        self.ros.params["/stier_slam/topics/outputs/live_obstacle_grid"] = "/custom/grid"

        self.module.LiveObstacleNode()

        self.assertEqual([entry[0] for entry in self.ros.subscriptions[-2:]], [
            "/custom/map", "/custom/cloud",
        ])
        self.assertEqual([entry[0] for entry in self.ros.publishers[-2:]], [
            "/custom/points", "/custom/grid",
        ])

    def test_waits_for_released_map_and_tf_then_publishes_bounded_free_layer(self):
        """Publishing before map/TF or emitting the base extent would mislead obstacle visualization."""
        cloud = self._cloud(5.0, [(-0.9, 0.1, 0.0), (math.nan, 0.1, 0.0)])

        self.node._on_cloud(cloud)
        self.assertEqual([entry[3].messages for entry in self.ros.publishers], [[], []])

        released_map = self._released_map()
        source_cells = list(released_map.data)
        self.node._on_map(released_map)
        self.node._on_cloud(cloud)
        self.assertEqual([entry[3].messages for entry in self.ros.publishers], [[], []])

        self.tf_buffer.transform = self._translation_transform(1.0, 0.0, 0.0)
        self.node._on_cloud(cloud)

        self.assertEqual(self.tf_buffer.calls[-1], ("map", "lidar_link", cloud.header.stamp))
        points_publisher, grid_publisher = [entry[3] for entry in self.ros.publishers]
        self.assertEqual(points_publisher.messages[-1].points, [(0.5, 0.5, 0.0)])
        grid = grid_publisher.messages[-1]
        self.assertEqual((grid.info.width, grid.info.height, grid.info.resolution), (2, 2, 1.0))
        self.assertEqual(len(grid.data), 4)
        self.assertEqual(grid.data.count(100), 1)
        self.assertEqual(released_map.data, source_cells)

    def test_timer_publishes_empty_layer_at_exact_expiry_without_a_new_cloud(self):
        """Only expiring on a later cloud would leave a disappeared obstacle visible indefinitely."""
        self.node._on_map(self._released_map())
        self.tf_buffer.transform = self._translation_transform(1.0, 0.0, 0.0)
        self.node._on_cloud(self._cloud(5.0, [(-0.9, 0.1, 0.0)]))

        _, callback, _ = self.ros.timers[0]
        callback(types.SimpleNamespace(current_real=_Stamp(7.0)))

        points_publisher, grid_publisher = [entry[3] for entry in self.ros.publishers]
        self.assertEqual(points_publisher.messages[-1].points, [])
        self.assertNotIn(100, grid_publisher.messages[-1].data)

    def test_expiry_timer_resets_when_ros_time_moves_backwards(self):
        """A non-reset rospy timer can die on rosbag /clock rewind before it clears stale cells."""
        self.assertTrue(self.ros.timers[0][2])

    def test_rejects_local_grid_configuration_above_memory_bound(self):
        """Allowing an arbitrary width times height would allocate an attacker-controlled grid."""
        self.ros.params["~local_grid_width"] = 10001
        self.ros.params["~local_grid_height"] = 10000

        with self.assertRaises(ValueError):
            self.module.LiveObstacleNode()

    def test_delayed_map_replacement_drops_old_cloud_before_publication(self):
        """A map callback interleaving a cloud update must not publish stale geometry or mix map origins."""
        old_map = self._released_map()
        new_map = self._released_map()
        new_map.info.origin.position.x = 10.0
        new_map.data = [0] * 6
        self.node._on_map(old_map)
        self.tf_buffer.transform = self._translation_transform(1.0, 0.0, 0.0)
        self.tf_buffer.before_lookup = lambda: self.node._on_map(new_map)

        self.node._on_cloud(self._cloud(5.0, [(-0.9, 0.1, 0.0)]))

        points_publisher, grid_publisher = [entry[3] for entry in self.ros.publishers]
        self.assertEqual(points_publisher.messages, [])
        self.assertEqual(grid_publisher.messages, [])

        self.tf_buffer.before_lookup = None
        self.tf_buffer.transform = self._translation_transform(10.0, 0.0, 0.0)
        self.node._on_cloud(self._cloud(6.0, [(0.1, 0.1, 0.0)]))

        self.assertEqual(points_publisher.messages[-1].points, [(10.5, 0.5, 0.0)])
        self.assertEqual(grid_publisher.messages[-1].info.origin.position.x, 9.0)

    def test_local_grid_stays_ego_centered_and_reports_far_cell_omission(self):
        """Centering on obstacle mean would move the local window and conceal deliberate far-cell clipping."""
        released_map = self._released_map()
        released_map.info.width = 5
        released_map.data = [0] * 10
        self.node._on_map(released_map)
        self.tf_buffer.transform = self._translation_transform(0.5, 0.5, 0.0)
        self.node._on_cloud(self._cloud(5.0, [(0.0, 0.0, 0.0), (2.0, 0.0, 0.0)]))

        points_publisher, grid_publisher = [entry[3] for entry in self.ros.publishers]
        self.assertEqual(
            points_publisher.messages[-1].points,
            [(0.5, 0.5, 0.0), (2.5, 0.5, 0.0)],
        )
        grid = grid_publisher.messages[-1]
        self.assertEqual((grid.info.origin.position.x, grid.info.origin.position.y), (-0.5, -0.5))
        self.assertEqual(grid.data.count(100), 1)
        self.assertTrue(any("omitted" in warning[1] for warning in self.ros.warnings))

        _, callback, _ = self.ros.timers[0]
        callback(types.SimpleNamespace(current_real=_Stamp(6.0)))
        live_grid = grid_publisher.messages[-1]
        self.assertEqual(
            (live_grid.info.origin.position.x, live_grid.info.origin.position.y), (-0.5, -0.5)
        )
        callback(types.SimpleNamespace(current_real=_Stamp(7.0)))
        empty_grid = grid_publisher.messages[-1]
        self.assertEqual(
            (empty_grid.info.origin.position.x, empty_grid.info.origin.position.y), (-0.5, -0.5)
        )
        self.assertEqual(points_publisher.messages[-1].points, [])

    def test_malformed_cloud_iteration_drops_without_publishing_or_changing_tracker(self):
        """Leaking a PointCloud2 unpack error would crash the callback after partially reading a cloud."""
        self.node._on_map(self._released_map())
        self.tf_buffer.transform = self._translation_transform(1.0, 0.0, 0.0)
        self.node._on_cloud(self._cloud(5.0, [(-0.9, 0.1, 0.0)]))
        points_publisher, grid_publisher = [entry[3] for entry in self.ros.publishers]
        point_count = len(points_publisher.messages)
        grid_count = len(grid_publisher.messages)

        def malformed_points():
            yield (-0.9, 0.1, 0.0)
            raise struct.error("truncated PointCloud2 record")

        self.module.point_cloud2.read_points = lambda message, **kwargs: malformed_points()
        self.node._on_cloud(self._cloud(6.0, []))

        self.assertEqual(len(points_publisher.messages), point_count)
        self.assertEqual(len(grid_publisher.messages), grid_count)
        self.assertEqual(self.node._tracker.snapshot(5.0), {(0, 0)})

    def test_ros_boundary_normalizes_intermediate_occupancy_and_rejects_out_of_range(self):
        """Passing non-trinary ROS occupancy values to GridMap would reject valid released-map thresholds."""
        released_map = self._released_map()
        released_map.data = [-1, 25, 50, 100, 0, 49]

        self.node._on_map(released_map)

        self.assertEqual(self.node._base_grid.cells, (-1, 0, 100, 100, 0, 0))
        released_map.data[0] = 101
        self.node._on_map(released_map)
        self.assertIsNone(self.node._tracker)

    def test_transform_rejects_bad_quaternion_norm_and_applies_3d_rotation(self):
        """Normalizing overflow or near-zero quaternions would fabricate a map-frame obstacle position."""
        transform = types.SimpleNamespace(
            transform=types.SimpleNamespace(
                translation=types.SimpleNamespace(x=1.0, y=2.0, z=3.0),
                rotation=types.SimpleNamespace(
                    x=0.0, y=math.sqrt(0.5), z=0.0, w=math.sqrt(0.5)
                ),
            )
        )
        rotated = self.module._transform_xyz((1.0, 0.0, 0.0), transform)
        self.assertAlmostEqual(rotated[0], 1.0, places=12)
        self.assertAlmostEqual(rotated[1], 2.0, places=12)
        self.assertAlmostEqual(rotated[2], 2.0, places=12)

        for quaternion in ((0.0, 0.0, 0.0, 0.0), (math.nan, 0.0, 0.0, 1.0),
                           (1.0e308, 0.0, 0.0, 1.0), (1.0e-200, 0.0, 0.0, 0.0)):
            with self.subTest(quaternion=quaternion):
                transform.transform.rotation.x = quaternion[0]
                transform.transform.rotation.y = quaternion[1]
                transform.transform.rotation.z = quaternion[2]
                transform.transform.rotation.w = quaternion[3]
                with self.assertRaises(ValueError):
                    self.module._transform_xyz((1.0, 0.0, 0.0), transform)

    def test_timer_expires_blocked_cloud_before_it_can_publish_stale_cells(self):
        """Holding the state lock during point iteration would let a stale cloud publish before /clock expiry."""
        self.node._on_map(self._released_map())
        self.tf_buffer.transform = self._translation_transform(1.0, 0.0, 0.0)
        self.node._on_cloud(self._cloud(4.0, [(-0.9, 0.1, 0.0)]))
        points_publisher, _ = [entry[3] for entry in self.ros.publishers]
        iteration_started = threading.Event()
        resume_iteration = threading.Event()
        timer_done = threading.Event()

        def blocked_points():
            iteration_started.set()
            resume_iteration.wait(2.0)
            yield (-0.9, 0.1, 0.0)

        self.module.point_cloud2.read_points = lambda message, **kwargs: blocked_points()
        cloud_thread, cloud_errors = _capturing_thread(
            lambda: self.node._on_cloud(self._cloud(5.0, [(-0.9, 0.1, 0.0)]))
        )
        _, timer_callback, _ = self.ros.timers[0]

        def run_timer():
            timer_callback(types.SimpleNamespace(current_real=_Stamp(7.0)))
            timer_done.set()

        timer_thread, timer_errors = _capturing_thread(run_timer)
        cloud_thread.start()
        try:
            self.assertTrue(iteration_started.wait(1.0))
            timer_thread.start()
            self.assertTrue(timer_done.wait(0.5))
            self.assertEqual(
                [message.header.stamp.to_sec() for message in points_publisher.messages], [4.0, 7.0]
            )
        finally:
            resume_iteration.set()
            cloud_thread.join(2.0)
            timer_thread.join(2.0)

        self.assertFalse(cloud_thread.is_alive())
        self.assertFalse(timer_thread.is_alive())
        self.assertEqual(cloud_errors, [])
        self.assertEqual(timer_errors, [])
        self.assertEqual(
            [message.header.stamp.to_sec() for message in points_publisher.messages], [4.0, 7.0]
        )
        self.assertEqual(points_publisher.messages[-1].points, [])

    def test_map_thread_invalidates_blocked_cloud_before_it_publishes(self):
        """Holding the state lock during cloud preparation would make a released-map replacement wait and leak old data."""
        old_map = self._released_map()
        new_map = self._released_map()
        new_map.info.origin.position.x = 10.0
        new_map.data = [0] * 6
        self.node._on_map(old_map)
        self.tf_buffer.transform = self._translation_transform(1.0, 0.0, 0.0)
        points_publisher, grid_publisher = [entry[3] for entry in self.ros.publishers]
        iteration_started = threading.Event()
        resume_iteration = threading.Event()
        map_started = threading.Event()
        map_done = threading.Event()

        def blocked_points():
            iteration_started.set()
            resume_iteration.wait(2.0)
            yield (-0.9, 0.1, 0.0)

        self.module.point_cloud2.read_points = lambda message, **kwargs: blocked_points()
        cloud_thread, cloud_errors = _capturing_thread(
            lambda: self.node._on_cloud(self._cloud(5.0, [(-0.9, 0.1, 0.0)]))
        )

        def replace_map():
            map_started.set()
            self.node._on_map(new_map)
            map_done.set()

        map_thread, map_errors = _capturing_thread(replace_map)
        cloud_thread.start()
        try:
            self.assertTrue(iteration_started.wait(1.0))
            map_thread.start()
            self.assertTrue(map_started.wait(1.0))
            self.assertTrue(map_done.wait(0.5))
        finally:
            resume_iteration.set()
            cloud_thread.join(2.0)
            map_thread.join(2.0)

        self.assertFalse(cloud_thread.is_alive())
        self.assertFalse(map_thread.is_alive())
        self.assertEqual(cloud_errors, [])
        self.assertEqual(map_errors, [])
        self.assertEqual(points_publisher.messages, [])
        self.assertEqual(grid_publisher.messages, [])

    def test_newer_cloud_invalidates_blocked_older_cloud_before_tracker_update(self):
        """Letting an older cloud update after a newer one would clear the latest layer as a false source reversal."""
        self.node._on_map(self._released_map())
        self.tf_buffer.transform = self._translation_transform(1.0, 0.0, 0.0)
        points_publisher, _ = [entry[3] for entry in self.ros.publishers]
        older_started = threading.Event()
        resume_older = threading.Event()
        older_cloud = self._cloud(5.0, [(-0.9, 0.1, 0.0)])
        older_cloud.block = True
        newer_cloud = self._cloud(6.0, [(0.1, 1.1, 0.0)])

        def read_points(message, **kwargs):
            if getattr(message, "block", False):
                older_started.set()
                resume_older.wait(2.0)
            return iter(message.points)

        self.module.point_cloud2.read_points = read_points
        older_thread, older_errors = _capturing_thread(
            lambda: self.node._on_cloud(older_cloud)
        )
        newer_thread, newer_errors = _capturing_thread(
            lambda: self.node._on_cloud(newer_cloud)
        )
        older_thread.start()
        try:
            self.assertTrue(older_started.wait(1.0))
            newer_thread.start()
            newer_thread.join(2.0)
            self.assertFalse(newer_thread.is_alive())
            self.assertEqual(newer_errors, [])
            self.assertEqual(
                [message.header.stamp.to_sec() for message in points_publisher.messages], [6.0]
            )
        finally:
            resume_older.set()
            older_thread.join(2.0)

        self.assertFalse(older_thread.is_alive())
        self.assertEqual(older_errors, [])
        self.assertEqual(
            [message.header.stamp.to_sec() for message in points_publisher.messages], [6.0]
        )
        self.assertEqual(self.node._tracker.snapshot(6.0), {(1, 1)})
        self.assertIsNone(self.node._tracker.last_reset_reason)

    def test_latest_started_blocked_cloud_wins_when_older_preparation_finishes_first(self):
        """Allowing an old preparation to commit first would let completion order override cloud arrival order."""
        self.node._on_map(self._released_map())
        self.tf_buffer.transform = self._translation_transform(1.0, 0.0, 0.0)
        points_publisher, grid_publisher = [entry[3] for entry in self.ros.publishers]
        older_started = threading.Event()
        newer_started = threading.Event()
        release_older = threading.Event()
        release_newer = threading.Event()
        older_cloud = self._cloud(5.0, [(-0.9, 0.1, 0.0)])
        newer_cloud = self._cloud(6.0, [(0.1, 1.1, 0.0)])
        older_cloud.preparation_started = older_started
        older_cloud.resume_preparation = release_older
        newer_cloud.preparation_started = newer_started
        newer_cloud.resume_preparation = release_newer

        def read_points(message, **kwargs):
            message.preparation_started.set()
            message.resume_preparation.wait(2.0)
            return iter(message.points)

        self.module.point_cloud2.read_points = read_points
        older_thread, older_errors = _capturing_thread(
            lambda: self.node._on_cloud(older_cloud)
        )
        newer_thread, newer_errors = _capturing_thread(
            lambda: self.node._on_cloud(newer_cloud)
        )
        older_thread.start()
        try:
            self.assertTrue(older_started.wait(1.0))
            newer_thread.start()
            self.assertTrue(newer_started.wait(1.0))
            release_older.set()
            older_thread.join(2.0)
            self.assertFalse(older_thread.is_alive())
            self.assertEqual(older_errors, [])
            self.assertEqual(points_publisher.messages, [])
            self.assertEqual(grid_publisher.messages, [])
            self.assertIsNone(self.node._tracker.last_reset_reason)
        finally:
            release_newer.set()
            older_thread.join(2.0)
            newer_thread.join(2.0)

        self.assertFalse(older_thread.is_alive())
        self.assertFalse(newer_thread.is_alive())
        self.assertEqual(older_errors, [])
        self.assertEqual(newer_errors, [])
        self.assertEqual(
            [message.header.stamp.to_sec() for message in points_publisher.messages], [6.0]
        )
        self.assertEqual(
            [message.header.stamp.to_sec() for message in grid_publisher.messages], [6.0]
        )
        self.assertEqual(self.node._tracker.snapshot(6.0), {(1, 1)})
        self.assertIsNone(self.node._tracker.last_reset_reason)

    def test_unsupported_point_field_datatype_drops_before_reading_points(self):
        """Delegating an unsupported PointCloud2 datatype to the Noetic helper can raise its internal NameError."""
        self.node._on_map(self._released_map())
        self.tf_buffer.transform = self._translation_transform(1.0, 0.0, 0.0)
        points_publisher, grid_publisher = [entry[3] for entry in self.ros.publishers]
        invalid_fields = [
            types.SimpleNamespace(name="x", offset=0, datatype=99, count=1),
            types.SimpleNamespace(name="y", offset=4, datatype=7, count=1),
            types.SimpleNamespace(name="z", offset=8, datatype=7, count=1),
        ]

        def should_not_read(*args, **kwargs):
            raise AssertionError("read_points must not receive unsupported metadata")

        self.module.point_cloud2.read_points = should_not_read
        self.node._on_cloud(self._cloud(5.0, [(-0.9, 0.1, 0.0)], invalid_fields))

        self.assertEqual(points_publisher.messages, [])
        self.assertEqual(grid_publisher.messages, [])

    def test_ignores_unrequested_pointcloud_fields_while_validating_xyz(self):
        """Rejecting an unrelated intensity field would discard ordinary leveled PointCloud2 messages."""
        self.node._on_map(self._released_map())
        self.tf_buffer.transform = self._translation_transform(1.0, 0.0, 0.0)
        fields = [
            types.SimpleNamespace(name="x", offset=0, datatype=7, count=1),
            types.SimpleNamespace(name="y", offset=4, datatype=7, count=1),
            types.SimpleNamespace(name="z", offset=8, datatype=7, count=1),
            types.SimpleNamespace(name="intensity", offset=12, datatype=1, count=1),
        ]

        self.node._on_cloud(self._cloud(5.0, [(-0.9, 0.1, 0.0)], fields))

        points_publisher, _ = [entry[3] for entry in self.ros.publishers]
        self.assertEqual(points_publisher.messages[-1].points, [(0.5, 0.5, 0.0)])

class _LookupException(Exception):
    pass


if __name__ == "__main__":
    unittest.main()
