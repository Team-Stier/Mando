#!/usr/bin/env python3
"""Bounded degraded-pipeline smoke without RTAB-Map or hardware drivers."""

import math
import threading
import time
import unittest

import rosgraph
import rosnode
import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import NavSatFix, NavSatStatus, PointCloud2
from sensor_msgs import point_cloud2
from tf2_msgs.msg import TFMessage


TOPIC_TYPES = {
    "/clock": Clock,
    "/tf": TFMessage,
    "/tf_static": TFMessage,
    "/slam/odometry/wheel": Odometry,
    "/slam/odometry/local": Odometry,
    "/slam/gnss/fix_accepted": NavSatFix,
    "/slam/scan/leveled_points": PointCloud2,
    "/slam/map/released": OccupancyGrid,
    "/slam/diagnostics": DiagnosticArray,
    "/slam/diagnostics/rtk_gate": DiagnosticStatus,
    "/slam/live_obstacles/points": PointCloud2,
    "/slam/live_obstacles/grid": OccupancyGrid,
}

TOPIC_TYPE_NAMES = {
    "/clock": "rosgraph_msgs/Clock",
    "/tf": "tf2_msgs/TFMessage",
    "/tf_static": "tf2_msgs/TFMessage",
    "/slam/odometry/wheel": "nav_msgs/Odometry",
    "/slam/odometry/local": "nav_msgs/Odometry",
    "/slam/gnss/fix_accepted": "sensor_msgs/NavSatFix",
    "/slam/scan/leveled_points": "sensor_msgs/PointCloud2",
    "/slam/map/released": "nav_msgs/OccupancyGrid",
    "/slam/diagnostics": "diagnostic_msgs/DiagnosticArray",
    "/slam/diagnostics/rtk_gate": "diagnostic_msgs/DiagnosticStatus",
    "/slam/live_obstacles/points": "sensor_msgs/PointCloud2",
    "/slam/live_obstacles/grid": "nav_msgs/OccupancyGrid",
}

STAMPED_TOPICS = (
    "/slam/odometry/wheel",
    "/slam/odometry/local",
    "/slam/gnss/fix_accepted",
    "/slam/scan/leveled_points",
    "/slam/diagnostics",
    "/slam/live_obstacles/points",
    "/slam/live_obstacles/grid",
)

RATE_BOUNDS_HZ = {
    "/clock": (14.0, 26.0),
    "/tf": (10.0, 35.0),
    "/slam/odometry/wheel": (14.0, 26.0),
    "/slam/odometry/local": (10.0, 35.0),
    "/slam/gnss/fix_accepted": (14.0, 26.0),
    "/slam/scan/leveled_points": (14.0, 26.0),
    "/slam/diagnostics": (3.0, 8.0),
    "/slam/diagnostics/rtk_gate": (14.0, 26.0),
    "/slam/live_obstacles/points": (8.0, 35.0),
    "/slam/live_obstacles/grid": (8.0, 35.0),
}


def _clock_reaches_target(clock_entries, target_stamp_nsec):
    """Return true only when monotonic /clock receipts cover a source stamp."""
    stamps_nsec = [message.clock.to_nsec() for _, message in clock_entries]
    return bool(stamps_nsec) and all(
        previous <= current
        for previous, current in zip(stamps_nsec, stamps_nsec[1:])
    ) and stamps_nsec[-1] >= target_stamp_nsec


class SyntheticPipelineSmokeTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rospy.core.is_initialized():
            rospy.init_node("synthetic_pipeline_smoke_test", anonymous=True)

    def setUp(self):
        self._condition = threading.Condition()
        self._samples = {topic: [] for topic in TOPIC_TYPES}
        self._subscribers = [
            rospy.Subscriber(
                topic,
                message_type,
                self._callback,
                callback_args=topic,
                queue_size=100,
                tcp_nodelay=True,
            )
            for topic, message_type in TOPIC_TYPES.items()
        ]

    def tearDown(self):
        for subscriber in self._subscribers:
            subscriber.unregister()

    def _callback(self, message, topic):
        with self._condition:
            self._samples[topic].append((time.monotonic(), message))
            self._condition.notify_all()

    def _wait_for(self, predicate, timeout_sec, description):
        deadline = time.monotonic() + timeout_sec
        with self._condition:
            while not predicate():
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    counts = {
                        topic: len(samples)
                        for topic, samples in self._samples.items()
                    }
                    self.fail(
                        "timeout waiting for {}; sample_counts={}".format(
                            description, counts
                        )
                    )
                self._condition.wait(min(remaining, 0.1))

    def _window(self, topic, start):
        with self._condition:
            return [entry for entry in self._samples[topic] if entry[0] >= start]

    @staticmethod
    def _receipt_rate(entries):
        elapsed = entries[-1][0] - entries[0][0]
        if elapsed <= 0.0:
            return math.inf
        return (len(entries) - 1) / elapsed

    def test_clock_target_wait_covers_multiple_delayed_callbacks(self):
        """The wait must not stop until /clock covers the frozen sensor window."""
        clock_entries = [
            (10.01, Clock(clock=rospy.Time(100, 50000000))),
            (10.06, Clock(clock=rospy.Time(100, 100000000))),
            (10.11, Clock(clock=rospy.Time(100, 150000000))),
        ]
        sensor_source_stamp_nsec = 100150000000

        self.assertFalse(
            _clock_reaches_target(clock_entries[:1], sensor_source_stamp_nsec)
        )
        self.assertFalse(
            _clock_reaches_target(clock_entries[:2], sensor_source_stamp_nsec)
        )
        self.assertTrue(
            _clock_reaches_target(clock_entries, sensor_source_stamp_nsec)
        )

    def test_processed_pipeline_topics_frames_rates_and_expiry(self):
        """A disconnected preprocessing node or unbounded live layer must fail the smoke."""
        self._wait_for(
            lambda: all(self._samples[topic] for topic in TOPIC_TYPES),
            12.0,
            "all processed-pipeline topics",
        )
        collection_start = time.monotonic()
        collection_duration_sec = 4.5
        self._wait_for(
            lambda: time.monotonic() - collection_start >= collection_duration_sec,
            collection_duration_sec + 1.0,
            "bounded rate and expiry window",
        )
        collection_end = time.monotonic()
        # Freeze the processed windows before waiting: callbacks from different
        # TCPROS topics have no cross-topic receipt ordering, so the first
        # post-boundary /clock callback may still trail multiple sensor ticks.
        with self._condition:
            windows = {
                topic: [
                    entry for entry in self._samples[topic]
                    if collection_start <= entry[0] <= collection_end
                ]
                for topic in RATE_BOUNDS_HZ
            }
        frozen_stamp_nsecs = [
            message.header.stamp.to_nsec()
            for topic in STAMPED_TOPICS
            for _, message in windows[topic]
        ]
        self.assertTrue(frozen_stamp_nsecs, "frozen sensor window has no stamps")
        target_max_stamp_nsec = max(frozen_stamp_nsecs)
        self._wait_for(
            lambda: _clock_reaches_target(
                self._samples["/clock"], target_max_stamp_nsec
            ),
            1.0,
            "clock source stamp covering the frozen sensor window",
        )
        with self._condition:
            captured_samples = {
                topic: list(entries) for topic, entries in self._samples.items()
            }
        self.assertIs(rospy.get_param("/use_sim_time"), True)
        self.assertIs(rospy.get_param("/wheel_odometry/calibration_required"), True)
        self.assertFalse(rospy.has_param("/ekf_local/transform_time_offset"))
        published_types = dict(rospy.get_published_topics())
        for topic, expected_type in TOPIC_TYPE_NAMES.items():
            self.assertEqual(published_types.get(topic), expected_type, topic)
        for topic, (minimum_hz, maximum_hz) in RATE_BOUNDS_HZ.items():
            entries = windows[topic]
            self.assertGreaterEqual(
                len(entries), 3, "not enough samples to measure {}".format(topic)
            )
            rate_hz = self._receipt_rate(entries)
            print(
                "SMOKE_RATE topic={} samples={} rate_hz={:.3f}".format(
                    topic, len(entries), rate_hz
                ),
                flush=True,
            )
            self.assertGreaterEqual(rate_hz, minimum_hz, topic)
            self.assertLessEqual(rate_hz, maximum_hz, topic)

        clock_stamps_nsec = [
            message.clock.to_nsec() for _, message in windows["/clock"]
        ]
        self.assertTrue(all(stamp > 0 for stamp in clock_stamps_nsec))
        self.assertTrue(all(
            current - previous == 50000000
            for previous, current in zip(clock_stamps_nsec, clock_stamps_nsec[1:])
        ))
        self.assertTrue(_clock_reaches_target(
            captured_samples["/clock"], target_max_stamp_nsec
        ))
        latest_clock_nsec = captured_samples["/clock"][-1][1].clock.to_nsec()

        for topic in STAMPED_TOPICS:
            messages = [message for _, message in windows[topic]]
            stamps_nsec = [message.header.stamp.to_nsec() for message in messages]
            self.assertTrue(stamps_nsec, topic)
            self.assertTrue(all(stamp > 0 for stamp in stamps_nsec), topic)
            self.assertTrue(
                all(
                    previous <= current
                    for previous, current in zip(stamps_nsec, stamps_nsec[1:])
                ),
                "non-monotonic source stamps on {}".format(topic),
            )
            self.assertLessEqual(max(stamps_nsec), latest_clock_nsec, topic)

        wheel = windows["/slam/odometry/wheel"][-1][1]
        self.assertEqual(wheel.header.frame_id, "odom")
        self.assertEqual(wheel.child_frame_id, "base_link")
        self.assertTrue(math.isfinite(wheel.twist.twist.linear.x))
        self.assertEqual(len(wheel.pose.covariance), 36)
        self.assertTrue(all(math.isfinite(value) for value in wheel.pose.covariance))

        local = windows["/slam/odometry/local"][-1][1]
        self.assertEqual(local.header.frame_id, "odom")
        self.assertEqual(local.child_frame_id, "base_link")
        self.assertTrue(math.isfinite(local.pose.pose.position.x))
        self.assertEqual(len(local.pose.covariance), 36)
        self.assertTrue(all(math.isfinite(value) for value in local.pose.covariance))

        accepted_fix = windows["/slam/gnss/fix_accepted"][-1][1]
        self.assertEqual(accepted_fix.header.frame_id, "gps_link")
        self.assertEqual(accepted_fix.status.status, NavSatStatus.STATUS_GBAS_FIX)

        leveled = windows["/slam/scan/leveled_points"][-1][1]
        self.assertEqual(leveled.header.frame_id, "base_link")
        self.assertGreater(leveled.width, 0)
        self.assertEqual([field.name for field in leveled.fields], ["x", "y", "z"])
        leveled_xyz = list(point_cloud2.read_points(
            leveled, field_names=("x", "y", "z"), skip_nans=False
        ))
        self.assertEqual(len(leveled_xyz), leveled.width)
        self.assertTrue(all(
            all(math.isfinite(coordinate) for coordinate in point)
            and -0.5 <= point[2] <= 0.5
            for point in leveled_xyz
        ))

        released_map = captured_samples["/slam/map/released"][-1][1]
        self.assertEqual(released_map.header.frame_id, "map")
        self.assertEqual((released_map.info.width, released_map.info.height), (26, 18))
        self.assertEqual(len(released_map.data), 26 * 18)
        self.assertEqual(set(released_map.data), {0, 100})

        diagnostics = [
            message
            for _, message in windows["/slam/diagnostics"]
        ]
        status_by_name = {
            status.name: status
            for message in diagnostics
            for status in message.status
        }
        expected_health_names = {
            "sensor_health/scan_raw",
            "sensor_health/imu",
            "sensor_health/gnss_fix",
            "sensor_health/gnss_rtk_status",
            "sensor_health/vehicle_speed",
            "sensor_health/steering_angle",
            "sensor_health/camera_front_image",
            "sensor_health/aggregate",
        }
        self.assertTrue(expected_health_names.issubset(status_by_name))
        self.assertEqual(
            status_by_name["sensor_health/camera_front_image"].message,
            "missing",
        )
        self.assertEqual(
            status_by_name["sensor_health/camera_front_image"].level,
            DiagnosticStatus.ERROR,
        )
        for name in expected_health_names - {
                "sensor_health/camera_front_image", "sensor_health/aggregate"}:
            self.assertEqual(status_by_name[name].level, DiagnosticStatus.OK, name)
        aggregate = status_by_name["sensor_health/aggregate"]
        aggregate_values = {value.key: value.value for value in aggregate.values}
        self.assertEqual(aggregate.message, "MISSING")
        self.assertEqual(aggregate.level, DiagnosticStatus.ERROR)
        self.assertEqual(aggregate_values.get("state"), "MISSING")
        self.assertEqual(aggregate_values.get("worst_sensor"), "camera_front_image")

        rtk_diagnostics = [
            message for _, message in windows["/slam/diagnostics/rtk_gate"]
        ]
        self.assertTrue(any(message.message == "accepted" for message in rtk_diagnostics))

        point_messages = [
            message for _, message in windows["/slam/live_obstacles/points"]
        ]
        def sample_index(message):
            raw_index = (message.header.stamp.to_sec() - 100.0) * 20.0
            index = int(round(raw_index))
            self.assertAlmostEqual(raw_index, index, places=6)
            return index

        def forward_obstacle_points(message):
            return [
                point
                for point in point_cloud2.read_points(
                    message, field_names=("x", "y", "z"), skip_nans=True
                )
                if 0.5 < point[0] < 2.0 and abs(point[1]) < 0.5
            ]

        live_grids = [message for _, message in windows["/slam/live_obstacles/grid"]]
        points_by_cycle_phase = {}
        for message in point_messages:
            index = sample_index(message)
            points_by_cycle_phase.setdefault((index // 40, index % 40), []).append(message)
        grids_by_cycle_phase = {}
        for message in live_grids:
            index = sample_index(message)
            grids_by_cycle_phase.setdefault((index // 40, index % 40), []).append(message)

        candidate_cycles = sorted({key[0] for key in points_by_cycle_phase})
        verified_cycle = None
        for cycle in candidate_cycles:
            point_observed_phases = {
                phase for candidate_cycle, phase in points_by_cycle_phase
                if candidate_cycle == cycle
            }
            grid_observed_phases = {
                phase for candidate_cycle, phase in grids_by_cycle_phase
                if candidate_cycle == cycle
            }
            jointly_observed_phases = point_observed_phases & grid_observed_phases
            pre_obstacle_phases = sorted(jointly_observed_phases & set(range(0, 10)))
            active_obstacle_phases = sorted(jointly_observed_phases & set(range(10, 20)))
            expiry_phases = sorted(jointly_observed_phases & set(range(34, 40)))
            point_active_phases = [
                phase for phase in active_obstacle_phases
                if any(forward_obstacle_points(message) for message in points_by_cycle_phase.get(
                    (cycle, phase), []
                ))
            ]
            grid_active_phases = [
                phase for phase in active_obstacle_phases
                if any(100 in message.data for message in grids_by_cycle_phase.get(
                    (cycle, phase), []
                ))
            ]
            point_expiry_nonempty = [
                phase for phase in range(34, 40)
                if any(forward_obstacle_points(message) for message in points_by_cycle_phase.get(
                    (cycle, phase), []
                ))
            ]
            grid_expiry_nonempty = [
                phase for phase in range(34, 40)
                if any(100 in message.data for message in grids_by_cycle_phase.get(
                    (cycle, phase), []
                ))
            ]
            print(
                "SMOKE_PHASE cycle={} observed_pre={} observed_active={} "
                "observed_expiry={} active_points={} active_grids={} "
                "expiry_points={} expiry_grids={}".format(
                    cycle,
                    pre_obstacle_phases,
                    active_obstacle_phases,
                    expiry_phases,
                    point_active_phases,
                    grid_active_phases,
                    point_expiry_nonempty,
                    grid_expiry_nonempty,
                ),
                flush=True,
            )
            # The EKF publishes odom -> base_link at about 15 Hz while the fixture
            # scan is 20 Hz.  Exact-stamp TF lookup therefore rejects a few scan
            # phases by design.  Require broad, same-cycle phase coverage instead
            # of weakening timestamp alignment or inventing interpolated evidence.
            if (len(pre_obstacle_phases) < 3
                    or len(active_obstacle_phases) < 6
                    or len(expiry_phases) < 3):
                continue
            pre_obstacle_points_clear = all(all(
                not forward_obstacle_points(message)
                for message in points_by_cycle_phase[(cycle, phase)]
            ) for phase in pre_obstacle_phases)
            pre_obstacle_grids_clear = all(all(
                100 not in message.data
                for message in grids_by_cycle_phase[(cycle, phase)]
            ) for phase in pre_obstacle_phases)
            active_phases = set(point_active_phases) & set(grid_active_phases)
            expired_points = all(all(
                not forward_obstacle_points(message)
                for message in points_by_cycle_phase[(cycle, phase)]
            ) for phase in expiry_phases)
            expired_grids = all(all(
                100 not in message.data
                for message in grids_by_cycle_phase[(cycle, phase)]
            ) for phase in expiry_phases)
            if (pre_obstacle_points_clear
                    and pre_obstacle_grids_clear
                    and len(active_phases) >= 6
                    and expired_points
                    and expired_grids):
                verified_cycle = cycle
                break
        self.assertIsNotNone(
            verified_cycle,
            "no synthetic cycle had clear pre-obstacle output, at least six "
            "same-stamp active phases, and at least three expired phases",
        )

        for grid in live_grids:
            self.assertEqual(grid.header.frame_id, "map")
            self.assertEqual((grid.info.width, grid.info.height), (40, 20))
            self.assertAlmostEqual(grid.info.resolution, 0.1)
            self.assertEqual(len(grid.data), 800)
            self.assertTrue(set(grid.data).issubset({-1, 100}))
            self.assertLess(
                grid.info.width * grid.info.resolution,
                released_map.info.width * released_map.info.resolution,
            )
            self.assertLess(
                grid.info.height * grid.info.resolution,
                released_map.info.height * released_map.info.resolution,
            )

        node_names = rosnode.get_node_names()
        self.assertIn("/synthetic_map_to_odom_fixture", node_names)
        self.assertFalse(
            any(name.rsplit("/", 1)[-1] == "rtabmap" for name in node_names),
            "degraded smoke must not start or claim RTAB-Map",
        )
        publishers, _subscribers, _services = rosgraph.Master(
            rospy.get_name()
        ).getSystemState()
        publishers_by_topic = {topic: set(nodes) for topic, nodes in publishers}
        self.assertEqual(publishers_by_topic.get("/tf"), {"/ekf_local"})
        self.assertEqual(
            publishers_by_topic.get("/tf_static"),
            {"/synthetic_sensor", "/synthetic_map_to_odom_fixture"},
        )

        static_transforms = [
            transform
            for _, message in captured_samples["/tf_static"]
            for transform in message.transforms
        ]
        fixture_transforms = [
            transform for transform in static_transforms
            if transform.header.frame_id == "map" and transform.child_frame_id == "odom"
        ]
        self.assertEqual(len(fixture_transforms), 1)
        fixture = fixture_transforms[0].transform
        self.assertEqual(
            (fixture.translation.x, fixture.translation.y, fixture.translation.z),
            (0.0, 0.0, 0.0),
        )
        self.assertEqual(
            (fixture.rotation.x, fixture.rotation.y, fixture.rotation.z, fixture.rotation.w),
            (0.0, 0.0, 0.0, 1.0),
        )
        dynamic_edges = [
            transform
            for _, message in windows["/tf"]
            for transform in message.transforms
            if transform.header.frame_id == "odom"
            and transform.child_frame_id == "base_link"
        ]
        self.assertTrue(dynamic_edges)
        self.assertTrue(all(
            transform.header.stamp.to_sec() > 0.0
            and all(math.isfinite(value) for value in (
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ))
            for transform in dynamic_edges
        ))


if __name__ == "__main__":
    import rostest

    rostest.rosrun(
        "stier_slam_bringup",
        "synthetic_pipeline_smoke",
        SyntheticPipelineSmokeTest,
    )
