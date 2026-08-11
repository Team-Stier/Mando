#!/usr/bin/env python3
"""Runtime contracts for the coherent, hardware-free SLAM demonstration."""

import math
import os
from collections import Counter
import threading
import time
import unittest
import xmlrpc.client

import rosgraph
import rosnode
import rospy
import yaml
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, NavSatStatus
from std_msgs.msg import Float64
from tf2_msgs.msg import TFMessage


PACKAGE_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SEQUENCE_START_SEC = 100.0
PUBLISH_RATE_HZ = 20.0
PERIOD_SEC = 1.0 / PUBLISH_RATE_HZ
CYCLE_SAMPLES = 40
OBSTACLE_START_SAMPLE = 10
OBSTACLE_END_SAMPLE = 20
TARGET_SAMPLE_COUNT = 80
EXACT_WINDOW_SAMPLE_COUNT = CYCLE_SAMPLES + 1
BASE_LATITUDE = 37.239
BASE_LONGITUDE = 126.773
EARTH_RADIUS_M = 6378137.0
STATIC_WALL_X_M = 3.125
STATIC_WALL_Y_M = 2.125
METERS_PER_DEGREE_LONGITUDE = (
    math.pi * EARTH_RADIUS_M * math.cos(math.radians(BASE_LATITUDE)) / 180.0
)

NORMALIZED_TOPICS = {
    "/slam/input/scan_raw": (LaserScan, "sensor_msgs/LaserScan"),
    "/slam/input/imu": (Imu, "sensor_msgs/Imu"),
    "/slam/input/gnss/fix": (NavSatFix, "sensor_msgs/NavSatFix"),
    "/slam/input/gnss/rtk_status": (
        DiagnosticStatus,
        "diagnostic_msgs/DiagnosticStatus",
    ),
    "/slam/input/vehicle_speed": (
        TwistWithCovarianceStamped,
        "geometry_msgs/TwistWithCovarianceStamped",
    ),
    "/slam/input/steering_angle": (Float64, "std_msgs/Float64"),
}

STAMPED_TOPICS = (
    "/slam/input/scan_raw",
    "/slam/input/imu",
    "/slam/input/gnss/fix",
    "/slam/input/vehicle_speed",
)

WHEEL_TOPIC = "/slam/odometry/wheel"
MAP_TOPIC = "/slam/test/demo_map"


class SyntheticTopicsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.init_node("synthetic_topics_contract_test", anonymous=True)
        cls._condition = threading.Condition()
        cls._messages = {topic: [] for topic in NORMALIZED_TOPICS}
        cls._messages[WHEEL_TOPIC] = []
        cls._receipts = {topic: [] for topic in cls._messages}
        cls._clock_messages = []
        cls._tf_static_messages = []
        cls._map_messages = []
        cls._subscribers = []

        for topic, (message_type, _) in NORMALIZED_TOPICS.items():
            cls._subscribers.append(rospy.Subscriber(
                topic,
                message_type,
                cls._on_message,
                callback_args=topic,
                queue_size=200,
            ))
        cls._subscribers.append(rospy.Subscriber(
            WHEEL_TOPIC,
            Odometry,
            cls._on_message,
            callback_args=WHEEL_TOPIC,
            queue_size=200,
        ))
        cls._subscribers.append(rospy.Subscriber(
            "/clock", Clock, cls._on_clock, queue_size=200
        ))
        cls._subscribers.append(rospy.Subscriber(
            "/tf_static", TFMessage, cls._on_tf_static, queue_size=10
        ))
        cls._subscribers.append(rospy.Subscriber(
            MAP_TOPIC, OccupancyGrid, cls._on_map, queue_size=2
        ))

        deadline = time.monotonic() + 7.0
        with cls._condition:
            while time.monotonic() < deadline and not cls._samples_ready():
                cls._condition.wait(timeout=min(0.1, deadline - time.monotonic()))

    @classmethod
    def tearDownClass(cls):
        for subscriber in cls._subscribers:
            subscriber.unregister()

    @classmethod
    def _on_message(cls, message, topic):
        with cls._condition:
            if len(cls._messages[topic]) < TARGET_SAMPLE_COUNT:
                cls._messages[topic].append(message)
                cls._receipts[topic].append(time.monotonic())
            cls._condition.notify_all()

    @classmethod
    def _on_clock(cls, message):
        with cls._condition:
            if len(cls._clock_messages) < TARGET_SAMPLE_COUNT:
                cls._clock_messages.append(message)
            cls._condition.notify_all()

    @classmethod
    def _on_tf_static(cls, message):
        with cls._condition:
            cls._tf_static_messages.append(message)
            cls._condition.notify_all()

    @classmethod
    def _on_map(cls, message):
        with cls._condition:
            cls._map_messages.append(message)
            cls._condition.notify_all()

    @classmethod
    def _samples_ready(cls):
        return (
            all(len(messages) >= TARGET_SAMPLE_COUNT for messages in cls._messages.values())
            and len(cls._clock_messages) >= TARGET_SAMPLE_COUNT
            and bool(cls._tf_static_messages)
            and bool(cls._map_messages)
        )

    def _require_samples(self, topic, count=10):
        messages = self._messages[topic]
        self.assertGreaterEqual(
            len(messages),
            count,
            "missing publisher or too few samples on {}: got {}".format(topic, len(messages)),
        )
        return messages

    def test_normalized_topics_have_exact_types_and_ten_samples(self):
        published_types = dict(rospy.get_published_topics("/"))
        for topic, (_, expected_type) in NORMALIZED_TOPICS.items():
            messages = self._require_samples(topic)
            self.assertEqual(published_types.get(topic), expected_type)
            self.assertTrue(all(message._type == expected_type for message in messages[:10]))
        wheel_messages = self._require_samples(WHEEL_TOPIC)
        self.assertEqual(published_types.get(WHEEL_TOPIC), "nav_msgs/Odometry")
        self.assertTrue(all(message._type == "nav_msgs/Odometry" for message in wheel_messages))

    def test_all_sensor_receipt_rates_are_at_least_eighteen_hz(self):
        for topic in NORMALIZED_TOPICS:
            self._require_samples(topic, 40)
            self.assertGreaterEqual(_receipt_rate(self._receipts[topic][:40]), 18.0, topic)

    def test_stamped_topics_share_exact_clock_stamps_and_frames(self):
        expected_step_nsec = 50000000
        stamp_sequences = {}
        for topic in STAMPED_TOPICS:
            messages = self._require_samples(topic, TARGET_SAMPLE_COUNT)
            stamps = [message.header.stamp.to_nsec() for message in messages]
            self.assertTrue(all(message.header.frame_id.strip() for message in messages[:10]))
            self.assertTrue(all(later > earlier for earlier, later in zip(stamps, stamps[1:])))
            stamp_sequences[topic] = stamps

        self.assertGreaterEqual(len(self._clock_messages), TARGET_SAMPLE_COUNT)
        stamp_sequences["/clock"] = [
            message.clock.to_nsec() for message in self._clock_messages
        ]
        wheel_messages = self._require_samples(WHEEL_TOPIC, TARGET_SAMPLE_COUNT)
        stamp_sequences[WHEEL_TOPIC] = [
            message.header.stamp.to_nsec() for message in wheel_messages
        ]

        window = _fixed_contiguous_window(
            stamp_sequences,
            expected_step_nsec,
            EXACT_WINDOW_SAMPLE_COUNT,
        )
        self.assertEqual(len(window), EXACT_WINDOW_SAMPLE_COUNT)

    def test_fixed_window_helper_rejects_clock_and_sensor_internal_gaps(self):
        step_nsec = 50000000
        complete = [100000000000 + index * step_nsec for index in range(80)]
        window = _fixed_contiguous_window(
            {"sensor": complete, "clock": complete},
            step_nsec,
            EXACT_WINDOW_SAMPLE_COUNT,
        )
        self.assertEqual(window, tuple(complete[:EXACT_WINDOW_SAMPLE_COUNT]))

        fault_sequences = (
            {"sensor": complete, "clock": complete[:20]},
            {
                "sensor": complete,
                "clock": [
                    stamp for index, stamp in enumerate(complete)
                    if index % 10 != 0
                ],
            },
            {
                "sensor": complete[:20] + complete[21:],
                "clock": complete,
            },
        )
        for sequences in fault_sequences:
            with self.subTest(sequences=sequences):
                with self.assertRaises(AssertionError):
                    _fixed_contiguous_window(
                        sequences,
                        step_nsec,
                        EXACT_WINDOW_SAMPLE_COUNT,
                    )

    def test_imu_is_identity_and_rtk_is_fixed_in_gps_frame(self):
        imu_messages = self._require_samples("/slam/input/imu")
        for message in imu_messages[:10]:
            self.assertEqual(
                (
                    message.orientation.x,
                    message.orientation.y,
                    message.orientation.z,
                    message.orientation.w,
                ),
                (0.0, 0.0, 0.0, 1.0),
            )
            self.assertEqual(message.angular_velocity.z, 0.0)
            self.assertNotEqual(message.orientation_covariance[0], -1.0)

        fixes = self._require_samples("/slam/input/gnss/fix")
        for message in fixes[:10]:
            self.assertEqual(message.header.frame_id, "gps_link")
            self.assertEqual(message.status.status, NavSatStatus.STATUS_GBAS_FIX)
            self.assertNotEqual(
                message.position_covariance_type,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN,
            )
            self.assertTrue(math.isfinite(message.latitude))
            self.assertTrue(math.isfinite(message.longitude))

        statuses = self._require_samples("/slam/input/gnss/rtk_status")
        for message in statuses[:10]:
            self.assertEqual(message.level, DiagnosticStatus.OK)
            self.assertEqual(message.message, "FIX")
            self.assertTrue(message.name.strip())

    def test_speed_steering_gnss_and_scan_form_one_closed_out_and_back_cycle(self):
        speeds = self._require_samples("/slam/input/vehicle_speed", TARGET_SAMPLE_COUNT)
        steering = self._require_samples("/slam/input/steering_angle", TARGET_SAMPLE_COUNT)
        fixes_by_stamp = _by_stamp(self._require_samples(
            "/slam/input/gnss/fix", TARGET_SAMPLE_COUNT
        ))
        imu_by_stamp = _by_stamp(self._require_samples(
            "/slam/input/imu", TARGET_SAMPLE_COUNT
        ))
        scans_by_stamp = _by_stamp(self._require_samples(
            "/slam/input/scan_raw", TARGET_SAMPLE_COUNT
        ))

        self.assertTrue(all(abs(message.data) <= 1.0e-12 for message in steering))
        state_by_index = {}
        for speed in speeds:
            sample_index = _sample_index(speed.header.stamp)
            expected_speed = _expected_speed(sample_index)
            expected_x = _expected_pose_x(sample_index)
            self.assertAlmostEqual(speed.twist.twist.linear.x, expected_speed, places=12)
            diagonal = [speed.twist.covariance[index] for index in (0, 7, 14, 21, 28, 35)]
            self.assertAlmostEqual(diagonal[0], 0.01)
            self.assertTrue(all(value >= 100000.0 for value in diagonal[1:]))

            stamp = speed.header.stamp.to_nsec()
            if (
                    stamp not in fixes_by_stamp
                    or stamp not in imu_by_stamp
                    or stamp not in scans_by_stamp):
                continue
            fix = fixes_by_stamp[stamp]
            imu = imu_by_stamp[stamp]
            scan = scans_by_stamp[stamp]
            measured_x = (fix.longitude - BASE_LONGITUDE) * METERS_PER_DEGREE_LONGITUDE
            self.assertAlmostEqual(fix.latitude, BASE_LATITUDE, places=10)
            self.assertAlmostEqual(measured_x, expected_x, places=5)
            self.assertAlmostEqual(
                imu.linear_acceleration.x,
                _expected_acceleration_x(sample_index),
                places=12,
            )
            self._assert_scan_matches_pose(scan, sample_index, expected_x)
            state_by_index[sample_index] = (
                speed.twist.twist.linear.x,
                fix.latitude,
                fix.longitude,
                tuple(scan.ranges),
            )

        closed_pairs = 0
        for sample_index, state in state_by_index.items():
            if sample_index + CYCLE_SAMPLES in state_by_index:
                self.assertEqual(state, state_by_index[sample_index + CYCLE_SAMPLES])
                closed_pairs += 1
        self.assertGreaterEqual(closed_pairs, 10)

    def _assert_scan_matches_pose(self, scan, sample_index, pose_x):
        self.assertEqual(len(scan.ranges), 360)
        self.assertAlmostEqual(scan.angle_increment * len(scan.ranges), 2.0 * math.pi)
        self.assertAlmostEqual(
            scan.angle_max,
            scan.angle_min + (len(scan.ranges) - 1) * scan.angle_increment,
            places=6,
        )
        self.assertTrue(all(
            math.isfinite(distance)
            and scan.range_min <= distance <= scan.range_max
            for distance in scan.ranges
        ))

        front_index = int(round((0.0 - scan.angle_min) / scan.angle_increment))
        side_index = int(round((math.pi * 0.5 - scan.angle_min) / scan.angle_increment))
        obstacle_active = (
            OBSTACLE_START_SAMPLE
            <= sample_index % CYCLE_SAMPLES
            < OBSTACLE_END_SAMPLE
        )
        self.assertAlmostEqual(
            scan.ranges[front_index],
            1.0 if obstacle_active else STATIC_WALL_X_M - pose_x,
            places=5,
        )
        self.assertAlmostEqual(scan.ranges[0], STATIC_WALL_X_M + pose_x, places=5)
        self.assertAlmostEqual(scan.ranges[side_index], STATIC_WALL_Y_M, places=5)

        if not obstacle_active:
            served = self._map_messages[-1]
            for beam_index, distance in enumerate(scan.ranges):
                angle = scan.angle_min + beam_index * scan.angle_increment
                endpoint_x = pose_x + distance * math.cos(angle)
                endpoint_y = distance * math.sin(angle)
                self.assertGreaterEqual(endpoint_x, -3.25 - 1.0e-5)
                self.assertLess(endpoint_x, 3.25)
                self.assertGreaterEqual(endpoint_y, -2.25 - 1.0e-5)
                self.assertLess(endpoint_y, 2.25)
                column = int(math.floor(
                    (endpoint_x - served.info.origin.position.x) / served.info.resolution
                ))
                row = int(math.floor(
                    (endpoint_y - served.info.origin.position.y) / served.info.resolution
                ))
                self.assertGreaterEqual(column, 0)
                self.assertLess(column, served.info.width)
                self.assertGreaterEqual(row, 0)
                self.assertLess(row, served.info.height)
                self.assertGreaterEqual(
                    served.data[row * served.info.width + column],
                    65,
                )

    def test_wheel_odometry_covers_speed_stamps_at_input_rate_and_closes_cycle(self):
        speed_messages = self._require_samples(
            "/slam/input/vehicle_speed", TARGET_SAMPLE_COUNT
        )
        wheel_messages = self._require_samples(WHEEL_TOPIC, TARGET_SAMPLE_COUNT)
        self.assertTrue(all(message.header.frame_id == "odom" for message in wheel_messages))
        self.assertTrue(all(message.child_frame_id == "base_link" for message in wheel_messages))

        wheel_stamps = [message.header.stamp.to_nsec() for message in wheel_messages]
        self.assertEqual(len(wheel_stamps), len(set(wheel_stamps)))
        stamp_sequences = {
            topic: [
                message.header.stamp.to_nsec()
                for message in self._require_samples(topic, TARGET_SAMPLE_COUNT)
            ]
            for topic in STAMPED_TOPICS
        }
        stamp_sequences["/clock"] = [
            message.clock.to_nsec() for message in self._clock_messages
        ]
        stamp_sequences[WHEEL_TOPIC] = wheel_stamps
        window = _fixed_contiguous_window(
            stamp_sequences,
            50000000,
            EXACT_WINDOW_SAMPLE_COUNT,
        )

        speed_by_stamp = _by_stamp(speed_messages)
        wheel_by_stamp = _by_stamp(wheel_messages)
        self.assertEqual(
            [stamp for stamp in window if stamp not in speed_by_stamp],
            [],
        )
        self.assertEqual(
            [stamp for stamp in window if stamp not in wheel_by_stamp],
            [],
        )
        receipt_by_stamp = {
            message.header.stamp.to_nsec(): receipt
            for message, receipt in zip(
                wheel_messages, self._receipts[WHEEL_TOPIC]
            )
        }
        window_receipts = [receipt_by_stamp[stamp] for stamp in window]
        window_rate_hz = _receipt_rate(window_receipts)
        self.assertGreaterEqual(window_rate_hz, 19.0)
        self.assertLessEqual(window_rate_hz, 21.0)

        first_message = wheel_by_stamp[window[0]]
        next_cycle_message = wheel_by_stamp[window[-1]]
        self.assertAlmostEqual(
            first_message.pose.pose.position.x,
            next_cycle_message.pose.pose.position.x,
            places=6,
        )
        self.assertAlmostEqual(
            first_message.pose.pose.position.y,
            next_cycle_message.pose.pose.position.y,
            places=6,
        )
        self.assertAlmostEqual(
            first_message.pose.pose.orientation.z,
            next_cycle_message.pose.pose.orientation.z,
            places=6,
        )

    def test_static_sensor_transforms_include_gps_and_camera_frames(self):
        self.assertTrue(self._tf_static_messages, "missing /tf_static sensor transforms")
        transforms = [
            transform
            for message in self._tf_static_messages
            for transform in message.transforms
        ]
        by_child = {transform.child_frame_id: transform for transform in transforms}
        expected_children = {"lidar_link", "imu_link", "gps_link", "camera_link"}
        self.assertTrue(expected_children.issubset(by_child))
        for child in expected_children:
            transform = by_child[child]
            self.assertEqual(transform.header.frame_id, "base_link")
            self.assertFalse(transform.header.frame_id.startswith("/"))
            self.assertFalse(transform.child_frame_id.startswith("/"))

    def test_synthetic_process_has_no_hardware_device_open(self):
        master = rosgraph.Master(rospy.get_name())
        uri = rosnode.get_api_uri(master, "/synthetic_sensor", skip_cache=True)
        self.assertIsNotNone(uri, "missing /synthetic_sensor node")
        code, message, pid = xmlrpc.client.ServerProxy(uri).getPid(rospy.get_name())
        self.assertEqual((code, message), (1, ""))

        command_line = _read_bytes("/proc/{}/cmdline".format(pid)).replace(b"\0", b" ")
        self.assertNotIn(b"/dev/", command_line)
        opened_hardware = []
        fd_root = "/proc/{}/fd".format(pid)
        for entry in os.listdir(fd_root):
            try:
                target = os.readlink(os.path.join(fd_root, entry))
            except FileNotFoundError:
                continue
            if (
                    target.startswith("/dev/")
                    and target != "/dev/null"
                    and not target.startswith("/dev/pts/")):
                opened_hardware.append(target)
        self.assertEqual(opened_hardware, [])

    def test_demo_map_geometry_matches_scan_wall_faces_and_map_server(self):
        yaml_path = os.path.join(PACKAGE_ROOT, "maps", "demo", "base.yaml")
        with open(yaml_path, "r", encoding="utf-8") as stream:
            metadata = yaml.safe_load(stream)
        self.assertEqual(metadata["image"], "base.pgm")
        self.assertEqual(metadata["resolution"], 0.25)
        self.assertEqual(metadata["origin"], [-3.25, -2.25, 0.0])
        self.assertEqual(metadata["negate"], 0)

        width, height, maximum, pixels = _read_ascii_pgm(
            os.path.join(os.path.dirname(yaml_path), metadata["image"])
        )
        self.assertEqual((width, height, maximum), (26, 18, 255))
        self.assertEqual(len(pixels), width * height)
        self.assertTrue(all(value == 0 for value in pixels[:width]))
        self.assertTrue(all(value == 0 for value in pixels[-width:]))
        self.assertTrue(all(pixels[row * width] == 0 for row in range(height)))
        self.assertTrue(all(pixels[row * width + width - 1] == 0 for row in range(height)))
        self.assertGreater(max(pixels), 250)

        resolution = metadata["resolution"]
        origin_x, origin_y, _ = metadata["origin"]
        self.assertEqual(origin_x + resolution * 0.5, -STATIC_WALL_X_M)
        self.assertEqual(origin_x + (width - 0.5) * resolution, STATIC_WALL_X_M)
        self.assertEqual(origin_y + resolution * 0.5, -STATIC_WALL_Y_M)
        self.assertEqual(origin_y + (height - 0.5) * resolution, STATIC_WALL_Y_M)

        self.assertTrue(self._map_messages, "map_server did not load the demo map")
        served = self._map_messages[-1]
        self.assertEqual(served.header.frame_id, "map")
        self.assertEqual((served.info.width, served.info.height), (26, 18))
        self.assertEqual(served.info.resolution, 0.25)
        self.assertEqual(served.info.origin.position.x, -3.25)
        self.assertEqual(served.info.origin.position.y, -2.25)
        self.assertEqual(len(served.data), 26 * 18)


def _receipt_rate(receipts):
    if len(receipts) < 2 or receipts[-1] <= receipts[0]:
        return 0.0
    return (len(receipts) - 1) / (receipts[-1] - receipts[0])


def _fixed_contiguous_window(stamp_sequences, step_nsec, sample_count):
    """Return one head-aligned window and reject any duplicate or inner gap."""
    if not stamp_sequences or step_nsec <= 0 or sample_count <= 0:
        raise ValueError("window inputs must be non-empty and positive")
    if any(not stamps for stamps in stamp_sequences.values()):
        raise AssertionError("every stream must contain at least one stamp")

    start_stamp = max(min(stamps) for stamps in stamp_sequences.values())
    expected = tuple(
        start_stamp + index * step_nsec for index in range(sample_count)
    )
    for label, stamps in stamp_sequences.items():
        counts = Counter(stamps)
        missing = [stamp for stamp in expected if counts[stamp] == 0]
        duplicates = [stamp for stamp in expected if counts[stamp] > 1]
        if missing or duplicates:
            raise AssertionError(
                "{} has missing={} duplicate={} in fixed window".format(
                    label, missing, duplicates
                )
            )
    return expected


def _sample_index(stamp):
    return int(round((stamp.to_sec() - SEQUENCE_START_SEC) * PUBLISH_RATE_HZ))


def _expected_speed(sample_index):
    phase = sample_index % CYCLE_SAMPLES
    if phase in (0, CYCLE_SAMPLES // 2):
        return 0.0
    return 0.5 if phase < CYCLE_SAMPLES // 2 else -0.5


def _expected_pose_x(sample_index):
    phase = sample_index % CYCLE_SAMPLES
    step_distance = 0.5 * PERIOD_SEC
    if phase < CYCLE_SAMPLES // 2:
        return phase * step_distance
    return (CYCLE_SAMPLES - 1 - phase) * step_distance


def _expected_acceleration_x(sample_index):
    return (
        _expected_speed(sample_index) - _expected_speed(sample_index - 1)
    ) * PUBLISH_RATE_HZ


def _by_stamp(messages):
    return {message.header.stamp.to_nsec(): message for message in messages}


def _read_bytes(path):
    with open(path, "rb") as stream:
        return stream.read()


def _read_ascii_pgm(path):
    with open(path, "r", encoding="ascii") as stream:
        tokens = []
        for line in stream:
            content = line.split("#", 1)[0]
            tokens.extend(content.split())
    if not tokens or tokens[0] != "P2":
        raise AssertionError("demo map must be an ASCII P2 PGM")
    width, height, maximum = map(int, tokens[1:4])
    return width, height, maximum, [int(value) for value in tokens[4:]]


if __name__ == "__main__":
    import rostest

    rostest.rosrun(
        "stier_slam_test_support",
        "synthetic_topics",
        SyntheticTopicsTest,
    )
