#!/usr/bin/env python3
"""합성 지도와 측정값으로 LiDAR localization 공개 경계를 검증한다."""

import math
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


MAP_TOPIC = "/map"
INTERNAL_MAP_TOPIC = "/mando_localization/internal/amcl/map"
SCAN_INPUT_TOPIC = "/molit/sensors/lidar/scan"
SCAN_RELAY_TOPIC = "/mando_localization/internal/amcl/scan"
GPS_POSE_TOPIC = "/molit/localization/gps/map_pose"
AMCL_POSE_TOPIC = "/mando_localization/internal/amcl/pose"
INITIALPOSE_TOPIC = "/mando_localization/internal/amcl/initialpose"
PUBLIC_INITIALPOSE_TOPIC = "/initialpose"
LIDAR_POSE_TOPIC = "/molit/localization/lidar/map_pose"
INTERNAL_NAVPVT_TOPIC = "/mando_localization/internal/driver/gps_navpvt"
PUBLIC_NAVPVT_TOPIC = "/molit/sensors/gps/navpvt"


class SyntheticLidarLocalizationTest(unittest.TestCase):
    def setUp(self):
        self.map_event = threading.Event()
        self.scan_event = threading.Event()
        self.initialpose_event = threading.Event()
        self.lidar_pose_event = threading.Event()
        self.navpvt_event = threading.Event()

        self.maps = []
        self.relayed_scans = []
        self.initialposes = []
        self.lidar_poses = []
        self.navpvt_messages = []

        self.map_subscriber = rospy.Subscriber(
            MAP_TOPIC, OccupancyGrid, self._map_callback, queue_size=1
        )
        self.scan_subscriber = rospy.Subscriber(
            SCAN_RELAY_TOPIC, LaserScan, self._scan_callback, queue_size=10
        )
        self.initialpose_subscriber = rospy.Subscriber(
            INITIALPOSE_TOPIC,
            PoseWithCovarianceStamped,
            self._initialpose_callback,
            queue_size=10,
        )
        self.lidar_pose_subscriber = rospy.Subscriber(
            LIDAR_POSE_TOPIC,
            PoseWithCovarianceStamped,
            self._lidar_pose_callback,
            queue_size=10,
        )
        self.navpvt_subscriber = rospy.Subscriber(
            PUBLIC_NAVPVT_TOPIC, String, self._navpvt_callback, queue_size=10
        )

        self.map_publisher = rospy.Publisher(
            INTERNAL_MAP_TOPIC, OccupancyGrid, queue_size=1, latch=True
        )
        self.scan_publisher = rospy.Publisher(
            SCAN_INPUT_TOPIC, LaserScan, queue_size=10
        )
        self.gps_pose_publisher = rospy.Publisher(
            GPS_POSE_TOPIC, PoseWithCovarianceStamped, queue_size=10
        )
        self.amcl_pose_publisher = rospy.Publisher(
            AMCL_POSE_TOPIC, PoseWithCovarianceStamped, queue_size=10
        )
        self.manual_initialpose_publisher = rospy.Publisher(
            PUBLIC_INITIALPOSE_TOPIC,
            PoseWithCovarianceStamped,
            queue_size=10,
        )
        self.navpvt_publisher = rospy.Publisher(
            INTERNAL_NAVPVT_TOPIC, String, queue_size=10
        )

    def _map_callback(self, message):
        self.maps.append(message)
        self.map_event.set()

    def _scan_callback(self, message):
        self.relayed_scans.append(message)
        self.scan_event.set()

    def _initialpose_callback(self, message):
        self.initialposes.append(message)
        self.initialpose_event.set()

    def _lidar_pose_callback(self, message):
        self.lidar_poses.append(message)
        self.lidar_pose_event.set()

    def _navpvt_callback(self, message):
        self.navpvt_messages.append(message)
        self.navpvt_event.set()

    def _wait_for_connections(self, publishers, timeout_sec=5.0):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if all(publisher.get_num_connections() > 0 for publisher in publishers):
                return
            rospy.sleep(0.02)
        self.fail("테스트 publisher가 제한 시간 안에 subscriber와 연결되지 않았습니다.")

    @staticmethod
    def _synthetic_map():
        message = OccupancyGrid()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "map"
        message.info.resolution = 0.5
        message.info.width = 12
        message.info.height = 12
        message.info.origin.position.x = -3.0
        message.info.origin.position.y = -3.0
        message.info.origin.orientation.w = 1.0
        message.data = []
        for row in range(message.info.height):
            for column in range(message.info.width):
                is_wall = (
                    row == 0
                    or column == 0
                    or row == message.info.height - 1
                    or column == message.info.width - 1
                )
                message.data.append(100 if is_wall else 0)
        return message

    @staticmethod
    def _synthetic_scan():
        message = LaserScan()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "laser_link"
        message.angle_min = -math.pi / 2.0
        message.angle_increment = math.pi / 59.0
        message.angle_max = message.angle_min + 59.0 * message.angle_increment
        message.range_min = 0.05
        message.range_max = 20.0
        message.ranges = [3.0] * 60
        return message

    @staticmethod
    def _map_pose(x, y, yaw, xy_variance=0.25, yaw_variance=0.1):
        message = PoseWithCovarianceStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "map"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(0.5 * yaw)
        message.pose.pose.orientation.w = math.cos(0.5 * yaw)
        message.pose.covariance[0] = xy_variance
        message.pose.covariance[7] = xy_variance
        message.pose.covariance[14] = 0.1
        message.pose.covariance[21] = 0.1
        message.pose.covariance[28] = 0.1
        message.pose.covariance[35] = yaw_variance
        return message

    def test_synthetic_map_initialization_scan_and_pose_gate(self):
        self._wait_for_connections(
            [
                self.map_publisher,
                self.scan_publisher,
                self.gps_pose_publisher,
                self.amcl_pose_publisher,
                self.manual_initialpose_publisher,
                self.navpvt_publisher,
            ]
        )

        synthetic_map = self._synthetic_map()
        self.map_publisher.publish(synthetic_map)
        self.assertTrue(self.map_event.wait(2.0), "합성 OccupancyGrid가 발행되지 않았습니다.")
        received_map = self.maps[-1]
        self.assertEqual("map", received_map.header.frame_id)
        self.assertEqual(
            received_map.info.width * received_map.info.height,
            len(received_map.data),
        )
        self.assertIn(0, received_map.data)
        self.assertIn(100, received_map.data)

        # 실제 타입 의존성 없이 NavPVT 계열 메시지를 공개 YAML 토픽으로 relay한다.
        for _ in range(10):
            self.navpvt_publisher.publish(String(data="synthetic-navpvt"))
            if self.navpvt_event.wait(0.1):
                break
        self.assertTrue(self.navpvt_event.is_set(), "NavPVT generic relay가 동작하지 않았습니다.")
        self.assertEqual("synthetic-navpvt", self.navpvt_messages[-1].data)

        scan = self._synthetic_scan()
        self.scan_publisher.publish(scan)
        self.assertTrue(self.scan_event.wait(2.0), "검증된 scan이 relay되지 않았습니다.")
        relayed_scan = self.relayed_scans[-1]
        self.assertEqual("laser_link", relayed_scan.header.frame_id)
        self.assertEqual(scan.header.stamp, relayed_scan.header.stamp)
        self.assertEqual(60, len(relayed_scan.ranges))

        gps_pose = self._map_pose(2.0, -1.0, 0.7, xy_variance=1.0)
        self.gps_pose_publisher.publish(gps_pose)
        self.assertTrue(
            self.initialpose_event.wait(2.0),
            "검증된 GPS pose가 AMCL 내부 initialpose로 전달되지 않았습니다.",
        )
        initialpose = self.initialposes[-1]
        self.assertAlmostEqual(2.0, initialpose.pose.pose.position.x)
        self.assertAlmostEqual(-1.0, initialpose.pose.pose.position.y)
        # GPS 단독 heading 대신 YAML에 명시된 초기 yaw와 분산을 사용한다.
        lidar_node_namespace = "/odometry_lidar_fusion_rostest"
        configured_yaw = rospy.get_param(
            lidar_node_namespace + "/initialization/gps_initial_yaw_rad"
        )
        configured_yaw_variance = rospy.get_param(
            lidar_node_namespace
            + "/initialization/gps_initial_yaw_variance_rad2"
        )
        self.assertAlmostEqual(
            math.sin(0.5 * configured_yaw),
            initialpose.pose.pose.orientation.z,
            places=6,
        )
        self.assertAlmostEqual(
            math.cos(0.5 * configured_yaw),
            initialpose.pose.pose.orientation.w,
            places=6,
        )
        self.assertAlmostEqual(
            configured_yaw_variance,
            initialpose.pose.covariance[35],
            places=6,
        )

        # GPS 자동 초기화는 한 번만 발행해야 한다.
        self.initialpose_event.clear()
        second_gps_pose = self._map_pose(2.2, -1.0, 0.7, xy_variance=1.0)
        self.gps_pose_publisher.publish(second_gps_pose)
        self.assertFalse(self.initialpose_event.wait(0.3))
        self.assertEqual(1, len(self.initialposes))

        # RViz의 공개 /initialpose는 어댑터를 거쳐 AMCL 내부 토픽으로 전달된다.
        manual_initialpose = self._map_pose(-0.5, 0.75, -0.2)
        self.manual_initialpose_publisher.publish(manual_initialpose)
        self.assertTrue(
            self.initialpose_event.wait(2.0),
            "공개 initialpose가 AMCL 내부 토픽으로 relay되지 않았습니다.",
        )
        self.assertEqual(2, len(self.initialposes))
        self.assertAlmostEqual(-0.5, self.initialposes[-1].pose.pose.position.x)

        # 최초 AMCL pose는 production 정책상 정상 3회가 누적되기 전까지 공개되지 않는다.
        for index in range(2):
            candidate = self._map_pose(1.0 + 0.05 * index, 1.5, 0.1)
            self.amcl_pose_publisher.publish(candidate)
            rospy.sleep(0.08)
        self.assertEqual([], self.lidar_poses)

        accepted_candidate = self._map_pose(1.1, 1.5, 0.1)
        self.amcl_pose_publisher.publish(accepted_candidate)
        self.assertTrue(
            self.lidar_pose_event.wait(2.0),
            "세 번째 정상 AMCL pose가 공개 LiDAR pose로 승인되지 않았습니다.",
        )
        approved_pose = self.lidar_poses[-1]
        self.assertEqual("map", approved_pose.header.frame_id)
        self.assertAlmostEqual(1.1, approved_pose.pose.pose.position.x)
        self.assertAlmostEqual(1.5, approved_pose.pose.pose.position.y)
        quaternion_norm = math.sqrt(
            approved_pose.pose.pose.orientation.z ** 2
            + approved_pose.pose.pose.orientation.w ** 2
        )
        self.assertAlmostEqual(1.0, quaternion_norm, places=6)


if __name__ == "__main__":
    rospy.init_node("synthetic_lidar_localization_rostest")
    rostest.rosrun(
        "mando_localization",
        "synthetic_occupancygrid_lidar_contract",
        SyntheticLidarLocalizationTest,
    )
