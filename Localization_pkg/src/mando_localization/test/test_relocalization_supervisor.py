#!/usr/bin/env python3
"""장기 GPS 단절 후보가 LiDAR와 GPS gate 확인 뒤에만 재승인되는지 확인한다."""

import copy
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseWithCovarianceStamped
from mando_localization.msg import GpsGateReanchor
from std_msgs.msg import Bool, String


GPS_GATE_POSE = "/mando_localization/internal/gps/gate_pose"
GPS_CANDIDATE = "/mando_localization/internal/gps/candidate_pose"
GPS_REANCHOR = "/mando_localization/internal/gps/reanchor_pose"
GPS_REANCHOR_ACCEPTED = "/mando_localization/internal/gps/reanchor_accepted"
LIDAR_POSE = "/molit/localization/lidar/map_pose"
GPS_MAP_POSE = "/molit/localization/gps/map_pose"
RECOVERY_ACTIVE = "/mando_localization/internal/recovery/active"
RECOVERY_STATE = "/molit/localization/recovery/state"


class RelocalizationSupervisorTest(unittest.TestCase):
    def setUp(self):
        self.public_poses = []
        self.reanchors = []
        self.recovery_active = []
        self.recovery_states = []
        self.public_pose_event = threading.Event()
        self.reanchor_event = threading.Event()

        self.public_pose_subscriber = rospy.Subscriber(
            GPS_MAP_POSE, PoseWithCovarianceStamped, self._public_pose_callback
        )
        self.reanchor_subscriber = rospy.Subscriber(
            GPS_REANCHOR, GpsGateReanchor, self._reanchor_callback
        )
        self.active_subscriber = rospy.Subscriber(
            RECOVERY_ACTIVE, Bool, lambda message: self.recovery_active.append(message.data)
        )
        self.state_subscriber = rospy.Subscriber(
            RECOVERY_STATE, String, lambda message: self.recovery_states.append(message.data)
        )

        self.gate_pose_publisher = rospy.Publisher(
            GPS_GATE_POSE, PoseWithCovarianceStamped, queue_size=10
        )
        self.candidate_publisher = rospy.Publisher(
            GPS_CANDIDATE, PoseWithCovarianceStamped, queue_size=10
        )
        self.lidar_publisher = rospy.Publisher(
            LIDAR_POSE, PoseWithCovarianceStamped, queue_size=10
        )
        self.reanchor_accepted_publisher = rospy.Publisher(
            GPS_REANCHOR_ACCEPTED, GpsGateReanchor, queue_size=10
        )

    def _public_pose_callback(self, message):
        self.public_poses.append(message)
        self.public_pose_event.set()

    def _reanchor_callback(self, message):
        self.reanchors.append(message)
        self.reanchor_event.set()

    def _wait_for_connections(self, timeout_sec=5.0):
        publishers = [
            self.gate_pose_publisher,
            self.candidate_publisher,
            self.lidar_publisher,
            self.reanchor_accepted_publisher,
        ]
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if all(publisher.get_num_connections() > 0 for publisher in publishers):
                return
            rospy.sleep(0.02)
        self.fail("Supervisor 테스트 publisher 연결이 완료되지 않았습니다.")

    @staticmethod
    def _pose(x, y):
        message = PoseWithCovarianceStamped()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "map"
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.w = 1.0
        message.pose.covariance[0] = 0.25
        message.pose.covariance[7] = 0.25
        message.pose.covariance[14] = 1.0
        message.pose.covariance[21] = 1.0
        message.pose.covariance[28] = 1.0
        message.pose.covariance[35] = 10.0
        return message

    def _publish_pair(self, gps_x, lidar_x, stamp=None):
        lidar = self._pose(lidar_x, 0.0)
        candidate = self._pose(gps_x, 0.0)
        # 동일 센서 epoch를 표현해 timestamp skew gate도 함께 검증한다.
        pair_stamp = stamp if stamp is not None else lidar.header.stamp
        lidar.header.stamp = pair_stamp
        candidate.header.stamp = pair_stamp
        self.lidar_publisher.publish(lidar)
        # 서로 다른 TCPROS 연결 사이에는 callback 도착 순서가 보장되지 않는다.
        # 이 테스트는 방금 발행한 LiDAR를 GPS 후보와 비교하는 시나리오이므로
        # LiDAR callback이 처리될 시간을 준 뒤 후보를 발행한다.
        rospy.sleep(0.05)
        self.candidate_publisher.publish(candidate)
        rospy.sleep(0.08)

    def test_lidar_assisted_long_outage_recovery(self):
        self._wait_for_connections()

        self.gate_pose_publisher.publish(self._pose(0.0, 0.0))
        self.assertTrue(self.public_pose_event.wait(2.0), "초기 gate pose가 relay되지 않았습니다.")
        initial_public_count = len(self.public_poses)
        self.public_pose_event.clear()

        rospy.sleep(2.15)

        # 안정적인 GPS 후보라도 LiDAR와 5 m 넘게 다르면 reanchor할 수 없다.
        for _ in range(3):
            self._publish_pair(20.0, 0.0)
        self.assertFalse(self.reanchor_event.wait(0.3))
        self.assertIn("LIDAR_MISMATCH", self.recovery_states)
        self.assertIn(True, self.recovery_active)

        # 두 번 일치한 뒤 한 번이라도 LiDAR가 불일치하면 카운트를 버린다.
        for _ in range(2):
            self._publish_pair(1.0, 1.2)
        self.assertFalse(self.reanchor_event.is_set())
        self._publish_pair(1.0, 7.0)
        self.assertFalse(self.reanchor_event.is_set())

        # 같은 timestamp를 반복 발행해도 서로 다른 3개 후보로 세지 않는다.
        duplicate_stamp = rospy.Time.now()
        for _ in range(3):
            self._publish_pair(1.0, 1.2, duplicate_stamp)
        self.assertFalse(self.reanchor_event.is_set())

        # 서로 다른 epoch의 GPS와 LiDAR가 3회 연속 일치해야 command를 낸다.
        for _ in range(2):
            self._publish_pair(1.0, 1.2)
        self.assertFalse(self.reanchor_event.is_set())
        self._publish_pair(1.0, 1.2)
        self.assertTrue(self.reanchor_event.wait(2.0), "LiDAR 보조 reanchor가 발행되지 않았습니다.")
        self.assertFalse(self.public_pose_event.wait(0.12))
        self.assertEqual(initial_public_count, len(self.public_poses))
        self.assertEqual("WAITING_FOR_GPS_GATE_REANCHOR", self.recovery_states[-1])
        self.assertTrue(self.recovery_active[-1])
        self.assertGreater(self.reanchors[-1].transaction_id, 0)
        self.assertAlmostEqual(1.0, self.reanchors[-1].pose.pose.pose.position.x)

        # ack 대기 중 후보나 gate pose가 들어와도 공개 승인 경로를 우회할 수 없다.
        reanchor_count = len(self.reanchors)
        self._publish_pair(1.0, 1.2)
        self.gate_pose_publisher.publish(self._pose(9.0, 0.0))
        rospy.sleep(0.05)
        self.assertEqual(reanchor_count, len(self.reanchors))
        self.assertEqual(initial_public_count, len(self.public_poses))

        # transaction, stamp 또는 XY가 다른 ack는 모두 무시한다.
        wrong_stamp_ack = copy.deepcopy(self.reanchors[-1])
        wrong_stamp_ack.pose.header.stamp += rospy.Duration.from_sec(0.01)
        self.reanchor_accepted_publisher.publish(wrong_stamp_ack)
        rospy.sleep(0.05)
        wrong_xy_ack = copy.deepcopy(self.reanchors[-1])
        wrong_xy_ack.pose.pose.pose.position.x += 0.1
        self.reanchor_accepted_publisher.publish(wrong_xy_ack)
        rospy.sleep(0.05)
        wrong_transaction_ack = copy.deepcopy(self.reanchors[-1])
        wrong_transaction_ack.transaction_id += 1
        self.reanchor_accepted_publisher.publish(wrong_transaction_ack)
        rospy.sleep(0.05)
        self.assertEqual(initial_public_count, len(self.public_poses))
        self.assertNotEqual("RECOVERED_WITH_LIDAR", self.recovery_states[-1])

        # GPS gate가 실제 적용한 command를 그대로 ack한 뒤에만 공개한다.
        matching_ack = copy.deepcopy(self.reanchors[-1])
        self.reanchor_accepted_publisher.publish(matching_ack)
        self.assertTrue(self.public_pose_event.wait(2.0), "복구 GPS pose가 공개되지 않았습니다.")
        self.assertAlmostEqual(1.0, self.public_poses[-1].pose.pose.position.x)
        self.assertEqual(initial_public_count + 1, len(self.public_poses))

        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if (
                self.recovery_states
                and self.recovery_states[-1] == "RECOVERED_WITH_LIDAR"
                and self.recovery_active
                and not self.recovery_active[-1]
            ):
                break
            rospy.sleep(0.02)
        self.assertEqual("RECOVERED_WITH_LIDAR", self.recovery_states[-1])
        self.assertFalse(self.recovery_active[-1])

        # 동일 ack 재전송은 공개 pose를 중복 발행하지 않는다.
        recovered_public_count = len(self.public_poses)
        self.reanchor_accepted_publisher.publish(matching_ack)
        rospy.sleep(0.1)
        self.assertEqual(recovered_public_count, len(self.public_poses))

        # ack가 유실되면 timeout 후에도 fail-closed이며 늦은 ack를 무시한다.
        rospy.sleep(2.15)
        self.reanchor_event.clear()
        for _ in range(3):
            self._publish_pair(2.0, 2.1)
        self.assertTrue(self.reanchor_event.wait(2.0))
        late_ack = copy.deepcopy(self.reanchors[-1])
        rospy.sleep(1.1)
        self.assertEqual("GPS_GATE_REANCHOR_TIMEOUT", self.recovery_states[-1])
        self.assertTrue(self.recovery_active[-1])
        self.assertEqual(recovered_public_count, len(self.public_poses))
        self.reanchor_accepted_publisher.publish(late_ack)
        rospy.sleep(0.1)
        self.assertEqual(recovered_public_count, len(self.public_poses))


if __name__ == "__main__":
    rospy.init_node("test_relocalization_supervisor")
    rostest.rosrun(
        "mando_localization",
        "test_relocalization_supervisor",
        RelocalizationSupervisorTest,
    )
