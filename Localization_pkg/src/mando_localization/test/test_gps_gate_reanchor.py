#!/usr/bin/env python3
"""GPS gate가 실제 anchor 적용 뒤에만 transaction ack를 내는지 확인한다."""

import copy
import threading
import time
import unittest

import rospy
import rostest
from geometry_msgs.msg import PoseWithCovarianceStamped
from mando_localization.msg import GpsGateReanchor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, NavSatStatus


GPS_FIX = "/molit/sensors/gps/fix"
LOCAL_ODOMETRY = "/molit/localization/local/odometry"
GPS_CANDIDATE = "/mando_localization/internal/gps/candidate_pose"
GPS_GATE_POSE = "/mando_localization/internal/gps/gate_pose"
GPS_REANCHOR = "/mando_localization/internal/gps/reanchor_pose"
GPS_REANCHOR_ACCEPTED = "/mando_localization/internal/gps/reanchor_accepted"


class GpsGateReanchorTest(unittest.TestCase):
    def setUp(self):
        self.candidates = []
        self.gate_poses = []
        self.acks = []
        self.candidate_event = threading.Event()
        self.gate_pose_event = threading.Event()
        self.ack_event = threading.Event()

        self.candidate_subscriber = rospy.Subscriber(
            GPS_CANDIDATE, PoseWithCovarianceStamped, self._candidate_callback
        )
        self.gate_pose_subscriber = rospy.Subscriber(
            GPS_GATE_POSE, PoseWithCovarianceStamped, self._gate_pose_callback
        )
        self.ack_subscriber = rospy.Subscriber(
            GPS_REANCHOR_ACCEPTED, GpsGateReanchor, self._ack_callback
        )
        self.fix_publisher = rospy.Publisher(GPS_FIX, NavSatFix, queue_size=10)
        self.local_publisher = rospy.Publisher(
            LOCAL_ODOMETRY, Odometry, queue_size=10
        )
        self.reanchor_publisher = rospy.Publisher(
            GPS_REANCHOR, GpsGateReanchor, queue_size=10
        )

    def _candidate_callback(self, message):
        self.candidates.append(message)
        self.candidate_event.set()

    def _gate_pose_callback(self, message):
        self.gate_poses.append(message)
        self.gate_pose_event.set()

    def _ack_callback(self, message):
        self.acks.append(message)
        self.ack_event.set()

    def _wait_for_connections(self, timeout_sec=5.0):
        publishers = [
            self.fix_publisher,
            self.local_publisher,
            self.reanchor_publisher,
        ]
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline and not rospy.is_shutdown():
            if all(publisher.get_num_connections() > 0 for publisher in publishers):
                return
            rospy.sleep(0.02)
        self.fail("GPS gate 테스트 publisher 연결이 완료되지 않았습니다.")

    def _publish_local(self, x):
        message = Odometry()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = x
        message.pose.pose.orientation.w = 1.0
        for index in (0, 7, 14, 21, 28, 35):
            message.pose.covariance[index] = 0.25
        self.local_publisher.publish(message)
        rospy.sleep(0.02)

    def _publish_fix(self):
        message = NavSatFix()
        message.header.stamp = rospy.Time.now()
        message.header.frame_id = "gps_link"
        message.status.status = NavSatStatus.STATUS_FIX
        message.status.service = NavSatStatus.SERVICE_GPS
        message.latitude = 37.0
        message.longitude = 127.0
        message.altitude = 10.0
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
        message.position_covariance[0] = 0.25
        message.position_covariance[4] = 0.25
        message.position_covariance[8] = 1.0
        self.fix_publisher.publish(message)
        rospy.sleep(0.06)

    def test_reanchor_ack_means_anchor_was_applied(self):
        self._wait_for_connections()

        # 최초 3회 정상 fix로 gate의 map=0/local=0 anchor를 만든다.
        for _ in range(3):
            self._publish_local(0.0)
            self._publish_fix()
        self.assertTrue(self.gate_pose_event.wait(2.0))
        self.assertAlmostEqual(0.0, self.gate_poses[-1].pose.pose.position.x)

        # Local이 20 m drift한 상태에서는 같은 GPS를 기존 innovation gate가 거부한다.
        self.gate_pose_event.clear()
        self.candidate_event.clear()
        self._publish_local(20.0)
        self._publish_fix()
        self.assertTrue(self.candidate_event.wait(1.0))
        self.assertFalse(self.gate_pose_event.wait(0.15))
        candidate = self.candidates[-1]

        # 최신 GPS 후보에서 2 m 넘게 벗어난 command는 적용/ack되지 않는다.
        bad_command = GpsGateReanchor()
        bad_command.transaction_id = 41
        bad_command.pose = copy.deepcopy(candidate)
        bad_command.pose.pose.pose.position.x += 3.0
        self._publish_local(20.0)
        self.reanchor_publisher.publish(bad_command)
        self.assertFalse(self.ack_event.wait(0.12))

        # 정확한 transaction pose는 적용 후 그대로 ack된다.
        command = GpsGateReanchor()
        command.transaction_id = 42
        command.pose = copy.deepcopy(candidate)
        self._publish_local(20.0)
        self.reanchor_publisher.publish(command)
        self.assertTrue(self.ack_event.wait(1.0))
        self.assertEqual(42, self.acks[-1].transaction_id)
        self.assertEqual(command.pose.header.stamp, self.acks[-1].pose.header.stamp)

        # 새 anchor(map=0/local=20)가 실제 적용되어 다음 fix가 즉시 통과한다.
        self.gate_pose_event.clear()
        self._publish_local(20.0)
        self._publish_fix()
        self.assertTrue(self.gate_pose_event.wait(1.0))
        self.assertAlmostEqual(0.0, self.gate_poses[-1].pose.pose.position.x)


if __name__ == "__main__":
    rospy.init_node("test_gps_gate_reanchor")
    rostest.rosrun(
        "mando_localization", "test_gps_gate_reanchor", GpsGateReanchorTest
    )
