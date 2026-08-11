#!/usr/bin/env python3
"""표준 JointState wheel encoder를 차량 속도 계약으로 변환한다."""

import math

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import JointState

from stier_slam_core.wheel_encoder import WheelEncoderEstimator


class WheelEncoderAdapterNode:
    """역할:
    좌·우 wheel encoder JointState를 기존 Ackermann 입력 차량속도로 변환한다.

    동작 방식:
    source timestamp와 joint 배열을 검증해 ``WheelEncoderEstimator``로 m/s를 계산하고,
    설정된 frame과 covariance를 담아 ``TwistWithCovarianceStamped``로 발행한다.
    """

    def __init__(self):
        if (rospy.get_param("~real_mode", False)
                and rospy.get_param("~calibration_required", True)):
            raise ValueError("wheel encoder calibration is required in real_mode")
        encoder_topic = rospy.get_param("/stier_slam/topics/inputs/wheel_encoder")
        vehicle_speed_topic = rospy.get_param("/stier_slam/topics/inputs/vehicle_speed")
        self._output_frame = rospy.get_param("~output_frame")
        if (not isinstance(self._output_frame, str) or not self._output_frame
                or self._output_frame.startswith("/")):
            raise ValueError("~output_frame must be a non-empty relative frame")
        self._twist_covariance = self._load_covariance()
        self._estimator = WheelEncoderEstimator(
            rospy.get_param("~wheel_radius_m"),
            rospy.get_param("~left_joint_name"),
            rospy.get_param("~right_joint_name"),
            rospy.get_param("~left_sign"),
            rospy.get_param("~right_sign"),
            rospy.get_param("~max_abs_wheel_speed_rad_s"),
        )
        self._publisher = rospy.Publisher(
            vehicle_speed_topic, TwistWithCovarianceStamped, queue_size=10
        )
        rospy.Subscriber(
            encoder_topic, JointState, self._on_encoder, queue_size=20
        )

    def _on_encoder(self, message):
        try:
            stamp = message.header.stamp.to_sec()
            speed_mps = self._estimator.update(
                stamp, message.name, message.position, message.velocity
            )
        except (AttributeError, TypeError, ValueError) as error:
            rospy.logwarn("rejecting wheel encoder sample: %s", error)
            return
        if speed_mps is None:
            return
        output = TwistWithCovarianceStamped()
        output.header.stamp = message.header.stamp
        output.header.frame_id = self._output_frame
        output.twist.twist.linear.x = speed_mps
        output.twist.covariance = list(self._twist_covariance)
        self._publisher.publish(output)

    @staticmethod
    def _load_covariance():
        covariance = rospy.get_param("~twist_covariance")
        try:
            covariance = tuple(covariance)
        except TypeError as error:
            raise ValueError("~twist_covariance must be iterable") from error
        if (len(covariance) != 36
                or not all(not isinstance(value, bool) and math.isfinite(value)
                           for value in covariance)):
            raise ValueError("~twist_covariance must contain 36 finite values")
        return covariance


if __name__ == "__main__":
    rospy.init_node("wheel_encoder_adapter")
    WheelEncoderAdapterNode()
    rospy.spin()
