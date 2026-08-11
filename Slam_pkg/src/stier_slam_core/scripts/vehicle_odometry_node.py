#!/usr/bin/env python3
"""timestamp가 있는 Ackermann control로 wheel-only Odometry를 발행한다."""

import math
import threading

import rospy
from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from stier_slam_core.ackermann import AckermannIntegrator, NonMonotonicTimeError


class VehicleOdometryNode:
    """역할:
    차량 속도와 조향각으로 wheel-only ``nav_msgs/Odometry``를 생성한다.

    동작 방식:
    두 입력을 receipt time 기준으로 짝지어 ``AckermannIntegrator``에 전달하고, 계산된
    planar pose와 covariance를 `/slam/odometry/wheel`에 source timestamp로 발행한다.
    """

    def __init__(self):
        if rospy.get_param("~real_mode", False) and rospy.get_param(
                "~calibration_required", True):
            rospy.logfatal("wheel odometry calibration is required in real_mode")
            rospy.signal_shutdown("wheel odometry calibration required")
            return

        vehicle_speed_topic = rospy.get_param("/stier_slam/topics/inputs/vehicle_speed")
        steering_angle_topic = rospy.get_param("/stier_slam/topics/inputs/steering_angle")
        wheel_odometry_topic = rospy.get_param("/stier_slam/topics/processed/wheel_odometry")
        self._integrator = AckermannIntegrator(
            rospy.get_param("~wheelbase_m"),
            rospy.get_param("~max_abs_steering_rad"),
            rospy.get_param("~max_abs_speed_mps"),
        )
        self._pose_covariance = self._load_covariance("~pose_covariance")
        self._twist_covariance = self._load_covariance("~twist_covariance")
        self._control_pair_timeout_sec = self._load_pair_timeout(
            "~control_pair_timeout_sec"
        )
        self._pair_lock = threading.RLock()
        self._pending_speed = None
        self._pending_steering = None
        self._last_receipt_time = None
        self._publisher = rospy.Publisher(wheel_odometry_topic, Odometry, queue_size=10)
        rospy.Subscriber(steering_angle_topic, Float64, self._on_steering, queue_size=10)
        rospy.Subscriber(
            vehicle_speed_topic, TwistWithCovarianceStamped, self._on_speed, queue_size=10
        )

    def _on_steering(self, message):
        if not math.isfinite(message.data):
            rospy.logwarn("ignoring non-finite steering input")
            return
        with self._pair_lock:
            try:
                receipt_time = self._receipt_time()
            except (AttributeError, TypeError, ValueError) as error:
                rospy.logwarn("rejecting steering receipt time: %s", error)
                return
            self._observe_receipt_time(receipt_time)
            self._expire_pending(receipt_time)
            if self._pending_speed is not None:
                speed_sample, _ = self._pending_speed
                self._pending_speed = None
                self._integrate_pair(speed_sample, message.data)
                return
            if self._pending_steering is not None:
                rospy.logwarn_throttle(5.0, "replacing unpaired steering sample")
            self._pending_steering = (message.data, receipt_time)

    def _on_speed(self, message):
        try:
            speed_sample = self._speed_sample(message)
        except (AttributeError, TypeError, ValueError) as error:
            rospy.logwarn("rejecting wheel odometry update: %s", error)
            return
        with self._pair_lock:
            try:
                receipt_time = self._receipt_time()
            except (AttributeError, TypeError, ValueError) as error:
                rospy.logwarn("rejecting wheel odometry receipt time: %s", error)
                return
            self._observe_receipt_time(receipt_time)
            self._expire_pending(receipt_time)
            if self._pending_steering is not None:
                steering_rad, _ = self._pending_steering
                self._pending_steering = None
                self._integrate_pair(speed_sample, steering_rad)
                return
            if self._pending_speed is not None:
                rospy.logwarn_throttle(5.0, "replacing unpaired vehicle speed sample")
            self._pending_speed = (speed_sample, receipt_time)

    @staticmethod
    def _speed_sample(message):
        source_stamp = message.header.stamp
        stamp = source_stamp.to_sec()
        speed_mps = message.twist.twist.linear.x
        twist_covariance = list(message.twist.covariance)
        if not math.isfinite(stamp) or stamp <= 0.0:
            raise ValueError("vehicle speed source stamp must be finite and non-zero")
        if not math.isfinite(speed_mps):
            raise ValueError("vehicle speed must be finite")
        if len(twist_covariance) != 36 or not all(
                math.isfinite(value) for value in twist_covariance):
            raise ValueError("vehicle speed covariance must contain 36 finite values")
        return source_stamp, stamp, speed_mps

    @staticmethod
    def _receipt_time():
        receipt_time = rospy.Time.now().to_sec()
        if not math.isfinite(receipt_time) or receipt_time < 0.0:
            raise ValueError("control receipt time must be finite and non-negative")
        return receipt_time

    def _observe_receipt_time(self, receipt_time):
        if self._last_receipt_time is not None and receipt_time < self._last_receipt_time:
            self._pending_speed = None
            self._pending_steering = None
            self._integrator.reset()
            rospy.logwarn(
                "control receipt clock reversed; cleared pending pair and reset odometry"
            )
        self._last_receipt_time = receipt_time

    def _expire_pending(self, receipt_time):
        for attribute, label in (
                ("_pending_speed", "vehicle speed"),
                ("_pending_steering", "steering")):
            pending = getattr(self, attribute)
            if pending is None:
                continue
            if receipt_time - pending[1] > self._control_pair_timeout_sec:
                setattr(self, attribute, None)
                rospy.logwarn_throttle(5.0, "expired unpaired %s sample", label)

    def _integrate_pair(self, speed_sample, steering_rad):
        source_stamp, stamp, speed_mps = speed_sample
        try:
            state = self._integrator.update(stamp, speed_mps, steering_rad)
        except (TypeError, ValueError, NonMonotonicTimeError) as error:
            rospy.logwarn("rejecting wheel odometry update: %s", error)
            return
        self._publish(state, source_stamp)

    @staticmethod
    def _load_pair_timeout(parameter_name):
        value = rospy.get_param(parameter_name)
        if (isinstance(value, bool) or not isinstance(value, (int, float))
                or not math.isfinite(value) or value <= 0.0 or value > 1.0):
            raise ValueError("{} must be finite, positive, and at most 1.0".format(
                parameter_name
            ))
        return float(value)

    @staticmethod
    def _load_covariance(parameter_name):
        try:
            covariance = list(rospy.get_param(parameter_name))
        except TypeError as error:
            raise ValueError("{} must contain 36 finite values".format(parameter_name)) from error
        if len(covariance) != 36 or not all(math.isfinite(value) for value in covariance):
            raise ValueError("{} must contain 36 finite values".format(parameter_name))
        return covariance

    def _publish(self, state, source_stamp):
        message = Odometry()
        # float 초 단위로 왕복 변환하면 1 ns가 손실될 수 있으므로 source nanosecond를 보존한다.
        message.header.stamp = source_stamp
        message.header.frame_id = "odom"
        message.child_frame_id = "base_link"
        message.pose.pose.position.x = state.x
        message.pose.pose.position.y = state.y
        message.pose.pose.orientation.z = math.sin(state.yaw * 0.5)
        message.pose.pose.orientation.w = math.cos(state.yaw * 0.5)
        message.pose.covariance = self._pose_covariance
        message.twist.twist.linear.x = state.linear_speed
        message.twist.twist.angular.z = state.yaw_rate
        message.twist.covariance = self._twist_covariance
        self._publisher.publish(message)


def main():
    rospy.init_node("vehicle_odometry")
    VehicleOdometryNode()
    rospy.spin()


if __name__ == "__main__":
    main()
