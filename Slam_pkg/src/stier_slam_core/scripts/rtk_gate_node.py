#!/usr/bin/env python3
"""최신 상태이며 검증된 RTK FIX 관측만 SLAM으로 전달한다."""

import copy
import math

import rospy
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue
from sensor_msgs.msg import NavSatFix

from stier_slam_core.rtk_gate import RtkFix, RtkGate


class CovarianceValidationError(ValueError):
    """일관된 covariance 거절 이유를 diagnostic 경계까지 전달한다."""

    def __init__(self, reason):
        super().__init__(reason)
        self.reason = reason


class RtkGateNode:
    """역할:
    RTK 상태와 ``NavSatFix``를 검증해 신뢰할 수 있는 GNSS 관측만 publish한다.

    동작 방식:
    최신 RTK 상태와 fix를 ``RtkGate`` 입력으로 변환하고, 수용된 fix는 원본 timestamp를
    유지해 `/slam/gnss/fix_accepted`로 전달하며 모든 판정 결과를 diagnostic으로 남긴다.
    """

    def __init__(self):
        gnss_fix_topic = rospy.get_param("/stier_slam/topics/inputs/gnss_fix")
        rtk_status_topic = rospy.get_param("/stier_slam/topics/inputs/gnss_rtk_status")
        accepted_gnss_topic = rospy.get_param("/stier_slam/topics/processed/accepted_gnss")
        rtk_diagnostics_topic = rospy.get_param("/stier_slam/topics/outputs/rtk_diagnostics")
        self._gate = RtkGate(
            rospy.get_param("~max_covariance_xy_m2"),
            rospy.get_param("~max_speed_mps"),
            rospy.get_param("~innovation_margin_m"),
        )
        self._rtk_state_max_age_sec = rospy.get_param("~rtk_state_max_age_sec")
        if not _is_finite_number(self._rtk_state_max_age_sec) or self._rtk_state_max_age_sec < 0.0:
            raise ValueError("~rtk_state_max_age_sec must be finite and non-negative")
        self._rtk_state = None
        self._rtk_state_receipt_time = None
        self._rejection_counts = {}
        self._accepted_count = 0
        self._accepted_publisher = rospy.Publisher(
            accepted_gnss_topic, NavSatFix, queue_size=10
        )
        self._diagnostic_publisher = rospy.Publisher(
            rtk_diagnostics_topic, DiagnosticStatus, queue_size=10
        )
        rospy.Subscriber(
            rtk_status_topic, DiagnosticStatus, self._on_rtk_status, queue_size=10
        )
        rospy.Subscriber(gnss_fix_topic, NavSatFix, self._on_fix, queue_size=10)

    def _on_rtk_status(self, message):
        state = message.message
        self._rtk_state = state if state in ("FIX", "FLOAT", "NO_FIX") else "NO_FIX"
        self._rtk_state_receipt_time = rospy.Time.now().to_sec()

    def _on_fix(self, message):
        rtk_state, state_reason = self._fresh_rtk_state()
        if state_reason is not None:
            self._record_rejection(state_reason)
            self._publish_diagnostic(state_reason, rtk_state)
            return

        try:
            fix = _rtk_fix_from_message(message)
        except CovarianceValidationError as error:
            self._record_rejection(error.reason)
            self._publish_diagnostic(error.reason, rtk_state)
            return
        decision = self._gate.evaluate(fix, rtk_state)
        if decision.accepted:
            self._accepted_count += 1
            self._accepted_publisher.publish(copy.deepcopy(message))
        else:
            self._record_rejection(decision.reason)
        self._publish_diagnostic(decision.reason, rtk_state)

    def _fresh_rtk_state(self):
        if self._rtk_state_receipt_time is None:
            return "NO_FIX", "rtk_state_stale"
        age_sec = rospy.Time.now().to_sec() - self._rtk_state_receipt_time
        if (not _is_finite_number(age_sec) or age_sec < 0.0
                or age_sec > self._rtk_state_max_age_sec):
            self._rtk_state = None
            self._rtk_state_receipt_time = None
            return "NO_FIX", "rtk_state_stale"
        return self._rtk_state, None

    def _record_rejection(self, reason):
        self._rejection_counts[reason] = self._rejection_counts.get(reason, 0) + 1

    def _publish_diagnostic(self, reason, rtk_state):
        status = DiagnosticStatus()
        status.name = "rtk_gate"
        status.message = reason
        status.level = _diagnostic_level(reason, rtk_state)
        values = [
            _key_value("rtk_state", rtk_state),
            _key_value("accepted", self._accepted_count),
        ]
        for rejection_reason in sorted(self._rejection_counts):
            values.append(_key_value(
                "rejected_{}".format(rejection_reason),
                self._rejection_counts[rejection_reason],
            ))
        status.values = values
        self._diagnostic_publisher.publish(status)


def _rtk_fix_from_message(message):
    if message.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
        raise CovarianceValidationError("covariance_unknown")
    covariance = list(message.position_covariance)
    if len(covariance) < 5:
        raise CovarianceValidationError("covariance_not_finite")
    covariance_x_m2 = covariance[0]
    covariance_y_m2 = covariance[4]
    if not _is_finite_number(covariance_x_m2) or not _is_finite_number(covariance_y_m2):
        raise CovarianceValidationError("covariance_not_finite")
    if covariance_x_m2 < 0.0 or covariance_y_m2 < 0.0:
        raise CovarianceValidationError("covariance_negative")
    return RtkFix(
        message.header.stamp.to_sec(),
        message.latitude,
        message.longitude,
        message.altitude,
        max(covariance_x_m2, covariance_y_m2),
        message.status.status,
    )


def _diagnostic_level(reason, rtk_state):
    if reason == "accepted":
        return DiagnosticStatus.OK
    if reason == "rtk_state_stale" or rtk_state == "NO_FIX":
        return DiagnosticStatus.ERROR
    return DiagnosticStatus.WARN


def _key_value(key, value):
    pair = KeyValue()
    pair.key = key
    pair.value = str(value)
    return pair


def _is_finite_number(value):
    try:
        return math.isfinite(value)
    except TypeError:
        return False


def main():
    rospy.init_node("rtk_gate")
    RtkGateNode()
    rospy.spin()


if __name__ == "__main__":
    main()
