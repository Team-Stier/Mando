#!/usr/bin/env python3
"""normalized sensor timestamp health를 공용 diagnostics topic으로 발행한다."""

import math
import threading

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TwistWithCovarianceStamped
from sensor_msgs.msg import Image, Imu, LaserScan, NavSatFix
from std_msgs.msg import Float64

from stier_slam_core.freshness import (
    FUTURE,
    MISSING,
    OK,
    STALE,
    TIME_REVERSED,
    FreshnessMonitor,
    SensorConfig,
)


SENSOR_CONTRACTS = (
    ("scan_raw", LaserScan, True),
    ("imu", Imu, True),
    ("gnss_fix", NavSatFix, True),
    ("gnss_rtk_status", DiagnosticStatus, False),
    ("vehicle_speed", TwistWithCovarianceStamped, True),
    ("steering_angle", Float64, False),
    ("camera_front_image", Image, True),
)
_STATE_PRIORITY = {OK: 0, STALE: 1, MISSING: 2, FUTURE: 3, TIME_REVERSED: 4}


class SensorHealthNode:
    """역할:
    normalized SLAM 입력들의 timestamp, 수신 지연과 주기를 통합 감시한다.

    동작 방식:
    각 sensor topic callback을 직렬화해 ``FreshnessMonitor``를 갱신하고, timer마다
    평가 결과를 `/slam/diagnostics`의 ``DiagnosticArray``로 변환해 발행한다.
    """

    def __init__(self):
        input_topics = rospy.get_param("/stier_slam/topics/inputs")
        diagnostics_topic = rospy.get_param("/stier_slam/topics/outputs/diagnostics")
        configs = _sensor_configs(rospy.get_param("~sensors"))
        future_tolerance_sec = rospy.get_param("~future_tolerance_sec")
        publish_rate_hz = rospy.get_param("~publish_rate_hz")
        if not _finite_positive(publish_rate_hz):
            raise ValueError("~publish_rate_hz must be finite and positive")

        self._monitor = FreshnessMonitor(configs, future_tolerance_sec)
        self._lock = threading.RLock()
        self._publisher = rospy.Publisher(diagnostics_topic, DiagnosticArray, queue_size=10)
        for name, message_type, has_header_stamp in SENSOR_CONTRACTS:
            topic = input_topics[name]
            callback = self._stamped_callback(name) if has_header_stamp else self._unstamped_callback(name)
            rospy.Subscriber(topic, message_type, callback, queue_size=10)
        self._timer = rospy.Timer(
            rospy.Duration(1.0 / publish_rate_hz), self._on_timer, reset=True
        )

    def _stamped_callback(self, name):
        return lambda message: self._on_stamped(name, message)

    def _unstamped_callback(self, name):
        return lambda message: self._on_unstamped(name, message)

    def _on_stamped(self, name, message):
        try:
            header_stamp = message.header.stamp.to_sec()
        except (AttributeError, TypeError, ValueError):
            rospy.logwarn_throttle(5.0, "dropping malformed %s header stamp", name)
            return
        self._update(name, header_stamp)

    def _on_unstamped(self, name, _message):
        self._update(name, None)

    def _update(self, name, header_stamp):
        try:
            with self._lock:
                receipt_time = rospy.Time.now().to_sec()
                self._monitor.update(name, header_stamp, receipt_time)
        except (AttributeError, KeyError, TypeError, ValueError):
            rospy.logwarn_throttle(5.0, "dropping malformed %s freshness sample", name)

    def _on_timer(self, _event):
        try:
            with self._lock:
                now = rospy.Time.now().to_sec()
                health_by_sensor = self._monitor.evaluate(now)
        except (AttributeError, TypeError, ValueError):
            rospy.logwarn_throttle(5.0, "dropping non-finite ROS diagnostic time")
            return
        self._publisher.publish(_diagnostic_array(now, health_by_sensor))


def _sensor_configs(raw_configs):
    if not isinstance(raw_configs, dict):
        raise ValueError("~sensors must be a dict")
    expected_names = {contract[0] for contract in SENSOR_CONTRACTS}
    if set(raw_configs) != expected_names:
        raise ValueError("~sensors must configure every normalized sensor exactly once")
    configs = {}
    for name, _message_type, expected_has_header in SENSOR_CONTRACTS:
        raw_config = raw_configs[name]
        if not isinstance(raw_config, dict) or set(raw_config) != {"timeout_sec", "has_header_stamp"}:
            raise ValueError("invalid sensor configuration for {}".format(name))
        if raw_config["has_header_stamp"] is not expected_has_header:
            raise ValueError("header contract mismatch for {}".format(name))
        configs[name] = SensorConfig(
            timeout_sec=raw_config["timeout_sec"],
            has_header_stamp=raw_config["has_header_stamp"],
        )
    return configs


def _diagnostic_array(now, health_by_sensor):
    array = DiagnosticArray()
    array.header.stamp = rospy.Time.from_sec(now)
    statuses = [_sensor_status(name, health_by_sensor[name]) for name, *_rest in SENSOR_CONTRACTS]
    array.status = statuses + [_aggregate_status(health_by_sensor)]
    return array


def _sensor_status(name, health):
    status = DiagnosticStatus()
    status.name = "sensor_health/{}".format(name)
    status.hardware_id = "stier_slam"
    status.level = _diagnostic_level(health.state)
    status.message = health.reason
    status.values = [
        _key_value("state", health.state),
        _key_value("reason", health.reason),
        _key_value("source_age_sec", health.source_age_sec),
        _key_value("receipt_age_sec", health.receipt_age_sec),
        _key_value("rate_hz", health.rate_hz),
        _key_value("last_header_stamp", health.last_header_stamp),
        _key_value("last_receipt_time", health.last_receipt_time),
    ]
    return status


def _aggregate_status(health_by_sensor):
    worst_name, worst_health = max(
        health_by_sensor.items(), key=lambda item: (_STATE_PRIORITY[item[1].state], item[0])
    )
    status = DiagnosticStatus()
    status.name = "sensor_health/aggregate"
    status.hardware_id = "stier_slam"
    status.level = _diagnostic_level(worst_health.state)
    status.message = worst_health.state
    status.values = [
        _key_value("state", worst_health.state),
        _key_value("reason", worst_health.reason),
        _key_value("worst_sensor", worst_name),
    ]
    return status


def _diagnostic_level(state):
    if state == OK:
        return DiagnosticStatus.OK
    if state == STALE:
        return DiagnosticStatus.WARN
    return DiagnosticStatus.ERROR


def _key_value(key, value):
    pair = KeyValue()
    pair.key = key
    pair.value = "" if value is None else str(value)
    return pair


def _finite_positive(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value) and value > 0.0


def main():
    rospy.init_node("sensor_health")
    SensorHealthNode()
    rospy.spin()


if __name__ == "__main__":
    main()
