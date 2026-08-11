#!/usr/bin/env python3
"""hardware device를 열지 않고 일관된 결정적 SLAM 입력을 발행한다."""

import math
import threading
import time

import rospy
import tf2_ros
from diagnostic_msgs.msg import DiagnosticStatus
from geometry_msgs.msg import TransformStamped, TwistWithCovarianceStamped
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Imu, LaserScan, NavSatFix, NavSatStatus
from std_msgs.msg import Float64


MAX_PUBLISH_RATE_HZ = 1000.0
SCAN_BEAM_COUNT = 360
SCAN_RANGE_MIN_M = 0.05
SCAN_RANGE_MAX_M = 10.0
BASE_LATITUDE = 37.239
BASE_LONGITUDE = 126.773
EARTH_RADIUS_M = 6378137.0
STATIC_WALL_X_M = 3.125
STATIC_WALL_Y_M = 2.125
# demo map에서 양의 방향 occupied border cell은 x=[3.0, 3.25) 범위다.
MAP_FREE_X_MAX_M = 3.0
SYNTHETIC_SPEED_MPS = 0.5
TEMPORARY_OBSTACLE_RANGE_M = 1.0
TEMPORARY_OBSTACLE_HALF_ANGLE_RAD = math.radians(6.0)
TEMPORARY_OBSTACLE_FREE_MARGIN_M = 0.05
UNMEASURED_VARIANCE = 1.0e6
METERS_PER_DEGREE_LONGITUDE = (
    math.pi * EARTH_RADIUS_M * math.cos(math.radians(BASE_LATITUDE)) / 180.0
)


class SyntheticSensorNode:
    """monotonic wall time에 맞춰 yaw가 0인 왕복 trajectory를 발행한다."""

    def __init__(self):
        real_mode = _validated_boolean(
            rospy.get_param("~real_mode", False), "~real_mode"
        )
        if real_mode:
            raise ValueError("synthetic sensors cannot run with ~real_mode enabled")

        self._publish_clock = _validated_boolean(
            rospy.get_param("~publish_clock", True), "~publish_clock"
        )
        self._publish_rate_hz, self._cycle_samples = (
            _validated_trajectory_parameters(
                rospy.get_param("~publish_rate_hz", 20.0),
                rospy.get_param("~cycle_samples", 40),
            )
        )
        self._sequence_start_sec = _positive_finite(
            rospy.get_param("~sequence_start_sec", 100.0), "~sequence_start_sec"
        )
        self._period_nsec = int(round(1.0e9 / self._publish_rate_hz))
        self._sequence_start_nsec = None
        self._obstacle_start_sample = _non_negative_integer(
            rospy.get_param("~obstacle_start_sample", 10), "~obstacle_start_sample"
        )
        self._obstacle_end_sample = _positive_integer(
            rospy.get_param("~obstacle_end_sample", 20), "~obstacle_end_sample"
        )
        if not (
                self._obstacle_start_sample
                < self._obstacle_end_sample
                <= self._cycle_samples):
            raise ValueError(
                "obstacle sample interval must be non-empty and inside ~cycle_samples"
            )

        self._base_frame = _frame(
            rospy.get_param("~base_frame", "base_link"), "~base_frame"
        )
        self._lidar_frame = _frame(
            rospy.get_param("~lidar_frame", "lidar_link"), "~lidar_frame"
        )
        self._imu_frame = _frame(
            rospy.get_param("~imu_frame", "imu_link"), "~imu_frame"
        )
        self._gnss_frame = _frame(
            rospy.get_param("~gnss_frame", "gps_link"), "~gnss_frame"
        )
        self._camera_frame = _frame(
            rospy.get_param("~camera_frame", "camera_link"), "~camera_frame"
        )
        frames = {
            self._base_frame,
            self._lidar_frame,
            self._imu_frame,
            self._gnss_frame,
            self._camera_frame,
        }
        if len(frames) != 5:
            raise ValueError("synthetic frame names must be distinct")

        self._scan_publisher = rospy.Publisher(
            "/slam/input/scan_raw", LaserScan, queue_size=10
        )
        self._imu_publisher = rospy.Publisher("/slam/input/imu", Imu, queue_size=10)
        self._fix_publisher = rospy.Publisher(
            "/slam/input/gnss/fix", NavSatFix, queue_size=10
        )
        self._rtk_status_publisher = rospy.Publisher(
            "/slam/input/gnss/rtk_status", DiagnosticStatus, queue_size=10
        )
        self._speed_publisher = rospy.Publisher(
            "/slam/input/vehicle_speed", TwistWithCovarianceStamped, queue_size=10
        )
        self._steering_publisher = rospy.Publisher(
            "/slam/input/steering_angle", Float64, queue_size=10
        )
        self._clock_publisher = (
            rospy.Publisher("/clock", Clock, queue_size=10)
            if self._publish_clock
            else None
        )
        self._static_broadcaster = tf2_ros.StaticTransformBroadcaster()
        self._shutdown = threading.Event()
        rospy.on_shutdown(self._shutdown.set)

    def run(self):
        self._initialize_sequence_origin()
        sample_index = 0
        period_sec = self._period_nsec / 1.0e9
        monotonic_origin_sec = time.monotonic()
        previous_monotonic_sec = monotonic_origin_sec
        next_deadline = monotonic_origin_sec
        self._publish_static_transforms(self._stamp(0))
        while not rospy.is_shutdown():
            stamp = self._stamp(sample_index)
            if self._clock_publisher is not None:
                self._clock_publisher.publish(Clock(clock=stamp))
            self._publish_sample(stamp, sample_index)
            current_monotonic_sec = time.monotonic()
            if self._publish_clock:
                sample_index += 1
                next_deadline += period_sec
                if current_monotonic_sec > next_deadline:
                    next_deadline = current_monotonic_sec
            else:
                sample_index, next_deadline = _next_wall_schedule(
                    sample_index,
                    monotonic_origin_sec,
                    previous_monotonic_sec,
                    current_monotonic_sec,
                    period_sec,
                )
                previous_monotonic_sec = current_monotonic_sec

            remaining_sec = next_deadline - current_monotonic_sec
            if remaining_sec > 0.0:
                self._shutdown.wait(remaining_sec)

    def _initialize_sequence_origin(self):
        if self._publish_clock:
            self._sequence_start_nsec = int(round(self._sequence_start_sec * 1.0e9))
            return
        self._sequence_start_nsec = rospy.Time.now().to_nsec()
        if self._sequence_start_nsec <= 0:
            raise ValueError("wall-time mode requires a positive current ROS epoch")

    def _stamp(self, sample_index):
        total_nsec = self._sequence_start_nsec + sample_index * self._period_nsec
        seconds, nanoseconds = divmod(total_nsec, 1000000000)
        return rospy.Time(seconds, nanoseconds)

    def _publish_sample(self, stamp, sample_index):
        phase = sample_index % self._cycle_samples
        pose_x = self._pose_x(phase)
        speed_mps = self._speed(phase)
        previous_speed_mps = self._speed((phase - 1) % self._cycle_samples)
        acceleration_x = (
            speed_mps - previous_speed_mps
        ) * self._publish_rate_hz
        obstacle_active = (
            self._obstacle_start_sample <= phase < self._obstacle_end_sample
        )

        # odometry consumer는 두 가지 TCPROS callback 도착 순서를 모두 지원한다.
        self._steering_publisher.publish(Float64(data=0.0))
        self._speed_publisher.publish(self._speed_message(stamp, speed_mps))
        self._imu_publisher.publish(self._imu_message(stamp, acceleration_x))
        self._scan_publisher.publish(
            self._scan_message(stamp, pose_x, obstacle_active)
        )
        # RTK gate는 각 NavSatFix 전에 최신 FIX 상태를 요구한다.
        self._rtk_status_publisher.publish(self._rtk_status_message())
        self._fix_publisher.publish(self._fix_message(stamp, pose_x))

    def _speed(self, phase):
        half_cycle = self._cycle_samples // 2
        if phase in (0, half_cycle):
            return 0.0
        return SYNTHETIC_SPEED_MPS if phase < half_cycle else -SYNTHETIC_SPEED_MPS

    def _pose_x(self, phase):
        return _synthetic_pose_x(
            phase, self._publish_rate_hz, self._cycle_samples
        )

    def _speed_message(self, stamp, speed_mps):
        message = TwistWithCovarianceStamped()
        message.header.stamp = stamp
        message.header.frame_id = self._base_frame
        message.twist.twist.linear.x = speed_mps
        for index in (7, 14, 21, 28, 35):
            message.twist.covariance[index] = UNMEASURED_VARIANCE
        message.twist.covariance[0] = 0.01
        return message

    def _imu_message(self, stamp, acceleration_x):
        message = Imu()
        message.header.stamp = stamp
        message.header.frame_id = self._imu_frame
        message.orientation.w = 1.0
        message.orientation_covariance = [
            0.001, 0.0, 0.0,
            0.0, 0.001, 0.0,
            0.0, 0.0, 0.001,
        ]
        message.angular_velocity_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01,
        ]
        message.linear_acceleration.x = acceleration_x
        message.linear_acceleration.z = 9.80665
        message.linear_acceleration_covariance = [
            0.04, 0.0, 0.0,
            0.0, 0.04, 0.0,
            0.0, 0.0, 0.04,
        ]
        return message

    def _scan_message(self, stamp, pose_x, obstacle_active):
        angle_min = -math.pi
        angle_increment = 2.0 * math.pi / SCAN_BEAM_COUNT
        ranges = _synthetic_scan_ranges(pose_x, obstacle_active)

        message = LaserScan()
        message.header.stamp = stamp
        message.header.frame_id = self._lidar_frame
        message.angle_min = angle_min
        message.angle_increment = angle_increment
        message.angle_max = angle_min + (SCAN_BEAM_COUNT - 1) * angle_increment
        message.time_increment = 1.0 / (
            self._publish_rate_hz * SCAN_BEAM_COUNT
        )
        message.scan_time = 1.0 / self._publish_rate_hz
        message.range_min = SCAN_RANGE_MIN_M
        message.range_max = SCAN_RANGE_MAX_M
        message.ranges = ranges
        return message

    def _fix_message(self, stamp, pose_x):
        message = NavSatFix()
        message.header.stamp = stamp
        message.header.frame_id = self._gnss_frame
        message.status.status = NavSatStatus.STATUS_GBAS_FIX
        message.status.service = NavSatStatus.SERVICE_GPS
        message.latitude = BASE_LATITUDE
        message.longitude = BASE_LONGITUDE + pose_x / METERS_PER_DEGREE_LONGITUDE
        message.altitude = 10.0
        message.position_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.04,
        ]
        message.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        return message

    @staticmethod
    def _rtk_status_message():
        message = DiagnosticStatus()
        message.level = DiagnosticStatus.OK
        message.name = "synthetic_rtk"
        message.message = "FIX"
        message.hardware_id = "synthetic"
        return message

    def _publish_static_transforms(self, stamp):
        transforms = []
        for child_frame in (
                self._lidar_frame,
                self._imu_frame,
                self._gnss_frame,
                self._camera_frame):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._base_frame
            transform.child_frame_id = child_frame
            transform.transform.rotation.w = 1.0
            transforms.append(transform)
        self._static_broadcaster.sendTransform(transforms)


def _wrapped_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _synthetic_pose_x(phase, publish_rate_hz, cycle_samples):
    half_cycle = cycle_samples // 2
    step_distance = SYNTHETIC_SPEED_MPS / publish_rate_hz
    if phase < half_cycle:
        return phase * step_distance
    return (cycle_samples - 1 - phase) * step_distance


def _synthetic_scan_ranges(pose_x, obstacle_active):
    angle_min = -math.pi
    angle_increment = 2.0 * math.pi / SCAN_BEAM_COUNT
    ranges = []
    for index in range(SCAN_BEAM_COUNT):
        angle = angle_min + index * angle_increment
        cosine = math.cos(angle)
        sine = math.sin(angle)
        if cosine > 1.0e-12:
            x_boundary = (STATIC_WALL_X_M - pose_x) / cosine
        elif cosine < -1.0e-12:
            x_boundary = (-STATIC_WALL_X_M - pose_x) / cosine
        else:
            x_boundary = math.inf
        if sine > 1.0e-12:
            y_boundary = STATIC_WALL_Y_M / sine
        elif sine < -1.0e-12:
            y_boundary = -STATIC_WALL_Y_M / sine
        else:
            y_boundary = math.inf
        distance = min(x_boundary, y_boundary)
        if (
                obstacle_active
                and abs(_wrapped_angle(angle))
                <= TEMPORARY_OBSTACLE_HALF_ANGLE_RAD):
            distance = TEMPORARY_OBSTACLE_RANGE_M
        ranges.append(distance)
    return ranges


def _validated_publish_rate(value):
    rate_hz = _positive_finite(value, "~publish_rate_hz")
    if rate_hz > MAX_PUBLISH_RATE_HZ:
        raise ValueError("~publish_rate_hz must be at most {}".format(
            MAX_PUBLISH_RATE_HZ
        ))
    return rate_hz


def _validated_cycle_samples(value):
    if (
            isinstance(value, bool)
            or not isinstance(value, int)
            or value < 4
            or value % 2 != 0):
        raise ValueError("~cycle_samples must be an even integer of at least 4")
    return value


def _validated_trajectory_parameters(publish_rate_hz, cycle_samples):
    publish_rate_hz = _validated_publish_rate(publish_rate_hz)
    cycle_samples = _validated_cycle_samples(cycle_samples)
    minimum_front_wall_range = (
        TEMPORARY_OBSTACLE_RANGE_M + SCAN_RANGE_MIN_M
    )
    maximum_pose_steps = cycle_samples // 2 - 1
    maximum_pose_x_allowed = (
        MAP_FREE_X_MAX_M
        - TEMPORARY_OBSTACLE_RANGE_M
        - TEMPORARY_OBSTACLE_FREE_MARGIN_M
    )
    maximum_pose_steps_allowed = (
        maximum_pose_x_allowed * publish_rate_hz / SYNTHETIC_SPEED_MPS
    )
    # 악의적으로 큰 cycle의 step count를 float로 변환할 때 overflow되지 않도록
    # 범위 제한이 없는 integer를 먼저 비교한다.
    if maximum_pose_steps > maximum_pose_steps_allowed + 1.0e-12:
        raise ValueError(
            "unsafe synthetic trajectory geometry: obstacle endpoint must stay "
            "inside map free-space with margin and all scans must remain in range"
        )
    maximum_pose_x = (
        maximum_pose_steps * SYNTHETIC_SPEED_MPS / publish_rate_hz
    )
    front_wall_range = STATIC_WALL_X_M - maximum_pose_x
    farthest_wall_range = math.hypot(
        STATIC_WALL_X_M + maximum_pose_x, STATIC_WALL_Y_M
    )
    if (
            front_wall_range + 1.0e-12 < minimum_front_wall_range
            or farthest_wall_range > SCAN_RANGE_MAX_M + 1.0e-12):
        raise ValueError(
            "unsafe synthetic trajectory geometry: obstacle endpoint must stay "
            "inside map free-space with margin and all scans must remain in range"
        )
    return publish_rate_hz, cycle_samples


def _next_wall_schedule(
        current_sample_index,
        monotonic_origin_sec,
        previous_monotonic_sec,
        current_monotonic_sec,
        period_sec):
    """원점에 맞춘 epoch를 유지하면서 놓친 wall-time tick을 건너뛴다."""
    if (
            isinstance(current_sample_index, bool)
            or not isinstance(current_sample_index, int)
            or current_sample_index < 0):
        raise ValueError("current sample index must be a non-negative integer")
    values = (
        monotonic_origin_sec,
        previous_monotonic_sec,
        current_monotonic_sec,
        period_sec,
    )
    if any(
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            for value in values):
        raise ValueError("wall schedule times must be finite numbers")
    if period_sec <= 0.0:
        raise ValueError("wall schedule period must be positive")
    if not (
            monotonic_origin_sec
            <= previous_monotonic_sec
            <= current_monotonic_sec):
        raise ValueError("wall schedule monotonic time must not reverse")

    elapsed_periods = (
        current_monotonic_sec - monotonic_origin_sec
    ) / period_sec
    origin_aligned_index = math.ceil(elapsed_periods - 1.0e-12)
    next_sample_index = max(current_sample_index + 1, origin_aligned_index)
    next_deadline = (
        monotonic_origin_sec + next_sample_index * period_sec
    )
    if not math.isfinite(next_deadline):
        raise ValueError("wall schedule deadline must remain finite")
    return next_sample_index, next_deadline


def _validated_boolean(value, name):
    if not isinstance(value, bool):
        raise ValueError("{} must be a boolean".format(name))
    return value


def _positive_finite(value, name):
    if isinstance(value, bool):
        raise ValueError("{} must be finite and positive".format(name))
    try:
        number = float(value)
    except (TypeError, ValueError) as error:
        raise ValueError("{} must be finite and positive".format(name)) from error
    if not math.isfinite(number) or number <= 0.0:
        raise ValueError("{} must be finite and positive".format(name))
    return number


def _positive_integer(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value <= 0:
        raise ValueError("{} must be a positive integer".format(name))
    return value


def _non_negative_integer(value, name):
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError("{} must be a non-negative integer".format(name))
    return value


def _frame(value, name):
    if (
            not isinstance(value, str)
            or not value.strip()
            or value != value.strip()
            or value.startswith("/")):
        raise ValueError("{} must be a non-empty relative frame".format(name))
    return value


def main():
    rospy.init_node("synthetic_sensor")
    SyntheticSensorNode().run()


if __name__ == "__main__":
    main()
