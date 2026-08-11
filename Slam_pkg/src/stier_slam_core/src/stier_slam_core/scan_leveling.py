"""외부 의존성 없이 2D LiDAR scan을 검증하고 roll/pitch를 보정한다."""

import math


class ScanValidationError(ValueError):
    """scan metadata 또는 filtering 설정이 안전하지 않을 때 발생한다."""


class QuaternionValidationError(ScanValidationError):
    """IMU orientation으로 유효한 회전을 만들 수 없을 때 발생한다."""


class ExcessiveTiltError(ScanValidationError):
    """roll 또는 pitch가 설정된 scan leveling 한계를 넘을 때 발생한다."""


def normalize_quaternion(qx, qy, qz, qw):
    """ROS ``x, y, z, w`` 순서의 유한한 unit quaternion을 반환한다."""
    values = _finite_tuple((qx, qy, qz, qw), "quaternion", QuaternionValidationError)
    norm = math.sqrt(sum(value * value for value in values))
    if not math.isfinite(norm) or norm == 0.0:
        raise QuaternionValidationError("quaternion norm must be finite and non-zero")
    return tuple(value / norm for value in values)


def roll_pitch_from_quaternion(quaternion):
    """yaw를 의도적으로 제외하고 base_link의 roll과 pitch를 추출한다."""
    values = _finite_tuple(quaternion, "quaternion", QuaternionValidationError)
    if len(values) != 4:
        raise QuaternionValidationError("quaternion must contain four finite values")
    qx, qy, qz, qw = normalize_quaternion(*values)
    roll = math.atan2(
        2.0 * (qw * qx + qy * qz),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
    sin_pitch = 2.0 * (qw * qy - qz * qx)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))
    return roll, pitch


def validate_max_abs_tilt(quaternion, max_abs_tilt_rad):
    """roll/pitch를 반환하며 어느 하나라도 한계를 넘으면 예외를 발생시킨다."""
    (max_abs_tilt_rad,) = _finite_tuple(
        (max_abs_tilt_rad,), "max_abs_tilt_rad", ScanValidationError
    )
    if max_abs_tilt_rad < 0.0:
        raise ScanValidationError("max_abs_tilt_rad must be non-negative")
    roll, pitch = roll_pitch_from_quaternion(quaternion)
    if abs(roll) > max_abs_tilt_rad or abs(pitch) > max_abs_tilt_rad:
        raise ExcessiveTiltError("IMU roll or pitch exceeds max_abs_tilt_rad")
    return roll, pitch


def level_scan(
        ranges, angle_min, angle_increment, range_min, range_max, quaternion,
        mount_xyz_rpy, min_z_m, max_z_m):
    """유효한 2D range를 roll/pitch가 보정된 ``base_link`` cloud로 투영한다."""
    angle_min, angle_increment, range_min, range_max = _finite_tuple(
        (angle_min, angle_increment, range_min, range_max),
        "scan metadata",
        ScanValidationError,
    )
    min_z_m, max_z_m = _finite_tuple(
        (min_z_m, max_z_m), "z filter limits", ScanValidationError
    )
    if angle_increment == 0.0:
        raise ScanValidationError("angle_increment must be non-zero")
    if range_min < 0.0 or range_max <= range_min:
        raise ScanValidationError("range limits must be finite, non-negative, and increasing")
    if max_z_m < min_z_m:
        raise ScanValidationError("max_z_m must be greater than or equal to min_z_m")

    mount = _finite_tuple(mount_xyz_rpy, "mount_xyz_rpy", ScanValidationError)
    if len(mount) != 6:
        raise ScanValidationError("mount_xyz_rpy must contain six finite values")
    mount_x, mount_y, mount_z, mount_roll, mount_pitch, mount_yaw = mount
    mount_rotation = _rpy_matrix(mount_roll, mount_pitch, mount_yaw)
    roll, pitch = roll_pitch_from_quaternion(quaternion)
    leveling_rotation = _rpy_matrix(roll, pitch, 0.0)

    points = []
    has_usable_range = False
    try:
        indexed_ranges = enumerate(ranges)
        for index, distance in indexed_ranges:
            try:
                finite_distance = math.isfinite(distance)
            except TypeError as error:
                raise ScanValidationError("ranges must contain numeric values") from error
            if not finite_distance:
                continue
            if distance < range_min or distance > range_max:
                continue
            has_usable_range = True

            angle = angle_min + index * angle_increment
            point_lidar = (distance * math.cos(angle), distance * math.sin(angle), 0.0)
            rotated_mount_point = _matrix_vector_product(mount_rotation, point_lidar)
            point_base = (
                rotated_mount_point[0] + mount_x,
                rotated_mount_point[1] + mount_y,
                rotated_mount_point[2] + mount_z,
            )
            point_leveled = _matrix_vector_product(leveling_rotation, point_base)
            if (not all(math.isfinite(value) for value in point_leveled)
                    or point_leveled[2] < min_z_m or point_leveled[2] > max_z_m):
                continue
            points.append(point_leveled)
    except TypeError as error:
        raise ScanValidationError("ranges must be iterable") from error

    if not has_usable_range:
        raise ScanValidationError("scan contains no finite in-range values")
    return points


def _finite_tuple(values, name, error_type):
    try:
        converted = tuple(values)
    except TypeError as error:
        raise error_type("{} must be iterable".format(name)) from error
    try:
        if not all(math.isfinite(value) for value in converted):
            raise error_type("{} must contain only finite values".format(name))
    except TypeError as error:
        raise error_type("{} must contain numeric values".format(name)) from error
    return converted


def _rpy_matrix(roll, pitch, yaw):
    cosine_roll = math.cos(roll)
    sine_roll = math.sin(roll)
    cosine_pitch = math.cos(pitch)
    sine_pitch = math.sin(pitch)
    cosine_yaw = math.cos(yaw)
    sine_yaw = math.sin(yaw)
    return (
        (
            cosine_yaw * cosine_pitch,
            cosine_yaw * sine_pitch * sine_roll - sine_yaw * cosine_roll,
            cosine_yaw * sine_pitch * cosine_roll + sine_yaw * sine_roll,
        ),
        (
            sine_yaw * cosine_pitch,
            sine_yaw * sine_pitch * sine_roll + cosine_yaw * cosine_roll,
            sine_yaw * sine_pitch * cosine_roll - cosine_yaw * sine_roll,
        ),
        (-sine_pitch, cosine_pitch * sine_roll, cosine_pitch * cosine_roll),
    )


def _matrix_vector_product(matrix, vector):
    return tuple(sum(component * value for component, value in zip(row, vector)) for row in matrix)
