"""좌·우 wheel encoder로 차량 전진속도를 계산한다."""

import math


class WheelEncoderEstimator:
    """역할:
    이름이 지정된 좌·우 wheel JointState를 차량 중심의 선속도로 변환한다.

    동작 방식:
    joint velocity가 있으면 우선 사용하고, 없으면 연속 position과 source timestamp를
    차분한다. 좌우 방향 부호와 wheel radius를 적용한 뒤 두 바퀴 속도의 평균을 반환한다.
    """

    def __init__(
            self, wheel_radius_m, left_joint_name, right_joint_name,
            left_sign, right_sign, max_abs_wheel_speed_rad_s):
        for name, value in (
                ("wheel_radius_m", wheel_radius_m),
                ("max_abs_wheel_speed_rad_s", max_abs_wheel_speed_rad_s)):
            if not _finite_number(value) or value <= 0.0:
                raise ValueError("{} must be finite and positive".format(name))
        if (not isinstance(left_joint_name, str) or not left_joint_name
                or not isinstance(right_joint_name, str) or not right_joint_name
                or left_joint_name == right_joint_name):
            raise ValueError("wheel joint names must be distinct non-empty strings")
        if left_sign not in (-1.0, 1.0) or right_sign not in (-1.0, 1.0):
            raise ValueError("wheel direction signs must be -1.0 or 1.0")
        self._wheel_radius_m = float(wheel_radius_m)
        self._left_joint_name = left_joint_name
        self._right_joint_name = right_joint_name
        self._left_sign = float(left_sign)
        self._right_sign = float(right_sign)
        self._max_abs_wheel_speed_rad_s = float(max_abs_wheel_speed_rad_s)
        self._last_stamp = None
        self._last_positions = None

    def update(self, stamp, names, positions, velocities):
        """유효한 새 sample의 m/s 속도를 반환하며 첫 position sample은 ``None``이다."""
        if not _finite_number(stamp) or stamp <= 0.0:
            raise ValueError("encoder stamp must be finite and positive")
        if self._last_stamp is not None and stamp <= self._last_stamp:
            raise ValueError("encoder stamps must strictly increase")
        indices = self._joint_indices(names)
        position_pair = self._optional_pair(positions, names, indices, "position")
        velocity_pair = self._optional_pair(velocities, names, indices, "velocity")
        if velocity_pair is not None:
            wheel_speeds = velocity_pair
        elif position_pair is not None:
            if self._last_positions is None:
                self._last_stamp = stamp
                self._last_positions = position_pair
                return None
            dt = stamp - self._last_stamp
            wheel_speeds = tuple(
                (current - previous) / dt
                for current, previous in zip(position_pair, self._last_positions)
            )
        else:
            raise ValueError("JointState must provide velocity or position for both wheels")

        signed = (
            wheel_speeds[0] * self._left_sign,
            wheel_speeds[1] * self._right_sign,
        )
        if any(abs(value) > self._max_abs_wheel_speed_rad_s for value in signed):
            raise ValueError("wheel angular speed exceeds configured absolute limit")
        speed_mps = 0.5 * (signed[0] + signed[1]) * self._wheel_radius_m
        self._last_stamp = stamp
        if position_pair is not None:
            self._last_positions = position_pair
        else:
            # position 기준시각과 현재 stamp가 어긋나지 않도록 다음 position을 재기준화한다.
            self._last_positions = None
        return speed_mps

    def _joint_indices(self, names):
        try:
            names = tuple(names)
        except TypeError as error:
            raise ValueError("joint names must be iterable") from error
        if len(names) != len(set(names)):
            raise ValueError("joint names must be unique")
        try:
            return names.index(self._left_joint_name), names.index(self._right_joint_name)
        except ValueError as error:
            raise ValueError("required wheel joint name is missing") from error

    @staticmethod
    def _optional_pair(values, names, indices, label):
        try:
            values = tuple(values)
        except TypeError as error:
            raise ValueError("joint {} must be iterable".format(label)) from error
        if not values:
            return None
        if len(values) != len(tuple(names)):
            raise ValueError("joint {} length must match name length".format(label))
        pair = values[indices[0]], values[indices[1]]
        if not all(_finite_number(value) for value in pair):
            raise ValueError("wheel {} values must be finite".format(label))
        return pair


def _finite_number(value):
    if isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except TypeError:
        return False
