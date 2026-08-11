"""timestamp가 있는 평면 Ackermann wheel odometry를 적분한다."""

from dataclasses import dataclass
import math


class NonMonotonicTimeError(ValueError):
    """odometry sample의 시간이 앞으로 진행하지 않을 때 발생한다."""


@dataclass(frozen=True)
class Pose2DState:
    """역할:
    Ackermann 적분 결과인 평면 위치, 속도, 시각을 하나로 보관한다.

    동작 방식:
    ``AckermannIntegrator``가 계산한 x, y, yaw, 선속도와 yaw rate를 변경 불가능한
    값으로 묶어 ROS adapter에 전달한다.
    """

    x: float
    y: float
    yaw: float
    linear_speed: float
    yaw_rate: float
    stamp: float


class AckermannIntegrator:
    """역할:
    차량 속도와 앞바퀴 조향각으로 rear-axle 기준 2D wheel odometry를 계산한다.

    동작 방식:
    source timestamp가 증가하는 입력만 받아 Ackermann 운동학의 yaw rate를 구하고,
    midpoint heading으로 x, y, yaw를 시간 적분해 ``Pose2DState``를 반환한다.
    """

    def __init__(self, wheelbase_m, max_abs_steering_rad, max_abs_speed_mps):
        for name, value in (
            ("wheelbase_m", wheelbase_m),
            ("max_abs_steering_rad", max_abs_steering_rad),
            ("max_abs_speed_mps", max_abs_speed_mps),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError("{} must be positive and finite".format(name))

        self._wheelbase_m = wheelbase_m
        self._max_abs_steering_rad = max_abs_steering_rad
        self._max_abs_speed_mps = max_abs_speed_mps
        self.reset()

    def reset(self):
        """평면 원점에서 결정적인 source-time epoch를 시작한다."""
        self._state = Pose2DState(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self._last_stamp = None

    def update(self, stamp, speed_mps, steering_rad):
        """시간이 확실히 증가한 차량 sample의 적분 pose를 반환한다."""
        for name, value in (
            ("stamp", stamp),
            ("speed_mps", speed_mps),
            ("steering_rad", steering_rad),
        ):
            if not math.isfinite(value):
                raise ValueError("{} must be finite".format(name))
        if abs(speed_mps) > self._max_abs_speed_mps:
            raise ValueError("speed_mps exceeds configured absolute limit")
        if abs(steering_rad) > self._max_abs_steering_rad:
            raise ValueError("steering_rad exceeds configured absolute limit")
        if self._last_stamp is not None and stamp <= self._last_stamp:
            raise NonMonotonicTimeError("odometry stamps must strictly increase")

        yaw_rate = speed_mps * math.tan(steering_rad) / self._wheelbase_m
        if self._last_stamp is None:
            self._state = Pose2DState(
                self._state.x,
                self._state.y,
                self._state.yaw,
                speed_mps,
                yaw_rate,
                stamp,
            )
            self._last_stamp = stamp
            return self._state

        dt = stamp - self._last_stamp
        midpoint_yaw = self._state.yaw + 0.5 * yaw_rate * dt
        yaw = self._normalize_yaw(self._state.yaw + yaw_rate * dt)
        self._state = Pose2DState(
            self._state.x + speed_mps * dt * math.cos(midpoint_yaw),
            self._state.y + speed_mps * dt * math.sin(midpoint_yaw),
            yaw,
            speed_mps,
            yaw_rate,
            stamp,
        )
        self._last_stamp = stamp
        return self._state

    @staticmethod
    def _normalize_yaw(yaw):
        return math.atan2(math.sin(yaw), math.cos(yaw))
