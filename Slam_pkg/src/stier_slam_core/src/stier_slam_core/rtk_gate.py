"""graph optimization 전에 RTK-GNSS fix를 결정적으로 검증한다."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class RtkFix:
    """역할:
    ``RtkGate``가 ROS 메시지 없이 판정할 수 있도록 RTK 관측값을 보관한다.

    동작 방식:
    source timestamp, WGS84 좌표, XY covariance와 NavSat 상태를 변경 불가능한 값으로
    묶어 gate 입력으로 전달한다.
    """

    stamp: float
    latitude: float
    longitude: float
    altitude: float
    covariance_xy_m2: float
    nav_status: int


@dataclass(frozen=True)
class GateDecision:
    """역할:
    한 번의 RTK gate 판정 결과와 그 이유를 표현한다.

    동작 방식:
    관측값의 수용 여부를 ``accepted``에, 수용 또는 거절 원인을 ``reason``에 담아
    ROS node가 publish와 diagnostic 처리를 결정하게 한다.
    """

    accepted: bool
    reason: str


class RtkGate:
    """역할:
    정확하고 물리적으로 가능한 RTK FIX만 RTAB-Map 입력으로 통과시킨다.

    동작 방식:
    RTK 상태, NavSat 상태, covariance, timestamp 순서와 직전 위치 대비 이동속도를
    검사하고, 모든 조건을 만족한 관측만 기준점으로 저장하며 수용한다.
    """

    def __init__(self, max_covariance_xy_m2, max_speed_mps, innovation_margin_m):
        for name, value in (
                ("max_covariance_xy_m2", max_covariance_xy_m2),
                ("max_speed_mps", max_speed_mps),
                ("innovation_margin_m", innovation_margin_m)):
            if not _is_finite_number(value) or value < 0.0:
                raise ValueError("{} must be finite and non-negative".format(name))
        self._max_covariance_xy_m2 = max_covariance_xy_m2
        self._max_speed_mps = max_speed_mps
        self._innovation_margin_m = innovation_margin_m
        self._previous_accepted_fix = None

    def evaluate(self, fix, rtk_state):
        """판정 결과를 반환하고 ``fix``가 수용된 경우에만 이력을 저장한다."""
        reason = self._rejection_reason(fix, rtk_state)
        if reason is not None:
            return GateDecision(False, reason)
        self._previous_accepted_fix = fix
        return GateDecision(True, "accepted")

    def _rejection_reason(self, fix, rtk_state):
        if rtk_state != "FIX":
            return "rtk_not_fixed"
        if fix.nav_status < 0:
            return "navsat_status_invalid"
        if not all(_is_finite_number(value) for value in (
                fix.stamp, fix.latitude, fix.longitude, fix.altitude)):
            return "fix_not_finite"
        if fix.stamp <= 0.0:
            return "source_stamp_invalid"
        if fix.latitude < -90.0 or fix.latitude > 90.0:
            return "latitude_out_of_bounds"
        if fix.longitude < -180.0 or fix.longitude > 180.0:
            return "longitude_out_of_bounds"
        if not _is_finite_number(fix.covariance_xy_m2):
            return "covariance_not_finite"
        if fix.covariance_xy_m2 < 0.0 or fix.covariance_xy_m2 > self._max_covariance_xy_m2:
            return "covariance_too_large"

        previous = self._previous_accepted_fix
        if previous is None:
            return None
        dt = fix.stamp - previous.stamp
        if dt <= 0.0:
            return "non_monotonic_time"
        innovation_m = _equirectangular_distance_m(previous, fix)
        max_innovation_m = self._max_speed_mps * dt + self._innovation_margin_m
        if innovation_m > max_innovation_m:
            return "innovation_too_large"
        return None


def _equirectangular_distance_m(first, second):
    """RTK innovation gate에 사용할 국소 측지 거리를 근사 계산한다."""
    earth_radius_m = 6371000.0
    latitude_1_rad = math.radians(first.latitude)
    latitude_2_rad = math.radians(second.latitude)
    longitude_delta_rad = math.radians(second.longitude - first.longitude)
    x_m = longitude_delta_rad * math.cos((latitude_1_rad + latitude_2_rad) * 0.5)
    y_m = latitude_2_rad - latitude_1_rad
    return earth_radius_m * math.hypot(x_m, y_m)


def _is_finite_number(value):
    try:
        return math.isfinite(value)
    except TypeError:
        return False
