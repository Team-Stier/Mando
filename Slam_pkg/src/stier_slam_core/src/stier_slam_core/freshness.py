"""rospy 의존성 없이 결정적인 ROS-time 센서 freshness 정책을 제공한다."""

from dataclasses import dataclass
import math


OK = "OK"
STALE = "STALE"
MISSING = "MISSING"
FUTURE = "FUTURE"
TIME_REVERSED = "TIME_REVERSED"


@dataclass(frozen=True)
class SensorConfig:
    """역할:
    센서 계약별 timeout과 Header timestamp 사용 여부를 정의한다.

    동작 방식:
    ``FreshnessMonitor``가 source age와 receipt age 중 무엇을 검사할지 결정할 때
    사용하는 변경 불가능한 설정값을 제공한다.
    """

    timeout_sec: float
    has_header_stamp: bool


@dataclass(frozen=True)
class HealthState:
    """역할:
    센서의 최신 상태를 ROS ``DiagnosticStatus``로 변환할 수 있게 표현한다.

    동작 방식:
    상태와 원인, source/receipt age, 관측 주기 및 마지막 시각을 한 결과 객체에 담아
    health node로 전달한다.
    """

    state: str
    reason: str
    source_age_sec: float
    receipt_age_sec: float
    rate_hz: float
    last_header_stamp: float
    last_receipt_time: float


@dataclass
class _SensorState:
    """역할:
    한 센서의 직전 timestamp와 추정 주기를 monitor 내부에 누적한다.

    동작 방식:
    새 메시지가 도착할 때마다 header/receipt 시각과 rate를 갱신하고, 시간 역행이
    감지되면 그 원인을 보존해 다음 health 평가에서 사용한다.
    """

    last_header_stamp: float = None
    last_receipt_time: float = None
    rate_hz: float = None
    time_reversed_reason: str = None


class FreshnessMonitor:
    """역할:
    이름이 지정된 센서 계약별 source time과 receipt time의 신선도를 감시한다.

    동작 방식:
    메시지 도착 시 timestamp 순서와 주기를 기록하고, 평가 시각을 기준으로 timeout,
    미래 timestamp 및 시간 역행을 검사해 각 센서의 ``HealthState``를 만든다.
    """

    def __init__(self, sensors, future_tolerance_sec):
        if not isinstance(sensors, dict) or not sensors:
            raise ValueError("sensors must be a non-empty dict")
        if not _finite_nonnegative(future_tolerance_sec):
            raise ValueError("future_tolerance_sec must be finite and non-negative")

        self._configs = {}
        self._states = {}
        for name, config in sensors.items():
            if not isinstance(name, str) or not name:
                raise ValueError("sensor names must be non-empty strings")
            if not isinstance(config, SensorConfig):
                raise ValueError("sensor config must be SensorConfig")
            if (not _finite_positive(config.timeout_sec)
                    or not isinstance(config.has_header_stamp, bool)):
                raise ValueError("sensor config is invalid")
            self._configs[name] = config
            self._states[name] = _SensorState()
        self._future_tolerance_sec = float(future_tolerance_sec)
        self._last_evaluation_time = None

    def update(self, name, header_stamp, receipt_time):
        """원본 timestamp를 정확히 보존하면서 검증된 sample 하나를 받아들인다."""
        config = self._config_for(name)
        _validate_receipt_time(receipt_time)
        if config.has_header_stamp:
            _validate_header_stamp(header_stamp)
        elif header_stamp is not None:
            raise ValueError("unstamped sensor must not receive a header stamp")

        state = self._states[name]
        if config.has_header_stamp and state.last_header_stamp is not None:
            if header_stamp <= state.last_header_stamp:
                state.time_reversed_reason = "header_time_reversed"
                return
        if state.last_receipt_time is not None and receipt_time < state.last_receipt_time:
            state.time_reversed_reason = "receipt_time_reversed"
            return
        if not config.has_header_stamp and state.last_receipt_time is not None:
            if receipt_time < state.last_receipt_time:
                state.time_reversed_reason = "receipt_time_reversed"
                return
            if receipt_time == state.last_receipt_time:
                return

        rate_hz = _rate_hz(
            header_stamp if config.has_header_stamp else receipt_time,
            state.last_header_stamp if config.has_header_stamp else state.last_receipt_time,
        )
        state.last_header_stamp = header_stamp if config.has_header_stamp else None
        state.last_receipt_time = receipt_time
        if rate_hz is not None:
            state.rate_hz = rate_hz
        state.time_reversed_reason = None

    def evaluate(self, now):
        """ROS time domain을 사용해 설정된 센서 순서대로 모든 health state를 반환한다."""
        _validate_receipt_time(now)
        if self._last_evaluation_time is not None and now < self._last_evaluation_time:
            for state in self._states.values():
                state.last_header_stamp = None
                state.last_receipt_time = None
                state.rate_hz = None
                state.time_reversed_reason = "evaluation_time_reversed"
        self._last_evaluation_time = now

        return {
            name: self._health_for(config, self._states[name], now)
            for name, config in self._configs.items()
        }

    def _config_for(self, name):
        try:
            return self._configs[name]
        except KeyError:
            raise KeyError("unknown sensor: {}".format(name)) from None

    def _health_for(self, config, state, now):
        if state.time_reversed_reason is not None:
            return _health(
                TIME_REVERSED,
                state.time_reversed_reason,
                state,
                None,
                None,
            )
        if state.last_receipt_time is None:
            return _health(MISSING, "missing", state, None, None)

        receipt_age_sec = now - state.last_receipt_time
        source_age_sec = (
            now - state.last_header_stamp if config.has_header_stamp else None
        )
        if receipt_age_sec < -self._future_tolerance_sec:
            return _health(FUTURE, "receipt_time_future", state, source_age_sec, receipt_age_sec)
        if config.has_header_stamp and source_age_sec < -self._future_tolerance_sec:
            return _health(FUTURE, "source_stamp_future", state, source_age_sec, receipt_age_sec)
        if config.has_header_stamp and source_age_sec >= config.timeout_sec:
            return _health(STALE, "source_age_stale", state, source_age_sec, receipt_age_sec)
        if receipt_age_sec >= config.timeout_sec:
            return _health(STALE, "receipt_age_stale", state, source_age_sec, receipt_age_sec)
        return _health(OK, "ok", state, source_age_sec, receipt_age_sec)


def _health(state, reason, sensor_state, source_age_sec, receipt_age_sec):
    return HealthState(
        state=state,
        reason=reason,
        source_age_sec=source_age_sec,
        receipt_age_sec=receipt_age_sec,
        rate_hz=sensor_state.rate_hz,
        last_header_stamp=sensor_state.last_header_stamp,
        last_receipt_time=sensor_state.last_receipt_time,
    )


def _rate_hz(current_time, previous_time):
    if previous_time is None:
        return None
    interval_sec = current_time - previous_time
    if interval_sec <= 0.0 or not math.isfinite(interval_sec):
        return None
    rate_hz = 1.0 / interval_sec
    return rate_hz if math.isfinite(rate_hz) else None


def _validate_header_stamp(value):
    if not _finite_positive(value):
        raise ValueError("header_stamp must be finite and positive")


def _validate_receipt_time(value):
    if not _finite_nonnegative(value):
        raise ValueError("receipt_time must be finite and non-negative")


def _finite_nonnegative(value):
    return _finite_number(value) and value >= 0.0


def _finite_positive(value):
    return _finite_number(value) and value > 0.0


def _finite_number(value):
    return not isinstance(value, bool) and isinstance(value, (int, float)) and math.isfinite(value)
