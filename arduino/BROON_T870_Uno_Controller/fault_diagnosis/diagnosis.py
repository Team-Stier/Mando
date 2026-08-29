"""Rule-based post-drive diagnosis for BROON T870 CAN logger CSV files."""

from __future__ import annotations

import csv
import json
from collections import Counter, deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Iterable


REQUIRED_COLUMNS = (
    "host_time_iso",
    "logger_ms",
    "complete",
    "seq",
    "protocol",
    "state",
    "fault",
    "mode",
    "status_flags",
    "uptime_s",
    "drive_req_pwm",
    "front_pwm",
    "rear_pwm",
    "steer_target_adc",
    "steer_actual_adc",
    "steer_pwm",
    "speed_kph",
    "encoder_delta",
    "encoder_corrected",
    "rc_steer_us",
    "rc_throttle_us",
    "rc_aux_us",
    "rc_flags",
    "rc_read_us",
    "tx_dropped",
    "can_eflg",
    "can_tec",
    "can_rec",
)

INTEGER_COLUMNS = set(REQUIRED_COLUMNS) - {"host_time_iso", "speed_kph"}
SEVERITY_RANK = {"INFO": 0, "LOW": 1, "MEDIUM": 2, "HIGH": 3, "CRITICAL": 4}


@dataclass
class Hit:
    code: str
    severity: str
    subsystem: str
    diagnosis: str
    possible_causes: str
    recommended_checks: str
    confidence_pct: int
    min_duration_s: float
    row_index: int
    time_s: float
    display_time: str
    evidence: dict[str, Any] = field(default_factory=dict)


@dataclass
class Event:
    event_id: int
    code: str
    severity: str
    subsystem: str
    diagnosis: str
    possible_causes: str
    recommended_checks: str
    confidence_pct: int
    start_index: int
    end_index: int
    start_time_s: float
    end_time_s: float
    start_time: str
    end_time: str
    duration_s: float
    evidence: str


@dataclass
class AnalysisResult:
    rows: list[dict[str, Any]]
    events: list[Event]
    load_warnings: list[str]
    input_columns: list[str]


def load_config(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def _as_int(value: str, column: str, line_number: int) -> int:
    try:
        return int(float(value))
    except (TypeError, ValueError) as error:
        raise ValueError(f"line {line_number}: {column} is not an integer: {value!r}") from error


def _as_float(value: str, column: str, line_number: int) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as error:
        raise ValueError(f"line {line_number}: {column} is not numeric: {value!r}") from error


def load_rows(path: Path) -> tuple[list[dict[str, Any]], list[str], list[str]]:
    rows: list[dict[str, Any]] = []
    warnings: list[str] = []
    with path.open("r", encoding="utf-8-sig", newline="") as stream:
        reader = csv.DictReader(stream)
        columns = reader.fieldnames or []
        missing = [column for column in REQUIRED_COLUMNS if column not in columns]
        if missing:
            raise ValueError("missing CSV columns: " + ", ".join(missing))

        for line_number, raw in enumerate(reader, start=2):
            try:
                parsed: dict[str, Any] = dict(raw)
                for column in INTEGER_COLUMNS:
                    parsed[column] = _as_int(raw[column], column, line_number)
                parsed["speed_kph"] = _as_float(raw["speed_kph"], "speed_kph", line_number)
                parsed["_line_number"] = line_number
                parsed["_time_s"] = parsed["logger_ms"] / 1000.0
                parsed["_display_time"] = raw["host_time_iso"] or f"logger_ms={parsed['logger_ms']}"
                rows.append(parsed)
            except ValueError as error:
                warnings.append(str(error))

    if not rows:
        raise ValueError("CSV contains no valid data rows")
    return rows, warnings, columns


def _counter_delta(current: int, previous: int, bits: int) -> int:
    modulus = 1 << bits
    delta = (current - previous) % modulus
    return 0 if delta > modulus // 2 else delta


def add_features(rows: list[dict[str, Any]], config: dict[str, Any]) -> None:
    expected_protocol = config["general"]["expected_protocol"]
    for index, row in enumerate(rows):
        row["sample_gap_s"] = 0.0
        row["seq_gap"] = 0
        row["tx_dropped_delta"] = 0
        row["steering_error_adc"] = row["steer_target_adc"] - row["steer_actual_adc"]
        row["steering_actual_delta_adc"] = 0
        row["controller_reset"] = 0
        row["protocol_valid"] = int(row["protocol"] == expected_protocol)
        if index == 0:
            continue

        previous = rows[index - 1]
        row["sample_gap_s"] = max(0.0, row["_time_s"] - previous["_time_s"])
        sequence_step = (row["seq"] - previous["seq"]) % 256
        row["seq_gap"] = 0 if sequence_step == 1 else 1
        row["steering_actual_delta_adc"] = row["steer_actual_adc"] - previous["steer_actual_adc"]
        reset = row["uptime_s"] + 1 < previous["uptime_s"]
        row["controller_reset"] = int(reset)
        if not reset:
            row["tx_dropped_delta"] = _counter_delta(
                row["tx_dropped"], previous["tx_dropped"], 16
            )


def _hit(
    hits: list[Hit],
    row: dict[str, Any],
    row_index: int,
    *,
    code: str,
    severity: str,
    subsystem: str,
    diagnosis: str,
    possible_causes: str,
    recommended_checks: str,
    confidence_pct: int,
    min_duration_s: float = 0.0,
    evidence: dict[str, Any] | None = None,
) -> None:
    hits.append(
        Hit(
            code=code,
            severity=severity,
            subsystem=subsystem,
            diagnosis=diagnosis,
            possible_causes=possible_causes,
            recommended_checks=recommended_checks,
            confidence_pct=confidence_pct,
            min_duration_s=min_duration_s,
            row_index=row_index,
            time_s=row["_time_s"],
            display_time=row["_display_time"],
            evidence=evidence or {},
        )
    )


FAULTS = {
    1: ("설정", "제어기 설정 오류", "핀맵·교정값·빌드 설정 불일치", "BuildOptions와 ControllerConfig 검토"),
    2: ("조향", "조향 포텐시오미터 오류", "센서 단선·단락·커넥터 이탈", "포텐시오미터 전원·GND·신호선과 ADC 측정"),
    3: ("조향", "조향모터 스톨", "조향기구 걸림·모터/드라이버/배선 이상", "조향기어 걸림과 모터드라이버 출력 점검"),
    4: ("조향", "조향 연속구동시간 초과", "목표각 도달 실패·기구부 저항 증가", "조향 응답과 목표/실제 ADC 점검"),
    5: ("구동", "구동 피드백 없음", "전륜 구동·엔코더·드라이버·배선 이상", "전륜 회전, 엔코더 파형과 드라이버 출력 점검"),
    6: ("구동", "차량 과속", "속도명령/엔코더 환산 오류 또는 비정상 구동", "속도 환산값과 구동 명령 점검"),
}


def evaluate_rules(rows: list[dict[str, Any]], config: dict[str, Any]) -> list[Hit]:
    hits: list[Hit] = []
    steering = config["steering"]
    drive = config["drive"]
    can = config["can"]
    rc = config["rc"]
    expected_protocol = config["general"]["expected_protocol"]
    steering_window: deque[dict[str, Any]] = deque()

    for index, row in enumerate(rows):
        if row["complete"] != 1:
            _hit(
                hits, row, index, code="CAN_INCOMPLETE_SAMPLE", severity="MEDIUM",
                subsystem="CAN", diagnosis="CAN 프레임 세트 누락",
                possible_causes="CAN 배선 노이즈·송신 drop·전원 불안정",
                recommended_checks="complete 비율, 종단저항, tx_dropped와 TEC 확인",
                confidence_pct=98, evidence={"complete": row["complete"]},
            )
        if row["complete"] == 1 and row["protocol"] != expected_protocol:
            _hit(
                hits, row, index, code="CAN_PROTOCOL_MISMATCH", severity="HIGH",
                subsystem="CAN", diagnosis="CAN 프로토콜 버전 불일치",
                possible_causes="제어기와 로거 코드 버전 불일치",
                recommended_checks="T870CanProtocol.h 복사본과 protocol version 확인",
                confidence_pct=99,
                evidence={"protocol": row["protocol"], "expected": expected_protocol},
            )
        if row["seq_gap"]:
            _hit(
                hits, row, index, code="CAN_SEQUENCE_GAP", severity="MEDIUM",
                subsystem="CAN", diagnosis="CAN sequence 연속성 이상",
                possible_causes="프레임 세트 유실·로거 재시작·송신 중단",
                recommended_checks="인접 seq와 logger_ms, 전원 재시작 여부 확인",
                confidence_pct=95, evidence={"seq": row["seq"]},
            )
        if row["sample_gap_s"] > can["sample_gap_warning_s"]:
            _hit(
                hits, row, index, code="CAN_SAMPLE_GAP", severity="MEDIUM",
                subsystem="CAN", diagnosis="CAN 로그 수신 공백",
                possible_causes="CAN 단절·USB 지연·제어기 또는 로거 재시작",
                recommended_checks="CAN 전원·배선과 Ubuntu USB 상태 확인",
                confidence_pct=90, evidence={"sample_gap_s": round(row["sample_gap_s"], 3)},
            )
        if row["tx_dropped_delta"] > 0:
            _hit(
                hits, row, index, code="CAN_TX_DROP", severity="MEDIUM",
                subsystem="CAN", diagnosis="CAN 송신 프레임 누락 증가",
                possible_causes="MCP2515 송신버퍼 포화·ACK 부재·버스 오류",
                recommended_checks="tx_dropped 증가율, TEC, 상대 노드 전원 점검",
                confidence_pct=98, evidence={"tx_dropped_delta": row["tx_dropped_delta"]},
            )
        if row["can_eflg"] != 0:
            _hit(
                hits, row, index, code="CAN_ERROR_FLAG", severity="HIGH",
                subsystem="CAN", diagnosis="MCP2515 CAN 오류 플래그 발생",
                possible_causes="CAN-H/L 단락·극성·종단저항·bitrate 이상",
                recommended_checks="EFLG 비트, CAN-H/L, 60 ohm 종단과 500 kbps 확인",
                confidence_pct=99, evidence={"can_eflg": row["can_eflg"]},
            )
        if row["can_tec"] >= can["tec_warning"]:
            severity = "HIGH" if row["can_tec"] >= can["tec_high"] else "MEDIUM"
            _hit(
                hits, row, index, code="CAN_TEC_HIGH", severity=severity,
                subsystem="CAN", diagnosis="CAN 송신 오류 카운터 상승",
                possible_causes="상대 노드 ACK 부재·배선·종단·bitrate 이상",
                recommended_checks="로거 전원, CAN-H/L, 종단저항과 bitrate 확인",
                confidence_pct=95, evidence={"can_tec": row["can_tec"]},
            )
        if row["can_rec"] >= can["rec_warning"]:
            severity = "HIGH" if row["can_rec"] >= can["rec_high"] else "MEDIUM"
            _hit(
                hits, row, index, code="CAN_REC_HIGH", severity=severity,
                subsystem="CAN", diagnosis="CAN 수신 오류 카운터 상승",
                possible_causes="CAN 신호 품질·배선·종단저항 이상",
                recommended_checks="CAN 파형, 커넥터, 종단저항과 노이즈 경로 확인",
                confidence_pct=90, evidence={"can_rec": row["can_rec"]},
            )
        if row["controller_reset"]:
            _hit(
                hits, row, index, code="CONTROLLER_RESET", severity="HIGH",
                subsystem="제어기", diagnosis="주행 로그 중 제어기 재시작",
                possible_causes="5 V 전원강하·리셋·워치독·USB/전원 불안정",
                recommended_checks="Uno 전원, 레귤레이터, 접지와 리셋 원인 확인",
                confidence_pct=92, evidence={"uptime_s": row["uptime_s"]},
            )

        if row["complete"] != 1 or row["protocol"] != expected_protocol:
            continue

        steering_window.append(row)
        window_start = row["_time_s"] - steering["hunting_window_s"]
        while steering_window and steering_window[0]["_time_s"] < window_start:
            steering_window.popleft()

        fault = row["fault"]
        if fault in FAULTS:
            subsystem, diagnosis, causes, checks = FAULTS[fault]
            _hit(
                hits, row, index, code=f"CONTROLLER_FAULT_{fault}", severity="HIGH",
                subsystem=subsystem, diagnosis=diagnosis, possible_causes=causes,
                recommended_checks=checks, confidence_pct=99,
                evidence={"fault": fault, "state": row["state"]},
            )
        elif fault != 0:
            _hit(
                hits, row, index, code="CONTROLLER_FAULT_UNKNOWN", severity="HIGH",
                subsystem="제어기", diagnosis="정의되지 않은 fault 코드",
                possible_causes="제어기와 진단기 버전 불일치",
                recommended_checks="DBC와 BroonT870Core.h fault 정의 확인",
                confidence_pct=99, evidence={"fault": fault},
            )

        flags = row["status_flags"]
        if flags & (1 << 1):
            _hit(
                hits, row, index, code="IMMEDIATE_STOP", severity="MEDIUM",
                subsystem="안전", diagnosis="즉시 정지 상태 발생",
                possible_causes="RC 원격정지·명령 무효·제어기 fault",
                recommended_checks="동일 시각의 fault, rc_flags와 command_valid 확인",
                confidence_pct=99, evidence={"status_flags": flags},
            )
        if row["uptime_s"] > 2 and not (flags & (1 << 4)):
            _hit(
                hits, row, index, code="CONFIGURATION_INVALID", severity="HIGH",
                subsystem="설정", diagnosis="제어기 설정 유효성 실패",
                possible_causes="조향·RC 교정값 또는 핀 설정 오류",
                recommended_checks="ControllerConfig와 calibration 확인",
                confidence_pct=99, evidence={"status_flags": flags},
            )

        actual_adc = row["steer_actual_adc"]
        pwm_abs = abs(row["steer_pwm"])
        error_abs = abs(row["steering_error_adc"])
        if actual_adc < steering["sensor_min_adc"] or actual_adc > steering["sensor_max_adc"]:
            _hit(
                hits, row, index, code="STEERING_SENSOR_RANGE", severity="HIGH",
                subsystem="조향", diagnosis="조향센서 전기적 범위 이탈",
                possible_causes="포텐시오미터 단선·단락·전원 또는 GND 이상",
                recommended_checks="A4 전압과 포텐시오미터 커넥터 확인",
                confidence_pct=98, evidence={"steer_actual_adc": actual_adc},
            )
        steering_active = pwm_abs >= steering["active_pwm"] and error_abs >= steering["tracking_error_adc"]
        if steering_active:
            _hit(
                hits, row, index, code="STEERING_TRACKING_ERROR", severity="MEDIUM",
                subsystem="조향", diagnosis="조향 목표 추종 오차 지속",
                possible_causes="조향 응답 지연·마찰 증가·게인 또는 기구부 문제",
                recommended_checks="목표/실제 ADC, PWM과 기구부 마찰 확인",
                confidence_pct=75,
                min_duration_s=steering["tracking_error_duration_s"],
                evidence={"error_adc": error_abs, "steer_pwm": row["steer_pwm"]},
            )
            if index > 0 and abs(row["steering_actual_delta_adc"]) <= steering["no_response_delta_adc"]:
                _hit(
                    hits, row, index, code="STEERING_NO_RESPONSE", severity="HIGH",
                    subsystem="조향", diagnosis="조향 PWM 대비 센서 무응답",
                    possible_causes="조향모터 스톨·기어 걸림·드라이버 또는 배선 이상",
                    recommended_checks="모터 출력, 조향기어와 포텐시오미터 체결 확인",
                    confidence_pct=88,
                    min_duration_s=steering["no_response_duration_s"],
                    evidence={
                        "error_adc": error_abs,
                        "steer_pwm": row["steer_pwm"],
                        "actual_delta_adc": row["steering_actual_delta_adc"],
                    },
                )

        window = list(steering_window)
        if len(window) >= 4:
            target_span = max(item["steer_target_adc"] for item in window) - min(
                item["steer_target_adc"] for item in window
            )
            signs = []
            for item in window:
                value = item["steer_pwm"]
                if value != 0:
                    signs.append(1 if value > 0 else -1)
            changes = sum(1 for left, right in zip(signs, signs[1:]) if left != right)
            if target_span <= steering["hunting_target_span_adc"] and changes >= steering["hunting_min_pwm_sign_changes"]:
                _hit(
                    hits, row, index, code="STEERING_HUNTING", severity="MEDIUM",
                    subsystem="조향", diagnosis="조향 헌팅 가능성",
                    possible_causes="데드밴드·제어게인·기계 유격 또는 센서 노이즈",
                    recommended_checks="목표 고정 구간의 PWM 방향전환과 ADC 진동 확인",
                    confidence_pct=72,
                    evidence={"pwm_sign_changes": changes, "target_span_adc": target_span},
                )

        drive_active = max(
            abs(row["drive_req_pwm"]), abs(row["front_pwm"]), abs(row["rear_pwm"])
        ) >= drive["active_pwm"]
        if drive_active and abs(row["speed_kph"]) <= drive["standstill_speed_kph"] and row["encoder_delta"] == 0:
            _hit(
                hits, row, index, code="DRIVE_NO_FEEDBACK", severity="HIGH",
                subsystem="구동", diagnosis="구동 PWM 대비 전륜 피드백 없음",
                possible_causes="전륜 모터·드라이버·배선·엔코더 또는 기계적 구속",
                recommended_checks="전륜 회전, 엔코더 파형과 드라이버 출력을 순서대로 확인",
                confidence_pct=86,
                min_duration_s=drive["no_feedback_duration_s"],
                evidence={
                    "drive_req_pwm": row["drive_req_pwm"],
                    "speed_kph": row["speed_kph"],
                    "encoder_delta": row["encoder_delta"],
                },
            )
        if abs(row["speed_kph"]) > drive["overspeed_kph"]:
            _hit(
                hits, row, index, code="DRIVE_OVERSPEED", severity="HIGH",
                subsystem="구동", diagnosis="차량 과속",
                possible_causes="과도한 명령·엔코더 환산 오류·비정상 구동",
                recommended_checks="속도 환산값, 바퀴 둘레, CPR과 구동 명령 확인",
                confidence_pct=95, evidence={"speed_kph": row["speed_kph"]},
            )
        outputs_zero = max(abs(row["drive_req_pwm"]), abs(row["front_pwm"]), abs(row["rear_pwm"])) <= 5
        if outputs_zero and abs(row["speed_kph"]) >= drive["uncommanded_speed_kph"]:
            _hit(
                hits, row, index, code="DRIVE_UNCOMMANDED_MOTION", severity="HIGH",
                subsystem="구동", diagnosis="구동 명령 없이 속도 감지",
                possible_causes="차량 관성·내리막·출력 상태 불일치·엔코더 노이즈",
                recommended_checks="노면 조건, 모터 출력과 엔코더 신호 확인",
                confidence_pct=75,
                min_duration_s=drive["uncommanded_duration_s"],
                evidence={"speed_kph": row["speed_kph"]},
            )
        mismatch = abs(row["front_pwm"] - row["rear_pwm"])
        if mismatch > drive["front_rear_mismatch_pwm"]:
            _hit(
                hits, row, index, code="DRIVE_PWM_MISMATCH", severity="MEDIUM",
                subsystem="구동", diagnosis="전륜·후륜 PWM 명령 불일치",
                possible_causes="출력 계산·제한·안전 인터록 상태 불일치",
                recommended_checks="동일 시각의 요청/전륜/후륜 PWM과 안전상태 확인",
                confidence_pct=90,
                min_duration_s=drive["mismatch_duration_s"],
                evidence={"front_pwm": row["front_pwm"], "rear_pwm": row["rear_pwm"]},
            )

        rc_flags = row["rc_flags"]
        invalid_channels = [name for bit, name in ((0, "steer"), (1, "throttle"), (2, "aux")) if not (rc_flags & (1 << bit))]
        if invalid_channels:
            _hit(
                hits, row, index, code="RC_SIGNAL_INVALID", severity="MEDIUM",
                subsystem="RC", diagnosis="RC 채널 유효성 상실",
                possible_causes="수신기 전원·신호선·송신기 링크·펄스 timeout",
                recommended_checks="무효 채널의 배선과 수신기 전원, 송신기 상태 확인",
                confidence_pct=98,
                min_duration_s=rc["invalid_duration_s"],
                evidence={"invalid_channels": "/".join(invalid_channels), "rc_flags": rc_flags},
            )
        if rc_flags & (1 << 3):
            _hit(
                hits, row, index, code="RC_REMOTE_STOP", severity="INFO",
                subsystem="RC", diagnosis="RC 원격정지 활성",
                possible_causes="운전자 정지 조작 또는 RC 신호 손실",
                recommended_checks="의도된 정지인지 운전자 기록과 비교",
                confidence_pct=99, evidence={"rc_flags": rc_flags},
            )

        margin = rc["pulse_margin_us"]
        pulse_checks = (
            (0, "rc_steer_us", rc["steer_min_us"], rc["steer_max_us"], "STEER", "조향"),
            (1, "rc_throttle_us", rc["throttle_min_us"], rc["throttle_max_us"], "THROTTLE", "스로틀"),
            (2, "rc_aux_us", rc["aux_min_us"], rc["aux_max_us"], "AUX", "AUX"),
        )
        for bit, column, minimum, maximum, code_label, label in pulse_checks:
            pulse = row[column]
            if rc_flags & (1 << bit) and not (minimum - margin <= pulse <= maximum + margin):
                _hit(
                    hits, row, index, code=f"RC_{code_label}_PULSE_RANGE", severity="MEDIUM",
                    subsystem="RC", diagnosis=f"RC {label} 펄스 범위 이탈",
                    possible_causes="RC 캘리브레이션·송신기 설정·신호 노이즈",
                    recommended_checks=f"{label} 채널 최소/중앙/최대 펄스 재측정",
                    confidence_pct=92, evidence={column: pulse},
                )

    return hits


def _format_evidence(group: list[Hit]) -> str:
    values: dict[str, list[Any]] = {}
    for hit in group:
        for key, value in hit.evidence.items():
            values.setdefault(key, []).append(value)
    parts = []
    for key, items in values.items():
        numeric = [item for item in items if isinstance(item, (int, float))]
        if len(numeric) == len(items) and numeric:
            minimum, maximum = min(numeric), max(numeric)
            text = f"{minimum}" if minimum == maximum else f"{minimum}~{maximum}"
        else:
            text = "/".join(dict.fromkeys(str(item) for item in items))
        parts.append(f"{key}={text}")
    return "; ".join(parts)


def merge_hits(hits: Iterable[Hit], config: dict[str, Any]) -> list[Event]:
    merge_gap = config["general"]["merge_gap_s"]
    grouped: dict[str, list[Hit]] = {}
    for hit in sorted(hits, key=lambda item: (item.code, item.time_s)):
        grouped.setdefault(hit.code, []).append(hit)

    event_groups: list[list[Hit]] = []
    for code_hits in grouped.values():
        current: list[Hit] = []
        for hit in code_hits:
            if current and hit.time_s - current[-1].time_s > merge_gap:
                event_groups.append(current)
                current = []
            current.append(hit)
        if current:
            event_groups.append(current)

    accepted: list[tuple[list[Hit], float]] = []
    for group in event_groups:
        duration = max(0.0, group[-1].time_s - group[0].time_s)
        if duration + 1e-9 >= group[0].min_duration_s:
            accepted.append((group, duration))
    accepted.sort(key=lambda item: (item[0][0].time_s, -SEVERITY_RANK[item[0][0].severity]))

    events: list[Event] = []
    for event_id, (group, duration) in enumerate(accepted, start=1):
        representative = max(group, key=lambda item: SEVERITY_RANK[item.severity])
        events.append(
            Event(
                event_id=event_id,
                code=representative.code,
                severity=representative.severity,
                subsystem=representative.subsystem,
                diagnosis=representative.diagnosis,
                possible_causes=representative.possible_causes,
                recommended_checks=representative.recommended_checks,
                confidence_pct=max(hit.confidence_pct for hit in group),
                start_index=group[0].row_index,
                end_index=group[-1].row_index,
                start_time_s=group[0].time_s,
                end_time_s=group[-1].time_s,
                start_time=group[0].display_time,
                end_time=group[-1].display_time,
                duration_s=round(duration, 3),
                evidence=_format_evidence(group),
            )
        )
    return events


def analyze_csv(input_path: Path, config: dict[str, Any]) -> AnalysisResult:
    rows, warnings, columns = load_rows(input_path)
    add_features(rows, config)
    hits = evaluate_rules(rows, config)
    events = merge_hits(hits, config)
    return AnalysisResult(rows=rows, events=events, load_warnings=warnings, input_columns=columns)


def event_counts(events: Iterable[Event]) -> Counter[str]:
    return Counter(event.severity for event in events)
