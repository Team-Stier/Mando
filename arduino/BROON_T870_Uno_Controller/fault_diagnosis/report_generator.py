"""Output writers for the T870 rule-based diagnosis result."""

from __future__ import annotations

import csv
from pathlib import Path

from diagnosis import AnalysisResult, Event, SEVERITY_RANK, event_counts


EVENT_COLUMNS = (
    "event_id",
    "code",
    "start_time",
    "end_time",
    "duration_s",
    "severity",
    "subsystem",
    "diagnosis",
    "possible_causes",
    "evidence",
    "confidence_pct",
    "recommended_checks",
)


def write_events_csv(result: AnalysisResult, path: Path) -> None:
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=EVENT_COLUMNS)
        writer.writeheader()
        for event in result.events:
            writer.writerow({column: getattr(event, column) for column in EVENT_COLUMNS})


def _events_by_row(result: AnalysisResult) -> dict[int, list[Event]]:
    mapping: dict[int, list[Event]] = {}
    for event in result.events:
        for row_index in range(event.start_index, event.end_index + 1):
            mapping.setdefault(row_index, []).append(event)
    return mapping


def write_diagnosis_data(result: AnalysisResult, path: Path) -> None:
    derived = (
        "sample_gap_s",
        "seq_gap",
        "tx_dropped_delta",
        "steering_error_adc",
        "steering_target_delta_adc",
        "steering_actual_delta_adc",
        "steering_actual_rate_adc_s",
        "steering_error_reduction_adc",
        "steering_response_direction",
        "controller_reset",
        "diagnostic_event_ids",
        "diagnostic_codes",
        "diagnostic_max_severity",
    )
    fields = list(result.input_columns) + list(derived)
    row_events = _events_by_row(result)
    with path.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for index, row in enumerate(result.rows):
            output = {column: row.get(column, "") for column in result.input_columns}
            for column in derived[:-3]:
                output[column] = row.get(column, "")
            events = row_events.get(index, [])
            output["diagnostic_event_ids"] = ";".join(str(event.event_id) for event in events)
            output["diagnostic_codes"] = ";".join(event.code for event in events)
            output["diagnostic_max_severity"] = (
                max(events, key=lambda event: SEVERITY_RANK[event.severity]).severity
                if events
                else ""
            )
            writer.writerow(output)


def _escape_table(text: object) -> str:
    return str(text).replace("|", "\\|").replace("\n", " ")


def write_summary(result: AnalysisResult, input_path: Path, path: Path, plot_created: bool) -> None:
    rows = result.rows
    complete_count = sum(row["complete"] == 1 for row in rows)
    complete_ratio = complete_count / len(rows) * 100.0
    duration = max(0.0, rows[-1]["_time_s"] - rows[0]["_time_s"])
    counts = event_counts(result.events)
    highest = max((event.severity for event in result.events), key=SEVERITY_RANK.get, default="INFO")
    if highest in {"CRITICAL", "HIGH"}:
        overall = "위험 또는 즉시 점검 필요"
    elif highest == "MEDIUM":
        overall = "주의"
    elif result.events:
        overall = "정보성 이벤트 있음"
    else:
        overall = "검출된 이상 없음"

    lines = [
        "# BROON T870 CAN 로그 자동진단 보고서",
        "",
        f"- 입력 파일: `{input_path}`",
        f"- 분석 행: {len(rows)}개",
        f"- 완전한 CAN 샘플: {complete_count}개 ({complete_ratio:.2f}%)",
        f"- 로그 구간: {rows[0]['_display_time']} ~ {rows[-1]['_display_time']}",
        f"- logger 기준 지속시간: {duration:.3f}초",
        f"- 종합 판정: **{overall}**",
        f"- 이벤트 수: HIGH {counts['HIGH']}, MEDIUM {counts['MEDIUM']}, LOW {counts['LOW']}, INFO {counts['INFO']}",
        f"- 그래프 생성: {'완료' if plot_created else '생략(matplotlib 미설치 또는 --no-plot)'}",
        "",
        "> confidence_pct는 규칙의 데이터 직접성에 따라 정한 설명용 점수이며, 통계적으로 보정된 고장 확률이 아니다.",
        "",
        "## 이벤트 요약",
        "",
    ]
    if result.events:
        lines.extend(
            [
                "| ID | 시작 시각 | 지속시간(s) | 심각도 | 계통 | 진단 |",
                "|---:|---|---:|---|---|---|",
            ]
        )
        for event in result.events:
            lines.append(
                f"| {event.event_id} | {_escape_table(event.start_time)} | {event.duration_s:.3f} | "
                f"{event.severity} | {_escape_table(event.subsystem)} | {_escape_table(event.diagnosis)} |"
            )
    else:
        lines.append("검출된 진단 이벤트가 없다.")

    lines.extend(["", "## 이벤트 상세", ""])
    for event in result.events:
        lines.extend(
            [
                f"### {event.event_id}. {event.diagnosis} (`{event.code}`)",
                "",
                f"- 발생 구간: {event.start_time} ~ {event.end_time}",
                f"- 지속시간: {event.duration_s:.3f}초",
                f"- 심각도: {event.severity}",
                f"- 문제 계통: {event.subsystem}",
                f"- 가능한 원인: {event.possible_causes}",
                f"- 판단 근거: {event.evidence or '추가 수치 없음'}",
                f"- 규칙 신뢰도: {event.confidence_pct}%",
                f"- 권장 점검: {event.recommended_checks}",
                "",
            ]
        )

    lines.extend(["## 데이터 읽기 경고", ""])
    if result.load_warnings:
        lines.extend(f"- {warning}" for warning in result.load_warnings)
    else:
        lines.append("- 없음")

    lines.extend(
        [
            "",
            "## 진단 한계",
            "",
            "이 차량은 전륜 엔코더만 사용하므로 후륜모터 단독 고장을 확정할 수 없다. "
            "배터리 전압, 모터 전류, 온도 데이터도 없으므로 배터리 저전압·과전류·과열은 진단하지 않는다. "
            "보고서의 가능한 원인은 점검 순서를 제시하는 후보이며 부품 고장을 단정하지 않는다.",
            "",
        ]
    )
    path.write_text("\n".join(lines), encoding="utf-8")


def create_plot(result: AnalysisResult, path: Path) -> bool:
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        return False

    valid = [row for row in result.rows if row["complete"] == 1]
    if not valid:
        return False
    origin = valid[0]["_time_s"]
    times = [row["_time_s"] - origin for row in valid]
    figure, axes = plt.subplots(3, 1, figsize=(14, 10), sharex=True)

    axes[0].plot(times, [row["speed_kph"] for row in valid], label="speed_kph")
    axes[0].plot(times, [row["drive_req_pwm"] for row in valid], label="drive_req_pwm", alpha=0.8)
    axes[0].plot(times, [row["front_pwm"] for row in valid], label="front_pwm", alpha=0.6)
    axes[0].plot(times, [row["rear_pwm"] for row in valid], label="rear_pwm", alpha=0.6)
    axes[0].set_ylabel("Speed / PWM")
    axes[0].legend(loc="upper right", ncol=4)
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(times, [row["steer_target_adc"] for row in valid], label="target_adc")
    axes[1].plot(times, [row["steer_actual_adc"] for row in valid], label="actual_adc")
    axes[1].plot(times, [row["steer_pwm"] for row in valid], label="steer_pwm", alpha=0.7)
    axes[1].set_ylabel("Steering")
    axes[1].legend(loc="upper right")
    axes[1].grid(True, alpha=0.3)

    axes[2].step(times, [row["complete"] for row in valid], label="complete")
    axes[2].plot(times, [row["tx_dropped"] for row in valid], label="tx_dropped")
    axes[2].plot(times, [row["can_tec"] for row in valid], label="can_tec")
    axes[2].plot(times, [row["can_rec"] for row in valid], label="can_rec")
    axes[2].set_xlabel("Logger elapsed time (s)")
    axes[2].set_ylabel("CAN quality")
    axes[2].legend(loc="upper right", ncol=4)
    axes[2].grid(True, alpha=0.3)

    colors = {"CRITICAL": "red", "HIGH": "red", "MEDIUM": "orange", "LOW": "gold", "INFO": "steelblue"}
    for event in result.events:
        start = event.start_time_s - origin
        end = max(start + 0.05, event.end_time_s - origin)
        for axis in axes:
            axis.axvspan(start, end, color=colors[event.severity], alpha=0.08)

    figure.suptitle("BROON T870 CAN log diagnosis")
    figure.tight_layout()
    figure.savefig(path, dpi=150)
    plt.close(figure)
    return True
