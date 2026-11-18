"""Deterministic assumption stress cases, NOT probabilities or product validation."""

from .model import simulate, validate


def compare_assumptions(overrides):
    c = validate(overrides)
    cases = [
        ("현재 가정", {}),
        ("선회 효과 50%", {"turn_gain_rad_s_m": c["turn_gain_rad_s_m"] * 0.5}),
        ("선회 효과 없음", {"turn_gain_rad_s_m": 0}),
        ("선회 방향 반대", {"turn_gain_rad_s_m": -c["turn_gain_rad_s_m"]}),
        (
            "모터 성능 50% 가정",
            {
                "no_load_speed_deg_s": max(1, c["no_load_speed_deg_s"] * 0.5),
                "stall_torque_nm": max(0.001, c["stall_torque_nm"] * 0.5),
            },
        ),
        ("명령 지연 0.2초", {"servo_latency_s": max(0.2, c["servo_latency_s"])}),
        ("조종줄에 전체 하중 가정", {"control_load_fraction": 1}),
        (
            "센서 오차 증가",
            {
                "gps_position_std_m": max(5, c["gps_position_std_m"]),
                "heading_std_deg": max(5, c["heading_std_deg"]),
            },
        ),
    ]
    rows = []
    for label, delta in cases:
        try:
            result = simulate({**c, **delta})
            s = result["summary"]
            rows.append(
                dict(
                    label=label,
                    overrides=delta,
                    status="계산 완료",
                    error_m=s["target_error_m"],
                    landed=s["landed"],
                    brake_mm=max(s["max_left_brake_mm"], s["max_right_brake_mm"]),
                    torque_nm=s["max_torque_nm"],
                    saturation_fraction=s["saturated_control_fraction"],
                    validity_exceeded=result["model_assessment"]["validity_exceeded"],
                )
            )
        except ValueError as e:
            rows.append(
                dict(
                    label=label,
                    overrides=delta,
                    status="모델 유효 범위 이탈",
                    detail=str(e),
                )
            )
    return dict(
        config=c,
        rows=rows,
        note="가정 민감도 비교. 확률 분포·실제 성공률이 아니며 50% 모터 성능은 전압 환산값이 아닙니다.",
    )
