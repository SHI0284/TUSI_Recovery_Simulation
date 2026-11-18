"""Compose a reviewable launch-to-landing scenario around the recovery model.

The deployment bridge and post-touchdown hold are explicitly illustrative,
not opening/impact dynamics. Existing recovery times and control logs stay intact.
"""

import math
from .inputs import read_ork, read_eng, DATA
from .attitude import axis_fields, ork_attitude, recovery_attitude

LABELS = {
    "PAD": "발사대 대기",
    "BOOST": "점화·동력 상승",
    "COAST": "관성 상승",
    "DEPLOY": "낙하산 전개",
    "STABILIZATION": "자세 안정화",
    "GUIDED_DESCENT": "회수 비행",
    "GROUND": "착지·회수",
    "TIME_LIMIT": "시간 제한 종료",
}


def build_mission(result):
    c = result["config"]
    first = result["trajectory"][0]
    climb = result["ascent"]
    apogee = climb[-1]
    rocket = read_ork()
    countdown = c["mission_countdown_s"]
    duration = c["deployment_transition_s"]
    offset = apogee["t_s"] + duration
    saved = (
        {r["Time"]: r for r in rocket["rows"]} if c["initialization"] == "ork" else {}
    )
    motor = None if saved else read_eng(DATA / (c["initialization"] + ".eng"))
    burnout = (
        next(float(e["time"]) for e in rocket["events"] if e["type"] == "burnout")
        if saved
        else motor.points[-1][0]
    )
    # No invented navigation observations or actuator loads before recovery starts.
    empty = {
        k: (False if isinstance(v, bool) else "" if isinstance(v, str) else 0.0)
        for k, v in first.items()
    }
    for kind in ("gps", "imu", "baro"):
        empty[kind + "_age_s"] = -1.0
    empty.update(estimated_distance_m=-1.0, guidance_status="RECOVERY_NOT_STARTED")

    def row(t, phase, x, y, h, **values):
        return {
            **empty,
            "t_s": t,
            "phase": phase,
            "phase_label": LABELS[phase],
            "east_m": x,
            "north_m": y,
            "altitude_m": max(0, h),
            "target_distance_m": math.hypot(
                c["target_east_m"] - x, c["target_north_m"] - y
            ),
            "recovery_t_s": -1.0,
            "recovery_active": False,
            "canopy_fraction": 0.0,
            "vertical_velocity_mps": 0.0,
            "thrust_n": 0.0,
            "data_source": "SCENARIO",
            "flight_model_valid": False,
            **axis_fields(math.pi / 2, 0, "UPRIGHT_ASSUMPTION"),
            **values,
        }

    prefix = []
    n = max(1, math.ceil(countdown / 0.1))
    if countdown:
        for i in range(n):
            prefix.append(row(-countdown + countdown * i / n, "PAD", 0, 0, 0))
    # Start exactly at ignition even when approximate ascent first sample is > 0.
    ascent_rows = (
        climb
        if climb[0]["t_s"] == 0
        else [{"t_s": 0, "east_m": 0, "north_m": 0, "altitude_m": 0}, *climb]
    )
    for i, a in enumerate(ascent_rows):
        if a["t_s"] >= apogee["t_s"]:
            continue
        phase = "BOOST" if a["t_s"] < burnout else "COAST"
        original = saved.get(a["t_s"])
        if original:
            vz = original["Vertical velocity"]
            thrust = original["Thrust"]
            attitude = ork_attitude(original)
            w = original["Wind velocity"]
            wd = original["Wind direction"]
            attitude.update(
                wind_east_mps=w * math.sin(wd), wind_north_mps=w * math.cos(wd)
            )
        else:
            attitude = axis_fields(math.pi / 2, 0, "VERTICAL_1D_ASSUMPTION")
            b = ascent_rows[min(i + 1, len(ascent_rows) - 1)]
            vz = (b["altitude_m"] - a["altitude_m"]) / max(1e-9, b["t_s"] - a["t_s"])
            thrust = motor.thrust(a["t_s"])
        prefix.append(
            row(
                a["t_s"],
                phase,
                a["east_m"],
                a["north_m"],
                a["altitude_m"],
                vertical_velocity_mps=vz,
                sink_mps=-vz,
                thrust_n=thrust,
                data_source="ORK_SAVED" if saved else "VERTICAL_ASCENT_MODEL",
                flight_model_valid=True,
                **attitude,
            )
        )
    # Monotone cubic height bridge; its prescribed time/loss is not an opening prediction.
    loss = apogee["altitude_m"] - first["altitude_m"]
    end_slope = -min(first["sink_mps"] * duration, 3 * loss)
    n = max(2, math.ceil(duration / 0.05))
    for i in range(n):
        u = i / n
        t = apogee["t_s"] + duration * u
        h = (
            apogee["altitude_m"]
            - loss * (3 * u * u - 2 * u * u * u)
            + end_slope * (u * u * u - u * u)
        )
        vz = (-loss * (6 * u - 6 * u * u) + end_slope * (3 * u * u - 2 * u)) / duration
        inflation = min(1, u / 0.6)
        inflation = inflation * inflation * (3 - 2 * inflation)
        phase = "DEPLOY" if u < 0.6 else "STABILIZATION"
        original = saved.get(apogee["t_s"])
        elevation = (
            original["Vertical orientation (zenith)"] if original else math.pi / 2
        )
        azimuth = original["Lateral orientation (azimuth)"] if original else 0
        attitude = axis_fields(
            elevation + (math.pi / 2 - elevation) * inflation,
            azimuth,
            "ILLUSTRATIVE_ATTITUDE_BLEND",
        )
        prefix.append(
            row(
                t,
                phase,
                apogee["east_m"],
                apogee["north_m"],
                h,
                vertical_velocity_mps=vz,
                sink_mps=-vz,
                canopy_fraction=inflation,
                heading_deg=c["initial_heading_deg"] * inflation,
                data_source="ILLUSTRATIVE_DEPLOYMENT",
                **attitude,
            )
        )

    def event(t, key, label, basis):
        return dict(t_s=t, key=key, label=label, basis=basis)

    events = [
        event(
            -countdown,
            "PAD",
            "발사대 대기",
            "시나리오 대기 시간 · 실제 ARM/점화 명령 없음",
        ),
        event(
            0,
            "BOOST",
            "점화·상승 시작",
            "ORK 저장 비행" if saved else "ENG 추력 기반 수직 상승 모델",
        ),
        event(
            burnout,
            "COAST",
            "추력 종료·관성 상승",
            "ORK burnout 이벤트" if saved else "ENG 추력 곡선 종료",
        ),
        event(
            apogee["t_s"],
            "DEPLOY",
            "최고점·전개 시작",
            "최대 고도 샘플에서 전환 · 전개 애니메이션은 가정",
        ),
        event(
            apogee["t_s"] + 0.6 * duration,
            "STABILIZATION",
            "캐노피 전개·안정화",
            "전개 시간의 60% 지점 · 가정",
        ),
        event(
            offset,
            "GUIDED_DESCENT",
            "회수 제어 시작",
            "가상 센서 유효 시 목표 PD · 모드 설정에 따름",
        ),
    ]
    last = result["trajectory"][-1]
    end = offset + last["t_s"]
    events.append(
        event(
            end,
            "GROUND" if result["summary"]["landed"] else "TIME_LIMIT",
            (
                "착지·결과 확인"
                if result["summary"]["landed"]
                else "미착지·계산 시간 종료"
            ),
            (
                "회수 물리 모델의 지면 교차"
                if result["summary"]["landed"]
                else "설정된 회수 계산 시간 소진"
            ),
        )
    )
    return dict(
        prefix=prefix,
        events=events,
        recovery_start_s=offset,
        touchdown_s=end if result["summary"]["landed"] else None,
        end_s=end + (c["landing_hold_s"] if result["summary"]["landed"] else 0),
        burnout_s=burnout,
        apogee_sample_s=apogee["t_s"],
        ork_apogee_event_s=(
            next(float(e["time"]) for e in rocket["events"] if e["type"] == "apogee")
            if saved
            else None
        ),
        ascent_attitude_summary=dict(
            source="ORK_SAVED_AXIS" if saved else "VERTICAL_1D_ASSUMPTION",
            max_tilt_deg=max(
                r["body_tilt_deg"]
                for r in prefix
                if r["phase"] in ("BOOST", "COAST", "DEPLOY")
            ),
            horizontal_at_apogee_m=math.hypot(apogee["east_m"], apogee["north_m"]),
        ),
        notes=[
            "All mission phases are simulation/replay, not commands to flight hardware.",
            "Deployment time and height loss define an illustrative bridge; opening force, drogue, separation and attitude transients are not solved.",
            "Only recovery runs virtual navigation sensors and PD. Recovery clock, manual pulse and GPS dropout settings start at recovery_start_s.",
            "Countdown and ground hold are scenario displays. Ground uses the last actuator pose without impact or braking dynamics.",
            "No flare, landing-leg deployment, calibrated landing detection, or final-approach controller is implemented.",
        ],
    )


def mission_rows(result):
    """Uniform export rows; the HTTP response sends prefix+offset to avoid duplication."""
    m = result.get("mission") or build_mission(result)
    yield from m["prefix"]
    last = None
    for i, r in enumerate(result["trajectory"]):
        ground = r["altitude_m"] == 0 and result["summary"]["landed"]
        last = {
            **r,
            "t_s": m["recovery_start_s"] + r["t_s"],
            "recovery_t_s": r["t_s"],
            "phase": "GROUND" if ground else "GUIDED_DESCENT",
            "phase_label": LABELS["GROUND" if ground else "GUIDED_DESCENT"],
            "recovery_active": not ground,
            "canopy_fraction": 1.0,
            "vertical_velocity_mps": -r["sink_mps"],
            "thrust_n": 0.0,
            "data_source": "RECOVERY_MODEL",
            "flight_model_valid": True,
            **recovery_attitude(r),
        }
        if ground:
            last.update(guidance_valid=False, guidance_status="TOUCHDOWN_SNAPSHOT")
        if i == len(result["trajectory"]) - 1 and not result["summary"]["landed"]:
            last.update(
                phase="TIME_LIMIT",
                phase_label=LABELS["TIME_LIMIT"],
                recovery_active=False,
                guidance_valid=False,
                guidance_status="SIMULATION_ENDED",
            )
        yield last
    if result["summary"]["landed"] and result["config"]["landing_hold_s"] > 0:
        yield {
            **last,
            "t_s": m["end_s"],
            "vertical_velocity_mps": 0.0,
            "sink_mps": 0.0,
            "airspeed_mps": 0.0,
            "data_source": "ILLUSTRATIVE_GROUND_HOLD",
            "flight_model_valid": False,
        }
