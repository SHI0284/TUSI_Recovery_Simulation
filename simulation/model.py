"""회수 시뮬레이션 실행기. UI·센서 제어기·액추에이터·환경을 연결한다.

좌표: East/North/Up [m], 방위: 북쪽 기준 시계방향 [rad].
실물 보정 전 축약 모델이며 6자유도/유연체 해석은 아니다.
"""

import math
from .constants import G, clamp
from .configuration import DEFAULTS, SERVO_REFERENCE, validate
from .actuator import Actuator
from .ascent import ascent
from .aerodynamics import trim, flight_derivative
from .inputs import read_ork
from .environment import WindField, Sensors
from .guidance import TargetPD
from .attitude import recovery_attitude


def simulate(overrides=None):
    """센서 → PD → 서보 → 비행 운동을 dt마다 갱신하고 시계열을 반환한다.

    회수 시간 t=0은 캐노피가 이미 안정화된 시점이다.
    발사·전개 화면 연결은 mission.build_mission에서 별도로 구성한다.
    """
    c = validate({} if overrides is None else overrides)
    rocket = read_ork()
    climb, mass, initialization_note = ascent(c, rocket)
    a = climb[-1]
    h = a["altitude_m"] - c["stabilization_height_loss_m"]
    if h <= 0:
        raise ValueError("Stabilization height loss exceeds apogee")
    east, north, psi, yaw = (
        a["east_m"],
        a["north_m"],
        math.radians(c["initial_heading_deg"]),
        0.0,
    )
    body_psi = psi
    speed, gamma = trim(c, mass)
    left, right = Actuator(), Actuator()
    t, next_control, cmd_l, cmd_r = 0.0, 0.0, 0.0, 0.0
    dt = c["dt_s"]
    rows, control_log = [], []
    wind, sensors, controller = WindField(c), Sensors(c), TargetPD(c, mass)
    observation = sensors.observe(0)
    guidance = {}
    warnings = [
        "UNVALIDATED: schematic servo supply is 6.5 V, while available ST3235 torque/speed data are at 12 V. Rig travel, drum, canopy area/CL/CD, line loads and turn gain remain unconfirmed.",
        "ORK main canopy is a 1.3 m round drag device (0.454 kg), not the linked two-line kite aerodynamic model.",
        "Starts in trimmed, fully inflated flight after an assumed height loss; opening, pendulum motion, collapse and stall are not simulated.",
        "Two independent position servos and ideal holding brakes assumed. This standalone repository contains no flight firmware; target PD is a sensor-driven simulation prototype, not deployed firmware.",
        "Recovery mass retains the whole rocket and spent motor. Drogue drag after main deployment is omitted; assumes it is unloaded or released.",
    ]
    warnings.append(
        "Synthetic wind includes gusts and height shear. Optional material wind-acceleration terms preserve ground-velocity inertia in the 3DOF point-mass model. Canopy deformation, local unsteady aerodynamic loads and raw IMU fusion are not resolved. Servo delay/acceleration, noise and sensor biases are unmeasured assumptions."
    )
    warnings.append(
        "CONTROLLABILITY UNCONFIRMED: a two-line stunt kite is not automatically equivalent to a separate-brake parafoil. Turn gain, sign, supported load and free-flight trim require measurements; the rendered deformation is illustrative."
    )
    max_brake, any_overload = 0.0, False

    def sensor_fields():
        o = observation
        gps, imu, baro = o["gps"] or {}, o["imu"] or {}, o["baro"] or {}
        return {
            **{k: v for k, v in o.items() if k.endswith(("_valid", "_age_s"))},
            "gps_east_m": gps.get("east_m", 0),
            "gps_north_m": gps.get("north_m", 0),
            "gps_ve_mps": gps.get("ve_mps", 0),
            "gps_vn_mps": gps.get("vn_mps", 0),
            "gps_course_deg": math.degrees(
                math.atan2(gps.get("ve_mps", 0), gps.get("vn_mps", 0))
            ),
            "sensor_heading_deg": math.degrees(imu.get("heading_rad", 0)),
            "sensor_yaw_rate_deg_s": math.degrees(imu.get("yaw_rad_s", 0)),
            "baro_altitude_m": baro.get("altitude_m", 0),
        }

    def actual_yaw_rate():
        dwe, dwn = wind.acceleration(t, h, speed * math.sin(gamma))
        return yaw - c["wind_inertia_enabled"] * (
            dwe * math.cos(psi) - dwn * math.sin(psi)
        ) / max(0.5, speed * math.cos(gamma))

    def body_rate():
        return (
            ((psi - body_psi + math.pi) % (2 * math.pi) - math.pi)
            / c["body_heading_lag_s"]
            if c["body_heading_lag_s"]
            else actual_yaw_rate()
        )

    def record(l, r, bank, lift, drag):
        we, wn, _, _ = wind.at(t, h)
        return {
            "t_s": t,
            "east_m": east,
            "north_m": north,
            "altitude_m": h,
            "heading_deg": math.degrees(body_psi),
            "air_heading_deg": math.degrees(psi),
            "air_heading_rate_deg_s": math.degrees(actual_yaw_rate()),
            "yaw_rate_deg_s": math.degrees(body_rate()),
            "steering_yaw_rate_deg_s": math.degrees(yaw),
            "airspeed_mps": speed,
            "sink_mps": -speed * math.sin(gamma),
            **recovery_attitude(
                {"bank_deg": math.degrees(bank), "heading_deg": math.degrees(body_psi)}
            ),
            "bank_deg": math.degrees(bank),
            "lift_n": lift,
            "drag_n": drag,
            **sensor_fields(),
            **guidance,
            "control_tick": len(control_log),
            "target_distance_m": math.hypot(
                c["target_east_m"] - east, c["target_north_m"] - north
            ),
            "wind_east_mps": we,
            "wind_north_mps": wn,
            "left_command_deg": cmd_l,
            "right_command_deg": cmd_r,
            **{"left_" + k: v for k, v in l.items()},
            **{"right_" + k: v for k, v in r.items()},
        }

    base = mass * G * c["control_load_fraction"] / 2
    initial_l, initial_r = left.observe(c, base), right.observe(c, base)
    initial_l.update(
        stalled=False, overloaded=initial_l["torque_nm"] >= c["stall_torque_nm"]
    )
    initial_r.update(
        stalled=False, overloaded=initial_r["torque_nm"] >= c["stall_torque_nm"]
    )
    q = 0.5 * c["rho_kg_m3"] * speed**2 * c["canopy_area_m2"]
    guidance = controller.update(0, observation)
    if c["mode"] != "target":
        guidance["guidance_status"] = (
            "IDEAL_HEADING" if c["mode"] == "heading" else "OPEN_LOOP"
        )
    controller.updates = 0
    rows.append(record(initial_l, initial_r, 0.0, q * c["cl"], q * c["cd"]))
    while t < c["duration_s"] - 1e-9 and h > 0:
        step = min(dt, c["duration_s"] - t)
        we, wn, _, _ = wind.at(t, h)
        observation = sensors.update(
            t,
            east,
            north,
            h,
            body_psi,
            body_rate(),
            speed * math.cos(gamma) * math.sin(psi) + we,
            speed * math.cos(gamma) * math.cos(psi) + wn,
        )
        if t + 1e-9 >= next_control:
            if c["mode"] == "manual" and c["command_start_s"] <= t < c["command_end_s"]:
                cmd_l, cmd_r = c["left_command_deg"], c["right_command_deg"]
            elif c["mode"] == "target":
                guidance = controller.update(t, observation)
                cmd_l, cmd_r = (
                    guidance["left_command_deg"],
                    guidance["right_command_deg"],
                )
            elif c["mode"] == "heading":
                error = (math.radians(c["target_heading_deg"]) - body_psi + math.pi) % (
                    2 * math.pi
                ) - math.pi
                u = clamp(c["heading_kp"] * error - c["yaw_kd_s"] * body_rate(), -1, 1)
                cmd_l, cmd_r = (
                    max(0, -u) * c["max_angle_deg"],
                    max(0, u) * c["max_angle_deg"],
                )
            else:
                cmd_l = cmd_r = 0.0
            control_log.append(
                {
                    "t_s": t,
                    **sensor_fields(),
                    **guidance,
                    "left_command_deg": cmd_l,
                    "right_command_deg": cmd_r,
                }
            )
            next_control += c["controller_period_s"]
        l = left.step(cmd_l, c, base, step)
        r = right.step(cmd_r, c, base, step)
        max_brake = max(max_brake, l["brake_m"], r["brake_m"])
        any_overload |= l["overloaded"] or r["overloaded"]
        symmetric = (l["brake_m"] + r["brake_m"]) / 2
        cl = c["cl"] + c["brake_cl_per_m"] * symmetric
        cd = c["cd"] + c["brake_cd_per_m"] * symmetric
        limit = min(
            math.radians(c["max_turn_rate_deg_s"]),
            G
            * math.tan(math.radians(c["max_bank_deg"]))
            / max(0.5, speed * math.cos(gamma)),
        )
        target_yaw = clamp(
            c["turn_gain_rad_s_m"] * (r["brake_m"] - l["brake_m"]), -limit, limit
        )
        yaw += (target_yaw - yaw) * (1 - math.exp(-step / c["turn_time_constant_s"]))
        bank = math.atan(speed * math.cos(gamma) * yaw / G)

        # 한 적분 구간에서는 서보·조향 상태를 고정하고 비행 상태를 RK4로 갱신한다.
        def derivative(state, sample_t):
            return flight_derivative(
                state,
                sample_t,
                config=c,
                mass_kg=mass,
                lift_coefficient=cl,
                drag_coefficient=cd,
                bank_rad=bank,
                steering_yaw_rate_rad_s=yaw,
                wind=wind,
            )

        old = [speed, gamma, psi, east, north, h]
        k1 = derivative(old, t)
        k2 = derivative([s + step * k / 2 for s, k in zip(old, k1)], t + step / 2)
        k3 = derivative([s + step * k / 2 for s, k in zip(old, k2)], t + step / 2)
        k4 = derivative([s + step * k for s, k in zip(old, k3)], t + step)
        new = [
            s + step * (a + 2 * b + 2 * d + e) / 6
            for s, a, b, d, e in zip(old, k1, k2, k3, k4)
        ]
        if (
            not all(math.isfinite(v) for v in new)
            or new[0] < 0.5
            or abs(new[1]) >= math.pi / 2
        ):
            raise ValueError(
                "Model left its valid forward-flight regime; reduce control or revise aerodynamic coefficients"
            )
        if new[5] < 0:
            fraction = old[5] / (old[5] - new[5])
            new = [a + fraction * (b - a) for a, b in zip(old, new)]
            new[5] = 0.0
            step *= fraction
        speed, gamma, psi, east, north, h = new
        if c["body_heading_lag_s"]:
            error = (psi - body_psi + math.pi) % (2 * math.pi) - math.pi
            body_psi += error * (1 - math.exp(-step / c["body_heading_lag_s"]))
        else:
            body_psi = psi
        t += step
        q = 0.5 * c["rho_kg_m3"] * speed**2 * c["canopy_area_m2"]
        lift, drag = q * cl, q * cd
        base = math.hypot(lift, drag) * c["control_load_fraction"] / 2
        observation = sensors.observe(t)
        rows.append(record(l, r, bank, lift, drag))
    if any_overload:
        warnings.append(
            "Motor torque requirement reaches stall torque. Winding may stop; ideal holding assumption cannot predict real backdrive/heating."
        )
    if max_brake > c["max_valid_brake_m"]:
        warnings.append(
            "Brake travel exceeds configured calibration limit; turn/stall response is outside the declared model range."
        )
    if h > 0:
        warnings.append(
            "Time limit reached before touchdown; endpoint is not a landing point."
        )
    end = rows[-1]
    voltage_matches = math.isclose(
        c["servo_supply_voltage_v"], c["motor_performance_voltage_v"], abs_tol=1e-6
    )
    if not voltage_matches:
        warnings.append(
            f"VOLTAGE MISMATCH: schematic/scenario supply {c['servo_supply_voltage_v']} V, torque/speed reference {c['motor_performance_voltage_v']} V. This is a reference-performance scenario, not a prediction at the specified supply. No voltage scaling applied."
        )
    return {
        "config": c,
        "warnings": warnings,
        "initialization_note": initialization_note,
        "servo_reference": SERVO_REFERENCE,
        "motor_voltage_matches_performance": voltage_matches,
        "uses_reference_torque_speed": c["motor_performance_voltage_v"]
        == SERVO_REFERENCE["reference_voltage_v"]
        and all(
            c[k] == SERVO_REFERENCE[k]
            for k in ("stall_torque_nm", "no_load_speed_deg_s")
        ),
        "rocket_geometry": rocket["geometry"],
        "model_assessment": {
            "calibrated_for_this_canopy": False,
            "rigging": c["rigging"],
            "motor_dynamic_parameters_measured": False,
            "uses_wind_inertia": bool(c["wind_inertia_enabled"]),
            "canopy_response": "Empirical differential-line turn law; two-line kite steering/loads and free-flight stability are not validated.",
            "validity_exceeded": max_brake > c["max_valid_brake_m"],
        },
        "ascent": climb,
        "trajectory": rows,
        "control_log": control_log,
        "sensor_counts": sensors.counts,
        "summary": {
            "recovery_mass_kg": mass,
            "apogee_m": a["altitude_m"],
            "apogee_time_s": a["t_s"],
            "start_altitude_m": rows[0]["altitude_m"],
            "descent_time_s": t,
            "landed": h == 0,
            "east_m": east,
            "north_m": north,
            "target_error_m": end["target_distance_m"],
            "closest_target_distance_m": min(r["target_distance_m"] for r in rows),
            "landed_in_target": h == 0
            and end["target_distance_m"] <= c["target_radius_m"],
            "control_updates": len(control_log),
            "valid_guidance_updates": sum(r["guidance_valid"] for r in control_log),
            "saturated_control_fraction": sum(
                r["command_saturated"] for r in control_log
            )
            / max(1, len(control_log)),
            "horizontal_displacement_m": math.hypot(
                east - a["east_m"], north - a["north_m"]
            ),
            "max_left_brake_mm": max(r["left_brake_m"] for r in rows) * 1000,
            "max_right_brake_mm": max(r["right_brake_m"] for r in rows) * 1000,
            "max_torque_nm": max(
                max(r["left_torque_nm"], r["right_torque_nm"]) for r in rows
            ),
            "max_yaw_rate_deg_s": max(abs(r["yaw_rate_deg_s"]) for r in rows),
            "heading_change_deg": end["heading_deg"] - rows[0]["heading_deg"],
        },
    }
