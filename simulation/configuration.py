"""설정 파일 로딩과 입력 범위 검증. 모든 물리량은 SI 단위를 기본으로 한다."""

from pathlib import Path
import json
import math
from .inputs import DATA

DEFAULTS = json.loads((Path(__file__).parent / "config.json").read_text())
SERVO_REFERENCE = json.loads((DATA / "st3235.json").read_text())


def validate(overrides):
    if not isinstance(overrides, dict):
        raise ValueError("Configuration must be an object")
    unknown = set(overrides) - set(DEFAULTS)
    if unknown:
        raise ValueError("Unknown parameters: " + ", ".join(sorted(unknown)))
    c = {**DEFAULTS, **overrides}
    for k, v in c.items():
        if k in ("mode", "initialization", "rigging"):
            continue
        if (
            isinstance(v, bool)
            or not isinstance(v, (float, int))
            or not math.isfinite(v)
        ):
            raise ValueError(k + " must be finite numeric data")
    if c["mode"] not in ("neutral", "manual", "heading", "target") or c[
        "initialization"
    ] not in ("ork", "measured", "theoretical"):
        raise ValueError("Unknown mode or initialization")
    if c["rigging"] not in ("unknown", "two_line", "brake_lines"):
        raise ValueError("Unknown rigging type")
    ranges = {
        "servo_latency_s": (0, 1),
        "servo_accel_deg_s2": (0, 20000),
        "servo_resolution_deg": (0, 5),
        "body_heading_lag_s": (0, 5),
        "wind_inertia_enabled": (0, 1),
        "gps_bias_std_m": (0, 20),
        "gps_bias_time_constant_s": (1, 300),
        "gyro_bias_deg_s": (-10, 10),
        "mission_countdown_s": (0, 30),
        "deployment_transition_s": (0.2, 30),
        "landing_hold_s": (0, 10),
        "dt_s": (0.005, 0.05),
        "duration_s": (1, 600),
        "stabilization_height_loss_m": (0, 200),
        "drum_radius_m": (0.001, 0.1),
        "pulley_ratio": (0.1, 10),
        "max_angle_deg": (1, 3600),
        "no_load_speed_deg_s": (1, 3600),
        "stall_torque_nm": (0.001, 100),
        "efficiency": (0.05, 1),
        "servo_supply_voltage_v": (6, 12.6),
        "motor_performance_voltage_v": (6, 12.6),
        "line_slack_m": (0, 0.5),
        "line_stiffness_n_m": (10, 1e7),
        "control_load_fraction": (0, 1),
        "brake_stiffness_n_m": (0, 10000),
        "canopy_area_m2": (0.1, 20),
        "cl": (0.05, 2),
        "cd": (0.05, 3),
        "brake_cl_per_m": (0, 20),
        "brake_cd_per_m": (0, 50),
        "turn_gain_rad_s_m": (-30, 30),
        "turn_time_constant_s": (0.1, 10),
        "max_turn_rate_deg_s": (1, 90),
        "max_bank_deg": (1, 60),
        "max_valid_brake_m": (0.001, 0.5),
        "rocket_cd": (0.1, 2),
        "rho_kg_m3": (0.5, 1.5),
        "controller_period_s": (0.01, 1),
        "heading_kp": (0, 10),
        "yaw_kd_s": (0, 10),
        "wind_east_mps": (-30, 30),
        "wind_north_mps": (-30, 30),
        "initial_heading_deg": (-360, 360),
        "target_heading_deg": (-360, 360),
        "left_command_deg": (0, c["max_angle_deg"]),
        "right_command_deg": (0, c["max_angle_deg"]),
        "command_start_s": (0, 600),
        "command_end_s": (0, 600),
    }
    ranges.update(
        {
            "target_east_m": (-10000, 10000),
            "target_north_m": (-10000, 10000),
            "target_radius_m": (1, 500),
            "navigation_kp_s_inv": (0, 10),
            "navigation_kd": (0, 10),
            "controller_turn_gain_rad_s_m": (-30, 30),
            "control_brake_limit_m": (0.001, 0.2),
            "brake_slew_m_s": (0.001, 1),
            "navigation_airspeed_mps": (0.5, 30),
            "wind_estimator_tau_s": (0.1, 100),
            "wind_compensation": (0, 1),
            "random_seed": (0, 2147483647),
            "gust_std_mps": (0, 10),
            "gust_time_constant_s": (0.2, 60),
            "wind_shear_east_mps_100m": (-5, 5),
            "wind_shear_north_mps_100m": (-5, 5),
            "gps_position_std_m": (0, 50),
            "gps_velocity_std_mps": (0, 5),
            "heading_std_deg": (0, 30),
            "heading_bias_deg": (-180, 180),
            "gyro_std_deg_s": (0, 20),
            "baro_std_m": (0, 30),
            "sensor_timeout_s": (0.1, 20),
            "gps_dropout_start_s": (0, 600),
            "gps_dropout_end_s": (0, 600),
            **{k + "_period_s": (c["dt_s"], 5) for k in ("gps", "imu", "baro")},
            **{k + "_latency_s": (0, 10) for k in ("gps", "imu", "baro")},
        }
    )
    if abs(c["controller_turn_gain_rad_s_m"]) < 0.01:
        raise ValueError("Controller turn calibration must be nonzero")
    if int(c["random_seed"]) != c["random_seed"]:
        raise ValueError("Random seed must be an integer")
    if c["gps_dropout_end_s"] < c["gps_dropout_start_s"]:
        raise ValueError("GPS dropout end must follow start")
    for k, (lo, hi) in ranges.items():
        if not lo <= c[k] <= hi:
            raise ValueError(f"{k} must be between {lo} and {hi}")
    if c["command_end_s"] < c["command_start_s"]:
        raise ValueError("Command end must follow start")
    if c["controller_period_s"] < c["dt_s"]:
        raise ValueError("Controller period must not be smaller than integration step")
    return c
