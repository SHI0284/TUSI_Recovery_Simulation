"""회수 비행의 정상 활공 초기조건. 양력·항력 계수는 보정 입력이다."""

import math
from .constants import G


def trim(c, mass):
    gamma = -math.atan2(c["cd"], c["cl"])
    speed = math.sqrt(
        2
        * mass
        * G
        / (c["rho_kg_m3"] * c["canopy_area_m2"] * math.hypot(c["cl"], c["cd"]))
    )
    return speed, gamma


def flight_derivative(
    state,
    sample_time_s,
    *,
    config,
    mass_kg,
    lift_coefficient,
    drag_coefficient,
    bank_rad,
    steering_yaw_rate_rad_s,
    wind,
):
    """[속도, 경로각, 공기상대 방위, 동, 북, 고도]의 시간 미분.

    state 단위: [m/s, rad, rad, m, m, m]. 바람 물질미분은 공기상대
    속도에 적용한다. 기체의 강체 회전 운동방정식은 별도 해석하지 않는다.
    """
    speed, flight_path, heading, east, north, altitude = state
    dynamic_pressure_area = (
        0.5 * config["rho_kg_m3"] * speed * speed * config["canopy_area_m2"]
    )
    lift = dynamic_pressure_area * lift_coefficient
    drag = dynamic_pressure_area * drag_coefficient
    wind_east, wind_north, _, _ = wind.at(sample_time_s, altitude)
    acceleration_east, acceleration_north = wind.acceleration(
        sample_time_s, altitude, speed * math.sin(flight_path)
    )
    along = (
        acceleration_east * math.sin(heading) + acceleration_north * math.cos(heading)
    ) * config["wind_inertia_enabled"]
    cross = (
        acceleration_east * math.cos(heading) - acceleration_north * math.sin(heading)
    ) * config["wind_inertia_enabled"]
    return [
        -drag / mass_kg - G * math.sin(flight_path) - math.cos(flight_path) * along,
        (
            lift * math.cos(bank_rad) / mass_kg
            - G * math.cos(flight_path)
            + math.sin(flight_path) * along
        )
        / max(speed, 0.5),
        steering_yaw_rate_rad_s - cross / max(0.5, speed * math.cos(flight_path)),
        speed * math.cos(flight_path) * math.sin(heading) + wind_east,
        speed * math.cos(flight_path) * math.cos(heading) + wind_north,
        speed * math.sin(flight_path),
    ]
