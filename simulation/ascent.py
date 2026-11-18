"""ORK 상승 결과 재생 또는 ENG 추력 기반 1차원 상승 계산."""

from .constants import G, clamp
from .inputs import DATA, read_eng


def ascent(c, rocket):
    if c["initialization"] == "ork":
        a = rocket["apogee"]
        rows = [
            {
                "t_s": r["Time"],
                "altitude_m": r["Altitude"],
                "east_m": r["Position East of launch"],
                "north_m": r["Position North of launch"],
            }
            for r in rocket["rows"]
            if r["Time"] <= a["Time"]
        ]
        return (
            rows,
            rocket["saved_recovery_mass_kg"],
            "Saved OpenRocket flight; its wind and motor are unchanged",
        )
    motor = read_eng(DATA / (c["initialization"] + ".eng"))
    total_impulse = motor.impulse_ns
    dt, t, h, v, impulse = 0.002, 0.0, 0.0, 0.0, 0.0
    rows, airborne = [], False
    for i in range(60000):
        thrust = motor.thrust(t)
        impulse += 0.5 * (thrust + motor.thrust(t + dt)) * dt
        mass = (
            rocket["dry_airframe_kg"]
            + motor.total_kg
            - motor.propellant_kg * clamp(impulse / total_impulse, 0, 1)
        )
        drag = (
            0.5
            * c["rho_kg_m3"]
            * c["rocket_cd"]
            * rocket["reference_area_m2"]
            * v
            * abs(v)
        )
        acceleration = (thrust - drag) / mass - G
        if not airborne and acceleration <= 0:
            v, h = 0.0, 0.0
        else:
            airborne = True
            old_v, old_h = v, h
            v += acceleration * dt
            h += 0.5 * (old_v + v) * dt
            if v <= 0 and t > motor.points[-1][0]:
                fraction = old_v / (old_v - v)
                rows.append(
                    {
                        "t_s": t + dt * fraction,
                        "altitude_m": old_h + 0.5 * old_v * dt * fraction,
                        "east_m": 0.0,
                        "north_m": 0.0,
                    }
                )
                return (
                    rows,
                    mass,
                    "Approximate vertical ascent; constant Cd, impulse-proportional propellant burn, no wind/rail attitude",
                )
        if i % 25 == 0:
            rows.append({"t_s": t + dt, "altitude_m": h, "east_m": 0.0, "north_m": 0.0})
        t += dt
    raise ValueError("No apogee reached with this thrust curve")
