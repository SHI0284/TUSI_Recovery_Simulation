"""OpenRocket 24.12 axis convention, verified in SimulationStatus.storeData.

Despite the 'zenith' label theta is elevation atan2(z, hypot(x,y)); phi is
atan2(east,north). Saved values are radians. Axial roll is not reconstructed.
"""

import math


def axis_fields(elevation, azimuth, source, measured=False, rates=(0, 0, 0)):
    elevation = max(-math.pi / 2, min(math.pi / 2, elevation))
    horizontal = math.cos(elevation)
    return dict(
        body_axis_east=horizontal * math.sin(azimuth),
        body_axis_north=horizontal * math.cos(azimuth),
        body_axis_up=math.sin(elevation),
        body_tilt_deg=90 - math.degrees(elevation),
        body_azimuth_deg=math.degrees(azimuth) % 360,
        attitude_source=source,
        body_attitude_from_ork=measured,
        angular_rates_available=measured,
        body_roll_rate_deg_s=math.degrees(rates[0]),
        body_pitch_rate_deg_s=math.degrees(rates[1]),
        body_yaw_rate_deg_s=math.degrees(rates[2]),
    )


def ork_attitude(row):
    return axis_fields(
        row["Vertical orientation (zenith)"],
        row["Lateral orientation (azimuth)"],
        "ORK_SAVED_AXIS",
        True,
        tuple(row[k] for k in ("Roll rate", "Pitch rate", "Yaw rate")),
    )


def recovery_attitude(row):
    bank = math.radians(row["bank_deg"])
    heading = math.radians(row["heading_deg"])
    fields = axis_fields(math.pi / 2 - abs(bank), heading, "RECOVERY_BANK_PROXY")
    fields.update(
        body_axis_east=math.sin(bank) * math.cos(heading),
        body_axis_north=-math.sin(bank) * math.sin(heading),
        body_axis_up=math.cos(bank),
        body_azimuth_deg=(
            math.degrees(
                math.atan2(
                    math.sin(bank) * math.cos(heading),
                    -math.sin(bank) * math.sin(heading),
                )
            )
            % 360
            if bank
            else 0.0
        ),
    )
    return fields
