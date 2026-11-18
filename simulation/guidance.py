"""Sensor-only target guidance and PD line control, independent of physics/UI.

update(now_s, observation) consumes timestamped virtual or recorded sensor data.
No truth position, true wind, true heading or plant turn gain is read here.
Coordinates: East/North in metres; heading clockwise from North, radians.
This is an engineering prototype, not armed flight firmware.
"""

import math


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class TargetPD:
    def __init__(self, c, assumed_mass_kg):
        self.c = c
        self.mass = assumed_mass_kg
        self.wind_e = self.wind_n = 0.0
        self.last_gps_time = None
        self.last_update = None
        self.last_delta = 0.0
        self.updates = 0

    def brake_to_angle(self, delta):
        if delta <= 0:
            return 0.0
        c = self.c
        # Feedforward mapping uses nominal static load, not instantaneous plant force.
        base = self.mass * 9.80665 * c["control_load_fraction"] / 2
        stroke = (
            c["line_slack_m"]
            + delta
            + (base + c["brake_stiffness_n_m"] * delta) / c["line_stiffness_n_m"]
        )
        return clamp(
            math.degrees(stroke * c["pulley_ratio"] / c["drum_radius_m"]),
            0,
            c["max_angle_deg"],
        )

    def update(self, now, observation):
        c, o = self.c, observation
        dt = (
            c["controller_period_s"]
            if self.last_update is None
            else max(1e-6, now - self.last_update)
        )
        self.last_update = now
        self.updates += 1
        out = dict(
            left_command_deg=0.0,
            right_command_deg=0.0,
            requested_left_brake_m=0.0,
            requested_right_brake_m=0.0,
            target_bearing_deg=0.0,
            desired_heading_deg=0.0,
            heading_error_deg=0.0,
            desired_yaw_rate_deg_s=0.0,
            estimated_distance_m=-1.0,
            p_term_rad_s=0.0,
            d_term_rad_s=0.0,
            estimated_wind_east_mps=self.wind_e,
            estimated_wind_north_mps=self.wind_n,
            guidance_valid=False,
            guidance_status="WAIT_SENSOR",
            command_saturated=False,
            estimated_upwind_unreachable=False,
        )
        if not (o["gps_valid"] and o["imu_valid"] and o["baro_valid"]):
            # Explicit simulation fallback; real-flight loss-of-sensor policy remains TBD.
            self.last_delta = 0.0
            out["guidance_status"] = "SENSOR_STALE_NEUTRAL"
            return out
        gps, imu = o["gps"], o["imu"]
        heading, yaw = imu["heading_rad"], imu["yaw_rad_s"]
        airspeed = c["navigation_airspeed_mps"]
        # GPS velocity - assumed air-relative velocity. Not a true-wind oracle.
        stamp = gps["sample_time_s"]
        if stamp != self.last_gps_time:
            interval = (
                c["gps_period_s"]
                if self.last_gps_time is None
                else max(0, stamp - self.last_gps_time)
            )
            blend = 1 - math.exp(-interval / c["wind_estimator_tau_s"])
            self.wind_e += blend * (
                gps["ve_mps"] - airspeed * math.sin(heading) - self.wind_e
            )
            self.wind_n += blend * (
                gps["vn_mps"] - airspeed * math.cos(heading) - self.wind_n
            )
            self.last_gps_time = stamp
        # Compensate known measurement age using measured GPS velocity.
        east = gps["east_m"] + gps["ve_mps"] * o["gps_age_s"]
        north = gps["north_m"] + gps["vn_mps"] * o["gps_age_s"]
        de, dn = c["target_east_m"] - east, c["target_north_m"] - north
        distance = math.hypot(de, dn)
        bearing = math.atan2(de, dn) if distance > 0.5 else heading
        cross = self.wind_e * math.cos(bearing) - self.wind_n * math.sin(bearing)
        along = self.wind_e * math.sin(bearing) + self.wind_n * math.cos(bearing)
        correction = (
            math.asin(clamp(-cross / airspeed, -0.95, 0.95)) * c["wind_compensation"]
        )
        desired_heading = wrap(bearing + correction)
        error = wrap(desired_heading - heading)
        pterm = c["navigation_kp_s_inv"] * error
        dterm = (
            -c["navigation_kd"] * yaw
        )  # D on measured yaw rate avoids setpoint derivative kicks.
        requested_rate = pterm + dterm
        # Controller's calibration gain is deliberately separate from the plant gain.
        raw_delta = requested_rate / c["controller_turn_gain_rad_s_m"]
        bound = c["control_brake_limit_m"]
        desired_delta = clamp(raw_delta, -bound, bound)
        limit = c["brake_slew_m_s"] * dt
        delta = self.last_delta + clamp(desired_delta - self.last_delta, -limit, limit)
        self.last_delta = delta
        left, right = max(0, -delta), max(0, delta)
        la, ra = self.brake_to_angle(left), self.brake_to_angle(right)
        out.update(
            left_command_deg=la,
            right_command_deg=ra,
            requested_left_brake_m=left,
            requested_right_brake_m=right,
            target_bearing_deg=math.degrees(bearing),
            desired_heading_deg=math.degrees(desired_heading),
            heading_error_deg=math.degrees(error),
            desired_yaw_rate_deg_s=math.degrees(requested_rate),
            estimated_distance_m=distance,
            p_term_rad_s=pterm,
            d_term_rad_s=dterm,
            estimated_wind_east_mps=self.wind_e,
            estimated_wind_north_mps=self.wind_n,
            guidance_valid=True,
            guidance_status="TRACK_TARGET",
            command_saturated=abs(raw_delta) > bound
            or max(la, ra) >= c["max_angle_deg"],
            estimated_upwind_unreachable=abs(cross) >= airspeed
            or along + math.sqrt(max(0, airspeed**2 - cross**2)) <= 0,
        )
        return out
