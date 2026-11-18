"""부하·유격·줄 탄성을 포함한 좌우 공통 서보 모델."""

from dataclasses import dataclass, field
from collections import deque
import math
from .constants import clamp


@dataclass
class Actuator:
    """위치 지령 → 수신 지연 → 드럼 회전 → 줄 당김·장력. 내부 각도는 rad."""

    angle_rad: float = 0.0
    velocity_rad_s: float = 0.0
    elapsed_s: float = 0.0
    applied_target_deg: float = 0.0
    last_sent_deg: float | None = None
    pending: deque = field(default_factory=deque)

    def observe(self, c, base_tension_n):
        radius = c["drum_radius_m"]
        stroke = radius * self.angle_rad / c["pulley_ratio"]
        # F = F0 + k_brake*delta; stroke = slack + delta + F/k_line.
        delta = max(
            0.0,
            (stroke - c["line_slack_m"] - base_tension_n / c["line_stiffness_n_m"])
            / (1 + c["brake_stiffness_n_m"] / c["line_stiffness_n_m"]),
        )
        tension = base_tension_n + c["brake_stiffness_n_m"] * delta
        torque = tension * radius / (c["efficiency"] * c["pulley_ratio"])
        return {
            "velocity_deg_s": math.degrees(self.velocity_rad_s),
            "applied_target_deg": self.applied_target_deg,
            "angle_deg": math.degrees(self.angle_rad),
            "takeup_m": radius * self.angle_rad,
            "stroke_m": stroke,
            "brake_m": delta,
            "tension_n": tension,
            "torque_nm": torque,
        }

    def step(self, target_deg, c, base_tension_n, dt):
        target_deg = clamp(target_deg, 0, c["max_angle_deg"])
        quantum = c["servo_resolution_deg"]
        if quantum:
            target_deg = clamp(
                round(target_deg / quantum) * quantum, 0, c["max_angle_deg"]
            )
        if target_deg != self.last_sent_deg:
            self.pending.append((self.elapsed_s + c["servo_latency_s"], target_deg))
            self.last_sent_deg = target_deg
        while self.pending and self.pending[0][0] <= self.elapsed_s + 1e-9:
            self.applied_target_deg = self.pending.popleft()[1]
        obs = self.observe(c, base_tension_n)
        error = math.radians(self.applied_target_deg) - self.angle_rad
        factor = (
            max(0.0, 1 - obs["torque_nm"] / c["stall_torque_nm"]) if error > 0 else 1.0
        )
        vmax = math.radians(c["no_load_speed_deg_s"]) * factor
        acceleration = math.radians(c["servo_accel_deg_s2"])
        if acceleration:
            desired = (
                math.copysign(
                    min(vmax, math.sqrt(2 * acceleration * abs(error))), error
                )
                if error
                else 0.0
            )
            old_velocity = self.velocity_rad_s
            self.velocity_rad_s += clamp(
                desired - self.velocity_rad_s, -acceleration * dt, acceleration * dt
            )
            self.velocity_rad_s = clamp(
                self.velocity_rad_s, -math.radians(c["no_load_speed_deg_s"]), vmax
            )
            movement = (old_velocity + self.velocity_rad_s) * dt / 2
            if error > 0 and factor == 0:
                self.velocity_rad_s = 0
                movement = 0
        else:
            movement = clamp(error, -vmax * dt, vmax * dt)
            self.velocity_rad_s = movement / dt
        if error and movement * error > 0 and abs(movement) >= abs(error):
            movement = error
            self.velocity_rad_s = 0.0
        elif not error:
            movement = 0
            self.velocity_rad_s = 0.0
        self.angle_rad = clamp(
            self.angle_rad + movement, 0, math.radians(c["max_angle_deg"])
        )
        self.elapsed_s += dt
        obs = self.observe(c, base_tension_n)
        obs["stalled"] = error > 1e-6 and factor == 0
        obs["overloaded"] = obs["torque_nm"] >= c["stall_torque_nm"]
        return obs
