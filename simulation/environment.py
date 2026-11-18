"""Reproducible synthetic wind and asynchronous virtual navigation sensors.

These are configurable error models, not specifications of the installed sensors.
Heading is an assumed attitude-estimator output; raw IMU fusion is not simulated.
"""

from collections import deque
import math
import random


class WindField:
    def __init__(self, c):
        self.c = c
        rng = random.Random(int(c["random_seed"]))
        a = math.exp(-0.1 / c["gust_time_constant_s"])
        sigma = c["gust_std_mps"] * math.sqrt(1 - a * a)
        v = [0.0, 0.0]
        self.samples = [tuple(v)]
        for _ in range(int(c["duration_s"] / 0.1) + 3):
            v = [a * x + sigma * rng.gauss(0, 1) for x in v]
            self.samples.append(tuple(v))

    def at(self, t, altitude):
        idx = max(0, min(len(self.samples) - 2, int(t / 0.1)))
        f = max(0, min(1, t / 0.1 - idx))
        gust = [
            a + (b - a) * f for a, b in zip(self.samples[idx], self.samples[idx + 1])
        ]
        z = max(0, altitude) / 100
        return (
            self.c["wind_east_mps"] + self.c["wind_shear_east_mps_100m"] * z + gust[0],
            self.c["wind_north_mps"]
            + self.c["wind_shear_north_mps_100m"] * z
            + gust[1],
            gust[0],
            gust[1],
        )

    def acceleration(self, t, altitude, vertical_velocity):
        """Material derivative dW/dt = partial_t W + vertical_speed * partial_h W.

        Fixed 0.1 s piecewise-linear gust samples, no horizontal spatial gradients.
        """
        idx = max(0, min(len(self.samples) - 2, int(t / 0.1)))
        derivative = [
            (b - a) / 0.1 for a, b in zip(self.samples[idx], self.samples[idx + 1])
        ]
        return tuple(
            (
                derivative[i] + self.c[key] * vertical_velocity / 100
                if altitude > 0
                else derivative[i]
            )
            for i, key in enumerate(
                ("wind_shear_east_mps_100m", "wind_shear_north_mps_100m")
            )
        )


class Sensors:
    def __init__(self, c):
        self.c = c
        self.rng = random.Random(int(c["random_seed"]) + 104729)
        self.bias_rng = random.Random(int(c["random_seed"]) + 700001)
        self.gps_bias = [0.0, 0.0]
        self.next = dict(gps=0.0, imu=0.0, baro=0.0)
        self.queues = {k: deque() for k in self.next}
        self.latest = {k: None for k in self.next}
        self.counts = dict(gps=0, imu=0, baro=0)

    def update(self, t, east, north, altitude, heading_rad, yaw_rad_s, ve, vn):
        c, rng = self.c, self.rng
        for kind in self.next:
            if t + 1e-9 >= self.next[kind]:
                self.next[kind] += c[kind + "_period_s"]
                if kind == "gps":
                    blend = math.exp(-c["gps_period_s"] / c["gps_bias_time_constant_s"])
                    self.gps_bias = [
                        blend * x
                        + c["gps_bias_std_m"]
                        * math.sqrt(1 - blend * blend)
                        * self.bias_rng.gauss(0, 1)
                        for x in self.gps_bias
                    ]
                dropout = (
                    kind == "gps"
                    and c["gps_dropout_start_s"] <= t < c["gps_dropout_end_s"]
                )
                if not dropout:
                    if kind == "gps":
                        packet = {
                            "east_m": east
                            + self.gps_bias[0]
                            + rng.gauss(0, c["gps_position_std_m"]),
                            "north_m": north
                            + self.gps_bias[1]
                            + rng.gauss(0, c["gps_position_std_m"]),
                            "ve_mps": ve + rng.gauss(0, c["gps_velocity_std_mps"]),
                            "vn_mps": vn + rng.gauss(0, c["gps_velocity_std_mps"]),
                        }
                    elif kind == "imu":
                        packet = {
                            "heading_rad": heading_rad
                            + math.radians(
                                c["heading_bias_deg"]
                                + rng.gauss(0, c["heading_std_deg"])
                            ),
                            "yaw_rad_s": yaw_rad_s
                            + math.radians(
                                c["gyro_bias_deg_s"] + rng.gauss(0, c["gyro_std_deg_s"])
                            ),
                        }
                    else:
                        packet = {
                            "altitude_m": altitude + rng.gauss(0, c["baro_std_m"])
                        }
                    packet["sample_time_s"] = t
                    self.queues[kind].append((t + c[kind + "_latency_s"], packet))
                    self.counts[kind] += 1
            while self.queues[kind] and self.queues[kind][0][0] <= t + 1e-9:
                self.latest[kind] = self.queues[kind].popleft()[1]
        return self.observe(t)

    def observe(self, t):
        out = {}
        for kind, packet in self.latest.items():
            age = t - packet["sample_time_s"] if packet is not None else -1.0
            valid = packet is not None and age <= self.c["sensor_timeout_s"]
            out[kind + "_valid"] = valid
            out[kind + "_age_s"] = age
            out[kind] = packet
        return out
