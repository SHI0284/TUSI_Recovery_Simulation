"""Shared SI constants and scalar bounds."""

G = 9.80665  # m/s²


def clamp(value, lower, upper):
    return min(upper, max(lower, value))
