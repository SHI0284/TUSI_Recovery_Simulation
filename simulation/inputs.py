"""Read supplied RASP thrust curves and saved OpenRocket flight data (SI)."""

from bisect import bisect_right
from dataclasses import dataclass
from pathlib import Path
import math
import xml.etree.ElementTree as ET
import zipfile
from .geometry import external_geometry

DATA = Path(__file__).parent / "data"


@dataclass
class MotorCurve:
    name: str
    propellant_kg: float
    total_kg: float
    points: list

    @property
    def impulse_ns(self):
        return sum(
            (b[0] - a[0]) * (a[1] + b[1]) / 2
            for a, b in zip(self.points, self.points[1:])
        )

    def thrust(self, t):
        if t < 0 or t >= self.points[-1][0]:
            return 0.0
        i = bisect_right(self.points, (t, math.inf))
        a, b = self.points[max(0, i - 1)], self.points[min(i, len(self.points) - 1)]
        return (
            a[1] if a[0] == b[0] else a[1] + (b[1] - a[1]) * (t - a[0]) / (b[0] - a[0])
        )


def read_eng(path):
    lines = [
        s.strip()
        for s in Path(path).read_text().splitlines()
        if s.strip() and not s.lstrip().startswith(";")
    ]
    header = lines[0].split()
    if len(header) != 7:
        raise ValueError("Expected one RASP motor with a seven-field header")
    points = [tuple(map(float, s.split())) for s in lines[1:]]
    if len(points) < 2 or any(
        len(p) != 2 or not all(math.isfinite(v) for v in p) or p[0] < 0 or p[1] < 0
        for p in points
    ):
        raise ValueError("Invalid thrust samples")
    if any(b[0] <= a[0] for a, b in zip(points, points[1:])):
        raise ValueError("Thrust times must strictly increase")
    if points[0][0] > 0:
        points.insert(0, (0.0, 0.0))
    if points[-1][1] != 0:
        raise ValueError("Motor curve must end at zero thrust")
    propellant, total = float(header[4]), float(header[5])
    if not 0 < propellant < total:
        raise ValueError("Invalid motor masses")
    return MotorCurve(header[0], propellant, total, points)


def read_ork(path=DATA / "rocket.ork"):
    with zipfile.ZipFile(path) as archive:
        root = ET.fromstring(archive.read("rocket.ork"))
    simulation = root.find("./simulations/simulation")
    branch = simulation.find("./flightdata/databranch")
    names = branch.get("types").split(",")
    rows = [
        dict(zip(names, map(float, p.text.split(","))))
        for p in branch.findall("datapoint")
    ]
    apogee = max(rows, key=lambda r: r["Altitude"])
    # Use saved calculated mass: summing overrides would miss geometry-based mass.
    dry_mass = rows[0]["Mass"] - rows[0]["Motor mass"]
    canopies = [
        {c.tag: c.text for c in n if c.tag != "subcomponents"}
        for n in root.findall(".//parachute")
    ]
    radius = float(root.find(".//bodytube/radius").text.split()[-1])
    return {
        "geometry": external_geometry(root),
        "creator": root.get("creator"),
        "simulation": simulation.findtext("name"),
        "saved_status": simulation.get("status"),
        "dry_airframe_kg": dry_mass,
        "saved_launch_mass_kg": rows[0]["Mass"],
        "saved_recovery_mass_kg": apogee["Mass"],
        "body_diameter_m": 2 * radius,
        "reference_area_m2": math.pi * radius**2,
        "saved_apogee_m": apogee["Altitude"],
        "saved_apogee_time_s": apogee["Time"],
        "saved_summary": dict(simulation.find("flightdata").attrib),
        "canopies": canopies,
        "events": [dict(e.attrib) for e in branch.findall("event")],
        "rows": rows,
        "apogee": apogee,
    }


def source_summary():
    rocket = read_ork()
    rocket.pop("rows")
    rocket.pop("apogee")
    rocket["curves"] = {}
    for name in ("measured", "theoretical"):
        c = read_eng(DATA / (name + ".eng"))
        rocket["curves"][name] = {
            "impulse_ns": c.impulse_ns,
            "duration_s": c.points[-1][0],
            "peak_n": max(p[1] for p in c.points),
            "motor_total_kg": c.total_kg,
            "propellant_kg": c.propellant_kg,
        }
    rocket["theoretical_duplicate_identical"] = (
        DATA / "theoretical.eng"
    ).read_bytes() == (DATA / "theoretical_duplicate.eng").read_bytes()
    return rocket
