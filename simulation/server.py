"""Local, offline UI. Standard library only; never connects to flight hardware."""

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
from pathlib import Path
from .inputs import source_summary
from .mission import build_mission
from .sensitivity import compare_assumptions
from .model import DEFAULTS, SERVO_REFERENCE, simulate, validate

WEB = Path(__file__).parent / "web"


class Handler(BaseHTTPRequestHandler):
    def send(self, status, data, content_type="application/json; charset=utf-8"):
        if not isinstance(data, bytes):
            data = json.dumps(data, ensure_ascii=False, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("X-Content-Type-Options", "nosniff")
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        if self.path == "/api/config":
            self.send(
                200,
                {
                    "config": DEFAULTS,
                    "sources": source_summary(),
                    "servo_reference": SERVO_REFERENCE,
                    "canopy_reference": json.loads(
                        (WEB.parent / "data/parafoil_visual_reference.json").read_text()
                    ),
                },
            )
        elif self.path == "/reference/parafoil.jpg":
            self.send(
                200,
                (
                    WEB.parent / "data/hardware/parafoil_product_reference.jpg"
                ).read_bytes(),
                "image/jpeg",
            )
        elif self.path in (
            "/",
            "/app.js",
            "/style.css",
            "/scene.js",
            "/vendor/three.module.js",
            "/vendor/three.core.js",
            "/vendor/OrbitControls.js",
            "/ui/config-fields.js",
            "/ui/mission-timeline.js",
            "/ui/charts.js",
        ):
            name = "index.html" if self.path == "/" else self.path[1:]
            mime = {".html": "text/html", ".js": "text/javascript", ".css": "text/css"}[
                Path(name).suffix
            ]
            self.send(200, (WEB / name).read_bytes(), mime + "; charset=utf-8")
        else:
            self.send(404, {"error": "Not found"})

    def do_POST(self):
        if self.path not in ("/api/simulate", "/api/sweep", "/api/sensitivity"):
            self.send(404, {"error": "Not found"})
            return
        try:
            if self.headers.get("Content-Type", "").split(";")[0] != "application/json":
                raise ValueError("Use application/json")
            size = int(self.headers.get("Content-Length", 0))
            if size <= 0 or size > 16384:
                raise ValueError("Invalid request size")
            config = json.loads(self.rfile.read(size))
            if self.path == "/api/sensitivity":
                self.send(200, compare_assumptions(config))
                return
            if self.path == "/api/sweep":
                c = validate(config)
                c.update(mode="manual", duration_s=30, left_command_deg=0)
                neutral = simulate({**c, "mode": "neutral"})["summary"]
                rows = []
                angles = sorted(
                    set(
                        [
                            a
                            for a in [0, 45, 90, 135, 180, 225, 270]
                            if a <= c["max_angle_deg"]
                        ]
                        + [min(270, c["max_angle_deg"])]
                    )
                )
                for angle in angles:
                    s = simulate({**c, "right_command_deg": angle})["summary"]
                    rows.append(
                        dict(
                            command_deg=angle,
                            brake_mm=s["max_right_brake_mm"],
                            yaw_deg_s=s["max_yaw_rate_deg_s"],
                            time_s=s["descent_time_s"],
                            landed=s["landed"],
                            delta_east_m=s["east_m"] - neutral["east_m"],
                            delta_north_m=s["north_m"] - neutral["north_m"],
                            displacement_from_neutral_m=math.hypot(
                                s["east_m"] - neutral["east_m"],
                                s["north_m"] - neutral["north_m"],
                            ),
                        )
                    )
                self.send(200, {"config": c, "rows": rows, "neutral": neutral})
                return
            result = simulate(config)
            neutral = simulate({**result["config"], "mode": "neutral"})
            result["mission"] = build_mission(result)
            result["neutral"] = {
                "summary": neutral["summary"],
                "trajectory": neutral["trajectory"],
                "mission": build_mission(neutral),
            }
            result["summary"]["endpoint_difference_from_neutral_m"] = math.hypot(
                result["summary"]["east_m"] - neutral["summary"]["east_m"],
                result["summary"]["north_m"] - neutral["summary"]["north_m"],
            )
            self.send(200, result)
        except (ValueError, TypeError, OverflowError) as e:
            self.send(400, {"error": str(e)})


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", type=int, default=8765)
    args = p.parse_args()
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    print(f"TUSI recovery simulator: http://127.0.0.1:{args.port}", flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == "__main__":
    main()
