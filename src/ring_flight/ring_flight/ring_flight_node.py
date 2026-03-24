import argparse
import json
import math
import time
from pathlib import Path
from typing import Dict, List

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from crazyflie_py import Crazyswarm
from tf2_msgs.msg import TFMessage


def _distance(a: List[float], b: List[float]) -> float:
    return math.sqrt(
        (a[0] - b[0]) ** 2 +
        (a[1] - b[1]) ** 2 +
        (a[2] - b[2]) ** 2
    )


def _vec_sub(a: List[float], b: List[float]) -> List[float]:
    return [a[0] - b[0], a[1] - b[1], a[2] - b[2]]


def _vec_add(a: List[float], b: List[float]) -> List[float]:
    return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]


def _vec_scale(v: List[float], s: float) -> List[float]:
    return [v[0] * s, v[1] * s, v[2] * s]


def _dot(a: List[float], b: List[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _norm(v: List[float]) -> float:
    return math.sqrt(_dot(v, v))


def _normalize(v: List[float]) -> List[float]:
    n = _norm(v)
    if n < 1e-9:
        return [1.0, 0.0, 0.0]
    return [v[0] / n, v[1] / n, v[2] / n]


def _lerp(a: List[float], b: List[float], t: float) -> List[float]:
    return [
        a[0] + (b[0] - a[0]) * t,
        a[1] + (b[1] - a[1]) * t,
        a[2] + (b[2] - a[2]) * t,
    ]


def _normal_from_orientation(orientation: List[float]) -> List[float]:
    # For vertical rings in this project, yaw is enough to define ring normal.
    yaw = float(orientation[2])
    return [math.cos(yaw), math.sin(yaw), 0.0]


def _assert_in_bounds(point: List[float], bounds: Dict[str, List[float]]) -> None:
    if not bounds["x"][0] <= point[0] <= bounds["x"][1]:
        raise ValueError(f"x out of bounds: {point}")
    if not bounds["y"][0] <= point[1] <= bounds["y"][1]:
        raise ValueError(f"y out of bounds: {point}")
    if not bounds["z"][0] <= point[2] <= bounds["z"][1]:
        raise ValueError(f"z out of bounds: {point}")


class RingPassEvaluator:
    def __init__(self, rings: List[Dict], clearance: float) -> None:
        self.clearance = clearance
        self.rings = []
        for ring in rings:
            center = [float(x) for x in ring["position"]]
            normal = _normalize(_normal_from_orientation(ring["orientation"]))
            self.rings.append({
                "id": int(ring["id"]),
                "center": center,
                "normal": normal,
                "radius": float(ring["radius"]),
                "crossed": False,
                "passed": False,
                "crossing_direction": None,
                "crossing_radial_error": None,
                "crossing_time": None,
                "best_center_distance": float("inf"),
            })

    def observe_segment(self, p0: List[float], p1: List[float], t0: float, t1: float) -> None:
        for ring in self.rings:
            c = ring["center"]
            n = ring["normal"]
            ring["best_center_distance"] = min(
                ring["best_center_distance"],
                _distance(p0, c),
                _distance(p1, c),
            )

            v0 = _vec_sub(p0, c)
            v1 = _vec_sub(p1, c)
            s0 = _dot(v0, n)
            s1 = _dot(v1, n)
            denom = s0 - s1

            if abs(denom) < 1e-9:
                continue
            if s0 * s1 > 0.0:
                continue

            tau = s0 / denom
            if tau < 0.0 or tau > 1.0:
                continue

            p_cross = _lerp(p0, p1, tau)
            v_cross = _vec_sub(p_cross, c)
            axial = _dot(v_cross, n)
            in_plane = _vec_sub(v_cross, _vec_scale(n, axial))
            radial_error = _norm(in_plane)
            direction = "forward" if (s1 - s0) > 0.0 else "reverse"
            t_cross = t0 + (t1 - t0) * tau

            # Keep best crossing candidate (closest to center).
            prev = ring["crossing_radial_error"]
            if prev is None or radial_error < prev:
                ring["crossed"] = True
                ring["crossing_direction"] = direction
                ring["crossing_radial_error"] = radial_error
                ring["crossing_time"] = t_cross

                allowed = max(ring["radius"] - self.clearance, 0.01)
                ring["passed"] = direction == "forward" and radial_error <= allowed

    def summary(self) -> Dict:
        total = len(self.rings)
        passed = sum(1 for r in self.rings if r["passed"])
        completion = 100.0 * passed / max(total, 1)

        passed_errors = [r["crossing_radial_error"] for r in self.rings if r["passed"]]
        if passed_errors:
            # Lower radial error means cleaner pass.
            quality_terms = []
            for ring, err in [(r, r["crossing_radial_error"]) for r in self.rings if r["passed"]]:
                quality_terms.append(max(0.0, 1.0 - err / ring["radius"]))
            quality = 100.0 * sum(quality_terms) / len(quality_terms)
        else:
            quality = 0.0

        # Weighted final score: completion dominates.
        score = 0.8 * completion + 0.2 * quality
        return {
            "rings_passed": passed,
            "rings_total": total,
            "completion_score": round(completion, 2),
            "quality_score": round(quality, 2),
            "final_score": round(score, 2),
            "rings": self.rings,
        }


class RingFlightMission:
    def __init__(self, config_path: Path, report_path: Path = None) -> None:
        self.config_path = config_path
        self.report_path = report_path
        self.config = self._load_config(config_path)
        self.mission = self.config["mission"]
        self.rings = self.config["rings"]
        self.ring_clearance = float(self.mission.get("ring_clearance", 0.03))
        self.tracking_dt = float(self.mission.get("tracking_dt", 0.05))

        self.swarm = Crazyswarm()
        self.time_helper = self.swarm.timeHelper
        self.cf = self.swarm.allcfs.crazyflies[0]
        self.cf_name = self.cf.prefix.lstrip("/")
        self.latest_tf_pos = None
        self.swarm.allcfs.create_subscription(TFMessage, "/tf", self._tf_callback, 10)
        self.evaluator = RingPassEvaluator(self.rings, self.ring_clearance)
        self.trajectory_samples: List[Dict] = []
        self._prev_pos = [0.0, 0.0, 0.0]
        self._prev_t = 0.0

    @staticmethod
    def _load_config(path: Path) -> Dict:
        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        if "mission" not in data or "rings" not in data:
            raise ValueError("Config must contain mission and rings.")
        return data

    def _build_waypoints(self) -> List[List[float]]:
        waypoints: List[List[float]] = []
        bounds = self.mission["bounds"]
        approach_distance = float(self.mission["approach_distance"])
        post_distance = float(self.mission["post_distance"])
        takeoff_point = self.mission["takeoff_point"]

        _assert_in_bounds(takeoff_point, bounds)
        waypoints.append(takeoff_point)

        for ring in self.rings:
            center = ring["position"]
            normal = _normal_from_orientation(ring["orientation"])

            pre = [
                center[0] - normal[0] * approach_distance,
                center[1] - normal[1] * approach_distance,
                center[2] - normal[2] * approach_distance,
            ]
            post = [
                center[0] + normal[0] * post_distance,
                center[1] + normal[1] * post_distance,
                center[2] + normal[2] * post_distance,
            ]

            _assert_in_bounds(pre, bounds)
            _assert_in_bounds(center, bounds)
            _assert_in_bounds(post, bounds)

            waypoints.extend([pre, center, post])

        return waypoints

    def _segment_duration(self, start: List[float], end: List[float]) -> float:
        speed = float(self.mission["speed_mps"])
        min_duration = float(self.mission["min_segment_duration"])
        return max(_distance(start, end) / speed, min_duration)

    def _segment_duration_with_speed(
        self,
        start: List[float],
        end: List[float],
        speed_mps: float,
    ) -> float:
        min_duration = float(self.mission["min_segment_duration"])
        return max(_distance(start, end) / speed_mps, min_duration)

    def _get_position(self) -> List[float]:
        # Prefer latest TF transform from subscription (reliable in simulation).
        if self.latest_tf_pos is not None:
            return self.latest_tf_pos

        # Fallback to pose topic cache.
        pos = self.cf.get_position()
        if pos is None or len(pos) != 3:
            return [0.0, 0.0, 0.0]
        return [float(pos[0]), float(pos[1]), float(pos[2])]

    def _tf_callback(self, msg: TFMessage) -> None:
        for t in msg.transforms:
            if t.child_frame_id == self.cf_name:
                p = t.transform.translation
                self.latest_tf_pos = [float(p.x), float(p.y), float(p.z)]
                return

    def _start_tracking(self) -> None:
        self._prev_t = self.time_helper.time()
        self._prev_pos = self._get_position()
        self.trajectory_samples.append({
            "t": self._prev_t,
            "pos": self._prev_pos,
        })

    def _track_once(self) -> None:
        t_now = self.time_helper.time()
        p_now = self._get_position()
        self.evaluator.observe_segment(self._prev_pos, p_now, self._prev_t, t_now)
        self.trajectory_samples.append({"t": t_now, "pos": p_now})
        self._prev_t = t_now
        self._prev_pos = p_now

    def _sleep_with_tracking(self, duration: float) -> None:
        end = self.time_helper.time() + duration
        while self.time_helper.time() < end:
            remain = end - self.time_helper.time()
            self.time_helper.sleep(min(self.tracking_dt, remain))
            self._track_once()

    def _write_report(self, waypoints: List[List[float]], summary: Dict, mission_start: float, mission_end: float) -> None:
        if self.report_path is None:
            ts = time.strftime("%Y%m%d_%H%M%S")
            self.report_path = Path.cwd() / f"ring_flight_report_{ts}.json"

        report = {
            "config_path": str(self.config_path),
            "mission_start": mission_start,
            "mission_end": mission_end,
            "mission_duration_s": round(mission_end - mission_start, 3),
            "mission": self.mission,
            "waypoints": waypoints,
            "score": summary,
            "trajectory_samples": self.trajectory_samples,
        }
        with self.report_path.open("w", encoding="utf-8") as f:
            json.dump(report, f, indent=2)

    def run(self) -> None:
        waypoints = self._build_waypoints()
        takeoff_height = float(self.mission["takeoff_height"])
        takeoff_duration = float(self.mission["takeoff_duration"])
        land_height = float(self.mission["land_height"])
        land_duration = float(self.mission["land_duration"])

        print(f"Loaded config: {self.config_path}")
        print(f"Using {len(self.rings)} rings and {len(waypoints)} waypoints.")
        print(f"Tracking dt={self.tracking_dt}s, clearance={self.ring_clearance}m")

        mission_start = self.time_helper.time()
        self._start_tracking()

        self.cf.takeoff(targetHeight=takeoff_height, duration=takeoff_duration)
        self._sleep_with_tracking(takeoff_duration + 0.8)

        previous = waypoints[0]
        for idx, wp in enumerate(waypoints[1:], start=1):
            duration = self._segment_duration(previous, wp)
            print(f"Waypoint {idx}: {wp}, duration={duration:.2f}s")
            self.cf.goTo(wp, 0.0, duration)
            self._sleep_with_tracking(duration + 0.4)
            previous = wp

        self.cf.land(targetHeight=land_height, duration=land_duration)
        self._sleep_with_tracking(land_duration + 0.8)

        mission_end = self.time_helper.time()
        summary = self.evaluator.summary()
        self._write_report(waypoints, summary, mission_start, mission_end)

        print("Mission scoring:")
        print(
            f"  passed={summary['rings_passed']}/{summary['rings_total']}, "
            f"completion={summary['completion_score']}, "
            f"quality={summary['quality_score']}, "
            f"final={summary['final_score']}"
        )
        for ring in summary["rings"]:
            print(
                f"  ring {ring['id']}: passed={ring['passed']}, crossed={ring['crossed']}, "
                f"direction={ring['crossing_direction']}, "
                f"radial_error={ring['crossing_radial_error']}"
            )
        print(f"Report saved: {self.report_path}")
        print("Mission finished.")


def _default_config_path() -> Path:
    package_share = Path(get_package_share_directory("ring_flight"))
    return package_share / "config" / "rings.yaml"


def main() -> None:
    parser = argparse.ArgumentParser(description="Ring flight mission runner")
    parser.add_argument(
        "--config",
        type=Path,
        default=_default_config_path(),
        help="Path to rings.yaml",
    )
    parser.add_argument(
        "--report",
        type=Path,
        default=None,
        help="Path to score report json (default: auto timestamp in current directory)",
    )
    args = parser.parse_args()

    mission = RingFlightMission(args.config, args.report)
    mission.run()


if __name__ == "__main__":
    main()
