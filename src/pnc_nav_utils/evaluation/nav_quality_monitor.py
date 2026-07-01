#!/usr/bin/env python3
"""
Navigation quality monitor for PNC Nav2.

Online evaluator for one or more navigation trials. It listens to goal, global
plan, optional local plan, odom, and cmd_vel, then reports whether the robot
reached the goal and how well the executed trajectory followed the planned path.

Usage:
  python3 src/pnc_nav_utils/evaluation/nav_quality_monitor.py

Typical topics:
  /goal_pose
  /global_plan
  /local_plan     optional, only used when available
  /odom
  /cmd_vel
"""

import argparse
import csv
import json
import math
import os
import statistics
import time
from dataclasses import asdict, dataclass, field
from typing import List, Optional, Sequence, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node


Point = Tuple[float, float, float]


@dataclass
class TrialMetrics:
    trial_id: int
    start_time_unix: float
    end_time_unix: float = 0.0
    success: bool = False
    result: str = "running"

    total_time_s: float = 0.0
    final_goal_error_m: float = 0.0
    planned_path_length_m: float = 0.0
    actual_distance_m: float = 0.0
    path_efficiency: float = 0.0

    mean_tracking_error_m: float = 0.0
    max_tracking_error_m: float = 0.0
    rmse_tracking_error_m: float = 0.0
    tracking_error_p95_m: float = 0.0

    local_plan_available: bool = False
    local_path_length_m: float = 0.0
    local_vs_global_mean_error_m: float = 0.0
    local_vs_global_max_error_m: float = 0.0
    actual_vs_local_mean_error_m: float = 0.0
    actual_vs_local_max_error_m: float = 0.0

    avg_speed_mps: float = 0.0
    max_speed_mps: float = 0.0
    avg_angular_speed_radps: float = 0.0
    max_angular_speed_radps: float = 0.0
    cmd_oscillation_score: float = 0.0
    stop_count: int = 0
    replan_count: int = 0
    navigation_score: float = 0.0


@dataclass
class TrialState:
    metrics: TrialMetrics
    goal: PoseStamped
    global_plan: Optional[Path] = None
    latest_local_plan: Optional[Path] = None
    odom_points: List[Point] = field(default_factory=list)
    cmd_linear: List[float] = field(default_factory=list)
    cmd_angular: List[float] = field(default_factory=list)
    last_odom_point: Optional[Point] = None
    stop_active: bool = False
    last_status_time: float = 0.0


def pose_to_point(pose: PoseStamped) -> Point:
    p = pose.pose.position
    return (p.x, p.y, p.z)


def odom_to_point(msg: Odometry) -> Point:
    p = msg.pose.pose.position
    return (p.x, p.y, p.z)


def path_to_points(path: Path) -> List[Point]:
    return [pose_to_point(pose) for pose in path.poses]


def distance(a: Point, b: Point) -> float:
    dx = a[0] - b[0]
    dy = a[1] - b[1]
    dz = a[2] - b[2]
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def path_length(points: Sequence[Point]) -> float:
    return sum(distance(points[i - 1], points[i]) for i in range(1, len(points)))


def point_to_segment_distance(point: Point, a: Point, b: Point) -> float:
    ax, ay, az = a
    bx, by, bz = b
    px, py, pz = point
    ab = (bx - ax, by - ay, bz - az)
    ap = (px - ax, py - ay, pz - az)
    ab_len_sq = ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]
    if ab_len_sq < 1e-12:
        return distance(point, a)
    t = (ap[0] * ab[0] + ap[1] * ab[1] + ap[2] * ab[2]) / ab_len_sq
    t = max(0.0, min(1.0, t))
    closest = (ax + t * ab[0], ay + t * ab[1], az + t * ab[2])
    return distance(point, closest)


def point_to_polyline_distance(point: Point, polyline: Sequence[Point]) -> float:
    if not polyline:
        return 0.0
    if len(polyline) == 1:
        return distance(point, polyline[0])
    return min(
        point_to_segment_distance(point, polyline[i - 1], polyline[i])
        for i in range(1, len(polyline))
    )


def polyline_errors(points: Sequence[Point], reference: Sequence[Point]) -> List[float]:
    if not points or not reference:
        return []
    return [point_to_polyline_distance(point, reference) for point in points]


def mean(values: Sequence[float]) -> float:
    return statistics.fmean(values) if values else 0.0


def rmse(values: Sequence[float]) -> float:
    if not values:
        return 0.0
    return math.sqrt(statistics.fmean(v * v for v in values))


def percentile(values: Sequence[float], percent: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = (len(ordered) - 1) * percent / 100.0
    low = math.floor(index)
    high = math.ceil(index)
    if low == high:
        return ordered[int(index)]
    weight = index - low
    return ordered[low] * (1.0 - weight) + ordered[high] * weight


def cmd_oscillation_score(linear: Sequence[float], angular: Sequence[float]) -> float:
    if len(linear) < 2 or len(angular) < 2:
        return 0.0
    linear_changes = [abs(linear[i] - linear[i - 1]) for i in range(1, len(linear))]
    angular_changes = [abs(angular[i] - angular[i - 1]) for i in range(1, len(angular))]
    return mean(linear_changes) + 0.5 * mean(angular_changes)


def compute_score(metrics: TrialMetrics, goal_tolerance: float, timeout_s: float) -> float:
    score = 0.0
    if metrics.success:
        score += 40.0

    if goal_tolerance > 1e-6:
        goal_error_score = max(0.0, 1.0 - metrics.final_goal_error_m / (goal_tolerance * 3.0))
        score += 15.0 * goal_error_score

    if timeout_s > 1e-6:
        time_score = max(0.0, 1.0 - metrics.total_time_s / timeout_s)
        score += 15.0 * time_score

    if metrics.path_efficiency > 0.0:
        efficiency_error = abs(1.0 - metrics.path_efficiency)
        score += 15.0 * max(0.0, 1.0 - efficiency_error)

    tracking_score = max(0.0, 1.0 - metrics.rmse_tracking_error_m / max(goal_tolerance * 3.0, 0.3))
    score += 10.0 * tracking_score

    oscillation_score = max(0.0, 1.0 - metrics.cmd_oscillation_score / 0.5)
    score += 5.0 * oscillation_score

    return round(max(0.0, min(100.0, score)), 1)


class NavQualityMonitor(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("nav_quality_monitor")
        self.args = args
        self.current: Optional[TrialState] = None
        self.completed_trials: List[TrialMetrics] = []
        self.trial_counter = 0
        self.plotter = None
        self.plot_enabled = args.plot
        self.last_plot_time = 0.0
        self.last_idle_status_time = 0.0

        self.create_subscription(PoseStamped, args.goal_topic, self.goal_callback, 10)
        self.create_subscription(Path, args.global_plan_topic, self.global_plan_callback, 10)
        self.create_subscription(Path, args.local_plan_topic, self.local_plan_callback, 10)
        self.create_subscription(Odometry, args.odom_topic, self.odom_callback, 50)
        self.create_subscription(Twist, args.cmd_vel_topic, self.cmd_vel_callback, 50)
        self.timer = self.create_timer(0.5, self.timer_callback)

        self.get_logger().info("Nav quality monitor started")
        self.get_logger().info(
            f"topics: goal={args.goal_topic}, global={args.global_plan_topic}, "
            f"local={args.local_plan_topic}, odom={args.odom_topic}, cmd={args.cmd_vel_topic}"
        )
        if self.plot_enabled:
            self.ensure_plotter()
        else:
            self.get_logger().info("live plot disabled; pass --plot to show global/local/actual paths")

    def goal_callback(self, msg: PoseStamped) -> None:
        if self.current is not None:
            self.finish_current("new_goal", success=False)

        self.trial_counter += 1
        metrics = TrialMetrics(trial_id=self.trial_counter, start_time_unix=time.time())
        self.current = TrialState(metrics=metrics, goal=msg, last_status_time=time.time())
        goal = msg.pose.position
        self.get_logger().info(
            f"Trial {self.trial_counter} started: goal=({goal.x:.2f}, {goal.y:.2f}, {goal.z:.2f})"
        )
        self.ensure_plotter()
        if self.plotter:
            self.plotter.reset()
            self.last_plot_time = 0.0
            self.refresh_plot(force=True)

    def global_plan_callback(self, msg: Path) -> None:
        if self.current is None:
            return
        self.current.global_plan = msg
        self.current.metrics.replan_count += 1
        points = path_to_points(msg)
        self.current.metrics.planned_path_length_m = path_length(points)
        if self.plotter:
            self.refresh_plot(force=True)
        self.get_logger().info(
            f"Trial {self.current.metrics.trial_id}: global plan received, "
            f"poses={len(points)}, length={self.current.metrics.planned_path_length_m:.2f}m"
        )

    def local_plan_callback(self, msg: Path) -> None:
        if self.current is None or not msg.poses:
            return
        self.current.latest_local_plan = msg
        self.current.metrics.local_plan_available = True
        if self.plotter:
            self.refresh_plot(force=True)

    def odom_callback(self, msg: Odometry) -> None:
        if self.current is None:
            return

        point = odom_to_point(msg)
        self.current.odom_points.append(point)

        if self.current.last_odom_point is not None:
            step = distance(self.current.last_odom_point, point)
            if step < self.args.max_odom_step:
                self.current.metrics.actual_distance_m += step
        self.current.last_odom_point = point

        goal_point = pose_to_point(self.current.goal)
        goal_error = distance(point, goal_point)
        self.current.metrics.final_goal_error_m = goal_error
        if goal_error <= self.args.goal_tolerance:
            self.finish_current("goal_reached", success=True)

    def cmd_vel_callback(self, msg: Twist) -> None:
        if self.current is None:
            return
        speed = math.hypot(msg.linear.x, msg.linear.y)
        angular = abs(msg.angular.z)
        self.current.cmd_linear.append(speed)
        self.current.cmd_angular.append(angular)

        stopped = speed < self.args.stop_speed and angular < self.args.stop_angular_speed
        if stopped and not self.current.stop_active:
            self.current.metrics.stop_count += 1
        self.current.stop_active = stopped

    def timer_callback(self) -> None:
        now = time.time()
        if self.current is None:
            if self.plotter:
                self.plotter.spin_once()
            if now - self.last_idle_status_time >= self.args.idle_print_period:
                self.get_logger().info(
                    f"Waiting for goal on {self.args.goal_topic}; no active trial yet"
                )
                self.last_idle_status_time = now
            return
        elapsed = time.time() - self.current.metrics.start_time_unix
        if now - self.current.last_status_time >= self.args.print_period:
            self.print_running_status(elapsed)
            self.current.last_status_time = now
        if self.plotter:
            self.refresh_plot()
        if elapsed > self.args.timeout:
            self.finish_current("timeout", success=False)

    def ensure_plotter(self) -> None:
        if self.plot_enabled and self.plotter is None:
            try:
                self.plotter = LivePlotter()
            except Exception as exc:
                self.plot_enabled = False
                self.plotter = None
                self.get_logger().error(
                    "live plot disabled: failed to create a Matplotlib GUI window. "
                    f"{exc}. Try running with MPLBACKEND=TkAgg or install a GUI backend."
                )
                return
            self.get_logger().info(
                f"live plot enabled; backend={self.plotter.backend}, "
                f"refresh={self.args.plot_period:.2f}s, "
                f"visible_odom_points={self.args.max_plot_points}"
            )

    def refresh_plot(self, force: bool = False) -> None:
        if self.plotter is None or self.current is None:
            return

        now = time.time()
        if not force and now - self.last_plot_time < self.args.plot_period:
            return

        goal = pose_to_point(self.current.goal)
        global_points = path_to_points(self.current.global_plan) if self.current.global_plan else []
        local_points = (
            path_to_points(self.current.latest_local_plan) if self.current.latest_local_plan else []
        )
        visible_odom_points = max(1, self.args.max_plot_points)
        actual_points = self.current.odom_points[-visible_odom_points:]
        self.plotter.update_all(goal, global_points, local_points, actual_points)
        self.last_plot_time = now

    def finish_current(self, result: str, success: bool) -> None:
        if self.current is None:
            return

        metrics = self.current.metrics
        metrics.end_time_unix = time.time()
        metrics.total_time_s = metrics.end_time_unix - metrics.start_time_unix
        metrics.success = success
        metrics.result = result

        global_points = path_to_points(self.current.global_plan) if self.current.global_plan else []
        local_points = path_to_points(self.current.latest_local_plan) if self.current.latest_local_plan else []
        actual_points = self.current.odom_points

        if metrics.actual_distance_m > 1e-6:
            metrics.path_efficiency = metrics.planned_path_length_m / metrics.actual_distance_m

        tracking_errors = polyline_errors(actual_points, global_points)
        metrics.mean_tracking_error_m = mean(tracking_errors)
        metrics.max_tracking_error_m = max(tracking_errors) if tracking_errors else 0.0
        metrics.rmse_tracking_error_m = rmse(tracking_errors)
        metrics.tracking_error_p95_m = percentile(tracking_errors, 95.0)

        if local_points:
            metrics.local_plan_available = True
            metrics.local_path_length_m = path_length(local_points)
            local_global_errors = polyline_errors(local_points, global_points)
            actual_local_errors = polyline_errors(actual_points, local_points)
            metrics.local_vs_global_mean_error_m = mean(local_global_errors)
            metrics.local_vs_global_max_error_m = max(local_global_errors) if local_global_errors else 0.0
            metrics.actual_vs_local_mean_error_m = mean(actual_local_errors)
            metrics.actual_vs_local_max_error_m = max(actual_local_errors) if actual_local_errors else 0.0

        metrics.avg_speed_mps = mean(self.current.cmd_linear)
        metrics.max_speed_mps = max(self.current.cmd_linear) if self.current.cmd_linear else 0.0
        metrics.avg_angular_speed_radps = mean(self.current.cmd_angular)
        metrics.max_angular_speed_radps = max(self.current.cmd_angular) if self.current.cmd_angular else 0.0
        metrics.cmd_oscillation_score = cmd_oscillation_score(
            self.current.cmd_linear, self.current.cmd_angular
        )
        metrics.navigation_score = compute_score(metrics, self.args.goal_tolerance, self.args.timeout)

        self.completed_trials.append(metrics)
        self.write_outputs()
        self.print_summary(metrics)
        if self.plotter:
            self.refresh_plot(force=True)
            self.plotter.set_title(
                f"Trial {metrics.trial_id}: {metrics.result}, score={metrics.navigation_score:.1f}"
            )
        self.current = None

    def print_running_status(self, elapsed: float) -> None:
        if self.current is None:
            return
        metrics = self.current.metrics
        global_points = path_to_points(self.current.global_plan) if self.current.global_plan else []
        recent_errors = polyline_errors(self.current.odom_points[-50:], global_points)
        recent_rmse = rmse(recent_errors)
        latest_speed = self.current.cmd_linear[-1] if self.current.cmd_linear else 0.0
        latest_wz = self.current.cmd_angular[-1] if self.current.cmd_angular else 0.0
        self.get_logger().info(
            f"Trial {metrics.trial_id} running: t={elapsed:.1f}s, "
            f"goal_error={metrics.final_goal_error_m:.3f}m, "
            f"actual={metrics.actual_distance_m:.2f}m, "
            f"recent_tracking_rmse={recent_rmse:.3f}m, "
            f"cmd=({latest_speed:.2f}m/s, {latest_wz:.2f}rad/s), "
            f"plans={metrics.replan_count}"
        )

    def print_summary(self, metrics: TrialMetrics) -> None:
        status = "PASS" if metrics.success else "FAIL"
        self.get_logger().info(
            f"Trial {metrics.trial_id} {status}: result={metrics.result}, "
            f"score={metrics.navigation_score:.1f}/100, time={metrics.total_time_s:.1f}s, "
            f"final_error={metrics.final_goal_error_m:.3f}m"
        )
        self.get_logger().info(
            f"  path: planned={metrics.planned_path_length_m:.2f}m, "
            f"actual={metrics.actual_distance_m:.2f}m, efficiency={metrics.path_efficiency:.3f}"
        )
        self.get_logger().info(
            f"  global tracking: mean={metrics.mean_tracking_error_m:.3f}m, "
            f"rmse={metrics.rmse_tracking_error_m:.3f}m, "
            f"p95={metrics.tracking_error_p95_m:.3f}m, max={metrics.max_tracking_error_m:.3f}m"
        )
        if metrics.local_plan_available:
            self.get_logger().info(
                f"  local plan: length={metrics.local_path_length_m:.2f}m, "
                f"local-vs-global mean={metrics.local_vs_global_mean_error_m:.3f}m, "
                f"actual-vs-local mean={metrics.actual_vs_local_mean_error_m:.3f}m"
            )
        else:
            self.get_logger().info("  local plan: unavailable")
        self.get_logger().info(
            f"  control: avg_speed={metrics.avg_speed_mps:.2f}m/s, "
            f"max_speed={metrics.max_speed_mps:.2f}m/s, "
            f"max_wz={metrics.max_angular_speed_radps:.2f}rad/s, "
            f"oscillation={metrics.cmd_oscillation_score:.3f}, stops={metrics.stop_count}"
        )

    def write_outputs(self) -> None:
        if self.args.csv_output:
            self.write_csv(self.args.csv_output)
        if self.args.json_output:
            latest = asdict(self.completed_trials[-1]) if self.completed_trials else {}
            with open(self.args.json_output, "w", encoding="utf-8") as f:
                json.dump(latest, f, indent=2)

    def write_csv(self, path: str) -> None:
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        rows = [asdict(trial) for trial in self.completed_trials]
        if not rows:
            return
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            writer.writerows(rows)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Evaluate PNC Nav2 navigation quality online.")
    parser.add_argument("--goal-topic", default="/goal_pose")
    parser.add_argument("--global-plan-topic", default="/global_plan")
    parser.add_argument("--local-plan-topic", default="/local_plan")
    parser.add_argument("--odom-topic", default="/odom")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--goal-tolerance", type=float, default=0.2)
    parser.add_argument("--timeout", type=float, default=60.0)
    parser.add_argument("--stop-speed", type=float, default=0.02)
    parser.add_argument("--stop-angular-speed", type=float, default=0.02)
    parser.add_argument("--max-odom-step", type=float, default=1.0)
    parser.add_argument("--print-period", type=float, default=1.0)
    parser.add_argument("--idle-print-period", type=float, default=5.0)
    parser.add_argument("--plot", action="store_true")
    parser.add_argument("--plot-period", type=float, default=0.5)
    parser.add_argument("--max-plot-points", type=int, default=2000)
    parser.add_argument("--csv-output", default="nav_quality_results.csv")
    parser.add_argument("--json-output", default="nav_quality_latest.json")
    return parser.parse_args()


class LivePlotter:
    def __init__(self) -> None:
        plt = self._load_pyplot()

        self.plt = plt
        self.backend = plt.get_backend()
        self.fig, self.ax = plt.subplots()
        self.global_line, = self.ax.plot([], [], "b-", label="global_plan")
        self.local_line, = self.ax.plot([], [], color="orange", label="local_plan")
        self.actual_line, = self.ax.plot([], [], "g-", label="actual_odom")
        self.goal_line, = self.ax.plot([], [], "r*", markersize=12, label="goal")
        self.start_line, = self.ax.plot([], [], "ko", markersize=6, label="start")
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.grid(True)
        self.ax.legend(loc="best")
        self.ax.set_title("Navigation quality monitor")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        plt.ion()
        plt.show(block=False)
        self.spin_once()

    @staticmethod
    def _load_pyplot():
        import importlib
        import sys

        import matplotlib

        backends = []
        current_backend = matplotlib.get_backend()
        if LivePlotter._is_interactive_backend(current_backend):
            backends.append(current_backend)
        backends.extend(["TkAgg", "Qt5Agg", "QtAgg"])

        errors = []
        for backend in dict.fromkeys(backends):
            try:
                matplotlib.use(backend, force=True)
                sys.modules.pop("matplotlib.pyplot", None)
                plt = importlib.import_module("matplotlib.pyplot")
                selected_backend = plt.get_backend()
                if LivePlotter._is_interactive_backend(selected_backend):
                    return plt
                errors.append(f"{backend}: selected non-interactive backend {selected_backend}")
            except Exception as exc:
                errors.append(f"{backend}: {exc}")

        details = "; ".join(errors) if errors else f"current backend is {current_backend}"
        raise RuntimeError(f"no interactive Matplotlib backend is available ({details})")

    @staticmethod
    def _is_interactive_backend(backend: object) -> bool:
        name = str(backend).lower()
        non_interactive = {"agg", "pdf", "ps", "svg", "cairo", "template"}
        return name not in non_interactive and "inline" not in name

    def reset(self) -> None:
        for line in (self.global_line, self.local_line, self.actual_line, self.goal_line, self.start_line):
            line.set_data([], [])
        self.set_title("Navigation quality monitor")
        self.spin_once()

    def update_goal(self, point: Point) -> None:
        self.goal_line.set_data([point[0]], [point[1]])
        self._rescale()

    def update_global(self, points: Sequence[Point]) -> None:
        self._set_line(self.global_line, points)

    def update_local(self, points: Sequence[Point]) -> None:
        self._set_line(self.local_line, points)

    def update_actual(self, points: Sequence[Point]) -> None:
        self._set_line(self.actual_line, points)
        if points:
            self.start_line.set_data([points[0][0]], [points[0][1]])

    def update_all(
        self,
        goal: Point,
        global_points: Sequence[Point],
        local_points: Sequence[Point],
        actual_points: Sequence[Point],
    ) -> None:
        self.goal_line.set_data([goal[0]], [goal[1]])
        self._set_line_data(self.global_line, global_points)
        self._set_line_data(self.local_line, local_points)
        self._set_line_data(self.actual_line, actual_points)
        if actual_points:
            self.start_line.set_data([actual_points[0][0]], [actual_points[0][1]])
        else:
            self.start_line.set_data([], [])
        self._rescale()

    def set_title(self, title: str) -> None:
        self.ax.set_title(title)
        self.spin_once()

    def spin_once(self) -> None:
        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)

    def _set_line(self, line, points: Sequence[Point]) -> None:
        self._set_line_data(line, points)
        self._rescale()

    def _set_line_data(self, line, points: Sequence[Point]) -> None:
        line.set_data([p[0] for p in points], [p[1] for p in points])

    def _rescale(self) -> None:
        self.ax.relim()
        self.ax.autoscale_view()
        self.spin_once()


def main() -> None:
    args = parse_args()
    node = None
    rclpy.init()
    try:
        node = NavQualityMonitor(args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None and node.current is not None:
            node.finish_current("interrupted", success=False)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
