#!/usr/bin/env python3

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import subprocess
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Odometry, Path as NavPath


@dataclass
class MethodConfig:
    method: str
    launch_file: str
    smoother_id: Optional[str]


METHODS: Dict[str, MethodConfig] = {
    'no_smoother': MethodConfig('no_smoother', 'bringup_no_smoother_tb3.launch.py', None),
    'savgol': MethodConfig('savgol', 'bringup_savgol_tb3.launch.py', 'savitzky_golay'),
    'se2_hybrid': MethodConfig('se2_hybrid', 'bringup_se2_hybrid_tb3.launch.py', 'se2_hybrid'),
}


def yaw_to_quat(yaw: float):
    half = 0.5 * yaw
    return (0.0, 0.0, math.sin(half), math.cos(half))


def make_pose(x: float, y: float, yaw: float, frame_id: str = 'map') -> PoseStamped:
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    qx, qy, qz, qw = yaw_to_quat(yaw)
    msg.pose.position.x = float(x)
    msg.pose.position.y = float(y)
    msg.pose.position.z = 0.0
    msg.pose.orientation.x = qx
    msg.pose.orientation.y = qy
    msg.pose.orientation.z = qz
    msg.pose.orientation.w = qw
    return msg


def path_length(path: NavPath) -> float:
    if len(path.poses) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(path.poses)):
        p0 = path.poses[i - 1].pose.position
        p1 = path.poses[i].pose.position
        total += math.hypot(p1.x - p0.x, p1.y - p0.y)
    return total


def pose_list_length(points: List[Tuple[float, float]]) -> float:
    if len(points) < 2:
        return 0.0
    total = 0.0
    for i in range(1, len(points)):
        total += math.hypot(points[i][0] - points[i - 1][0], points[i][1] - points[i - 1][1])
    return total


def path_headings(path: NavPath) -> np.ndarray:
    if len(path.poses) < 2:
        return np.array([0.0])
    xs = np.array([p.pose.position.x for p in path.poses], dtype=float)
    ys = np.array([p.pose.position.y for p in path.poses], dtype=float)
    dx = np.gradient(xs)
    dy = np.gradient(ys)
    return np.arctan2(dy, dx)


def wrap_angle(angle: np.ndarray) -> np.ndarray:
    return (angle + np.pi) % (2.0 * np.pi) - np.pi


def curvature_metrics(path: NavPath) -> Tuple[float, float]:
    if len(path.poses) < 3:
        return 0.0, 0.0

    xs = np.array([p.pose.position.x for p in path.poses], dtype=float)
    ys = np.array([p.pose.position.y for p in path.poses], dtype=float)
    theta = np.unwrap(path_headings(path))
    ds = np.hypot(np.diff(xs), np.diff(ys))
    ds = np.where(ds < 1e-6, 1e-6, ds)

    kappa = np.diff(theta) / ds
    mean_abs_kappa = float(np.mean(np.abs(kappa))) if kappa.size > 0 else 0.0
    dkappa = np.diff(kappa)
    mean_abs_dkappa = float(np.mean(np.abs(dkappa))) if dkappa.size > 0 else 0.0
    return mean_abs_kappa, mean_abs_dkappa


def heading_error(raw_path: NavPath, smoothed_path: NavPath) -> float:
    raw_h = path_headings(raw_path)
    sm_h = path_headings(smoothed_path)
    n = min(raw_h.shape[0], sm_h.shape[0])
    if n == 0:
        return 0.0
    return float(np.mean(np.abs(wrap_angle(sm_h[:n] - raw_h[:n]))))


class BenchmarkRunner:
    def __init__(self, output_dir: Path, goals: List[dict], nav_timeout: float, startup_wait: float):
        self.output_dir = output_dir
        self.goals = goals
        self.nav_timeout = nav_timeout
        self.startup_wait = startup_wait

    def _load_method_launch(self, method: str) -> List[str]:
        method_cfg = METHODS[method]
        return ['ros2', 'launch', 'nav2_se2_hybrid_smoother', method_cfg.launch_file, 'headless:=True']

    def _save_paths_csv(self, method: str, goal_idx: int, raw_path: NavPath, smoothed_path: NavPath, traj_xy):
        path_dir = self.output_dir / 'paths' / method
        path_dir.mkdir(parents=True, exist_ok=True)

        raw_csv = path_dir / f'goal_{goal_idx:02d}_raw.csv'
        smooth_csv = path_dir / f'goal_{goal_idx:02d}_smoothed.csv'
        traj_csv = path_dir / f'goal_{goal_idx:02d}_trajectory.csv'

        with raw_csv.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for pose in raw_path.poses:
                writer.writerow([pose.pose.position.x, pose.pose.position.y])

        with smooth_csv.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for pose in smoothed_path.poses:
                writer.writerow([pose.pose.position.x, pose.pose.position.y])

        with traj_csv.open('w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y'])
            for x, y in traj_xy:
                writer.writerow([x, y])

    def run_method(self, method: str) -> List[dict]:
        cfg = METHODS[method]
        launch_cmd = self._load_method_launch(method)
        env = os.environ.copy()
        env.setdefault('TURTLEBOT3_MODEL', 'burger')

        launch_proc = subprocess.Popen(launch_cmd, env=env)
        records = []

        try:
            time.sleep(self.startup_wait)

            rclpy.init(args=None)
            navigator = BasicNavigator()

            last_pose = {'xy': None}
            trajectory_xy: List[Tuple[float, float]] = []

            def odom_cb(msg: Odometry):
                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                last_pose['xy'] = (x, y)
                trajectory_xy.append((x, y))

            navigator.create_subscription(Odometry, '/odom', odom_cb, 10)

            initial = self.goals[0]['start']
            initial_pose = make_pose(initial['x'], initial['y'], initial['yaw'])
            init_cov = PoseWithCovarianceStamped()
            init_cov.header.frame_id = 'map'
            init_cov.pose.pose = initial_pose.pose
            init_cov.pose.covariance = [0.0] * 36
            init_cov.pose.covariance[0] = 0.25
            init_cov.pose.covariance[7] = 0.25
            init_cov.pose.covariance[35] = 0.068
            navigator.setInitialPose(initial_pose)
            navigator.waitUntilNav2Active(localizer='amcl')

            for goal_idx, goal_cfg in enumerate(self.goals, start=1):
                trajectory_xy.clear()
                start = goal_cfg['start']
                goal = goal_cfg['goal']
                start_pose = make_pose(start['x'], start['y'], start['yaw'])
                goal_pose = make_pose(goal['x'], goal['y'], goal['yaw'])

                raw_path = navigator.getPath(start_pose, goal_pose, planner_id='GridBased', use_start=True)
                if raw_path is None:
                    records.append({
                        'method': method,
                        'goal_index': goal_idx,
                        'status': 'planner_failed',
                    })
                    continue

                smoothing_ms = 0.0
                if cfg.smoother_id is None:
                    smoothed_path = raw_path
                else:
                    t0 = time.perf_counter()
                    smoothed_path = navigator.smoothPath(raw_path, smoother_id=cfg.smoother_id)
                    smoothing_ms = (time.perf_counter() - t0) * 1000.0
                    if smoothed_path is None:
                        smoothed_path = raw_path

                kappa_mean, dkappa_mean = curvature_metrics(smoothed_path)
                raw_kappa_mean, raw_dkappa_mean = curvature_metrics(raw_path)
                heading_err = heading_error(raw_path, smoothed_path)

                nav_start = time.perf_counter()
                navigator.goToPose(goal_pose)
                while not navigator.isTaskComplete():
                    rclpy.spin_once(navigator, timeout_sec=0.05)
                    if time.perf_counter() - nav_start > self.nav_timeout:
                        navigator.cancelTask()
                        break

                nav_result = navigator.getResult()
                nav_time = time.perf_counter() - nav_start

                success = nav_result == TaskResult.SUCCEEDED
                status = 'succeeded' if success else 'failed'

                final_err = float('nan')
                if last_pose['xy'] is not None:
                    final_err = math.hypot(last_pose['xy'][0] - goal['x'], last_pose['xy'][1] - goal['y'])

                traveled = pose_list_length(trajectory_xy)

                self._save_paths_csv(method, goal_idx, raw_path, smoothed_path, trajectory_xy)

                records.append({
                    'method': method,
                    'goal_index': goal_idx,
                    'status': status,
                    'success': int(success),
                    'raw_path_length': path_length(raw_path),
                    'smoothed_path_length': path_length(smoothed_path),
                    'raw_mean_abs_curvature': raw_kappa_mean,
                    'smoothed_mean_abs_curvature': kappa_mean,
                    'raw_mean_abs_curvature_variation': raw_dkappa_mean,
                    'smoothed_mean_abs_curvature_variation': dkappa_mean,
                    'heading_error_raw_vs_smoothed': heading_err,
                    'smoothing_time_ms': smoothing_ms,
                    'navigation_time_s': nav_time,
                    'robot_distance_m': traveled,
                    'final_goal_error_m': final_err,
                })

            navigator.lifecycleShutdown()
            navigator.destroy_node()
            rclpy.shutdown()

        finally:
            try:
                launch_proc.send_signal(signal.SIGINT)
                launch_proc.wait(timeout=20)
            except Exception:
                launch_proc.kill()

        return records

    def save_reports(self, records: List[dict]):
        self.output_dir.mkdir(parents=True, exist_ok=True)

        raw_csv = self.output_dir / 'benchmark_raw.csv'
        summary_csv = self.output_dir / 'benchmark_summary.csv'

        df = pd.DataFrame(records)
        df.to_csv(raw_csv, index=False)

        summary = (
            df.groupby('method', dropna=False)
            .agg(
                runs=('goal_index', 'count'),
                success_rate=('success', 'mean'),
                raw_path_length_mean=('raw_path_length', 'mean'),
                smoothed_path_length_mean=('smoothed_path_length', 'mean'),
                raw_curvature_mean=('raw_mean_abs_curvature', 'mean'),
                smoothed_curvature_mean=('smoothed_mean_abs_curvature', 'mean'),
                raw_curvature_variation_mean=('raw_mean_abs_curvature_variation', 'mean'),
                smoothed_curvature_variation_mean=('smoothed_mean_abs_curvature_variation', 'mean'),
                heading_error_mean=('heading_error_raw_vs_smoothed', 'mean'),
                smoothing_time_ms_mean=('smoothing_time_ms', 'mean'),
                navigation_time_s_mean=('navigation_time_s', 'mean'),
                robot_distance_m_mean=('robot_distance_m', 'mean'),
                final_goal_error_m_mean=('final_goal_error_m', 'mean'),
            )
            .reset_index()
        )

        if 'savgol' in set(summary['method']):
            savgol_nav_time = float(summary.loc[summary['method'] == 'savgol', 'navigation_time_s_mean'].iloc[0])
            summary['relative_nav_time_vs_savgol'] = summary['navigation_time_s_mean'] / max(savgol_nav_time, 1e-6)

        summary.to_csv(summary_csv, index=False)

        self._plot_metrics(df)
        self._plot_paths(df)

    def _plot_metrics(self, df: pd.DataFrame):
        fig, axes = plt.subplots(2, 2, figsize=(12, 8))
        agg = df.groupby('method').mean(numeric_only=True)

        agg['smoothed_mean_abs_curvature'].plot(kind='bar', ax=axes[0, 0], title='Mean |Curvature|')
        agg['smoothed_mean_abs_curvature_variation'].plot(kind='bar', ax=axes[0, 1], title='Mean |ΔCurvature|')
        agg['navigation_time_s'].plot(kind='bar', ax=axes[1, 0], title='Navigation time [s]')
        agg['success'].plot(kind='bar', ax=axes[1, 1], title='Success rate')

        for ax in axes.flatten():
            ax.grid(alpha=0.3)
            ax.set_xlabel('method')

        plt.tight_layout()
        plt.savefig(self.output_dir / 'metrics_barplots.png', dpi=180)
        plt.close(fig)

    def _plot_paths(self, df: pd.DataFrame):
        fig, ax = plt.subplots(figsize=(8, 8))
        for method in sorted(df['method'].unique()):
            path_dir = self.output_dir / 'paths' / method
            first = path_dir / 'goal_01_smoothed.csv'
            if not first.exists():
                continue
            d = pd.read_csv(first)
            ax.plot(d['x'], d['y'], label=f'{method} smoothed')
            raw = path_dir / 'goal_01_raw.csv'
            if raw.exists() and method == 'no_smoother':
                r = pd.read_csv(raw)
                ax.plot(r['x'], r['y'], 'k--', linewidth=2, label='raw path')
            traj = path_dir / 'goal_01_trajectory.csv'
            if traj.exists():
                t = pd.read_csv(traj)
                ax.plot(t['x'], t['y'], alpha=0.6, label=f'{method} robot')

        ax.set_aspect('equal', adjustable='box')
        ax.set_title('XY trajectories (goal_01)')
        ax.set_xlabel('x [m]')
        ax.set_ylabel('y [m]')
        ax.grid(alpha=0.3)
        ax.legend()
        plt.tight_layout()
        plt.savefig(self.output_dir / 'xy_trajectories.png', dpi=180)
        plt.close(fig)


def load_goals(path: Path) -> List[dict]:
    with path.open('r', encoding='utf-8') as f:
        data = yaml.safe_load(f)
    goals = data.get('goals', [])
    if not goals:
        raise RuntimeError(f'No goals found in {path}')
    return goals


def parse_args():
    parser = argparse.ArgumentParser(description='Benchmark Nav2 smoothers in TB3 simulation')
    parser.add_argument(
        '--methods', nargs='+', default=['no_smoother', 'savgol', 'se2_hybrid'], choices=sorted(METHODS.keys())
    )
    parser.add_argument('--goals-file', type=Path, default=None)
    parser.add_argument('--output-dir', type=Path, default=Path('benchmark_results'))
    parser.add_argument('--nav-timeout', type=float, default=180.0)
    parser.add_argument('--startup-wait', type=float, default=20.0)
    return parser.parse_args()


def main():
    args = parse_args()

    if args.goals_file is None:
        share = Path(get_package_share_directory('nav2_se2_hybrid_benchmark'))
        goals_file = share / 'config' / 'goals_tb3_world.yaml'
    else:
        goals_file = args.goals_file

    goals = load_goals(goals_file)
    runner = BenchmarkRunner(args.output_dir, goals, args.nav_timeout, args.startup_wait)

    all_records: List[dict] = []
    for method in args.methods:
        all_records.extend(runner.run_method(method))

    runner.save_reports(all_records)
    print(f'Benchmark completed. Results in {args.output_dir}')


if __name__ == '__main__':
    main()
