#!/usr/bin/env python3
"""Simple offline benchmark for path smoothers.

Input CSV format per path:
  x,y[,theta][,smoothing_ms]

Example:
  python3 benchmark_paths.py \
    --raw raw.csv \
    --variant none:none.csv \
    --variant savgol:savgol.csv \
    --variant se2_hybrid:se2_hybrid.csv \
    --outdir results
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, Tuple

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


def wrap_angle(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2.0 * np.pi) - np.pi


def load_path(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    required = {"x", "y"}
    if not required.issubset(df.columns):
        raise ValueError(f"{csv_path} must contain x,y columns")
    if "theta" not in df.columns:
        dx = np.gradient(df["x"].to_numpy())
        dy = np.gradient(df["y"].to_numpy())
        df["theta"] = np.arctan2(dy, dx)
    return df


def path_length(df: pd.DataFrame) -> float:
    dx = np.diff(df["x"].to_numpy())
    dy = np.diff(df["y"].to_numpy())
    return float(np.sum(np.hypot(dx, dy)))


def mean_curvature_variation(df: pd.DataFrame) -> float:
    x = df["x"].to_numpy()
    y = df["y"].to_numpy()
    theta = np.unwrap(df["theta"].to_numpy())
    ds = np.hypot(np.diff(x), np.diff(y))
    ds = np.where(ds < 1e-6, 1e-6, ds)
    kappa = np.diff(theta) / ds
    if kappa.size < 2:
        return 0.0
    dk = np.diff(kappa)
    return float(np.mean(np.abs(dk)))


def mean_heading_error(raw: pd.DataFrame, other: pd.DataFrame) -> float:
    n = min(len(raw), len(other))
    if n == 0:
        return 0.0
    e = wrap_angle(other["theta"].to_numpy()[:n] - raw["theta"].to_numpy()[:n])
    return float(np.mean(np.abs(e)))


def benchmark(raw_df: pd.DataFrame, variants: Dict[str, pd.DataFrame]) -> pd.DataFrame:
    rows = []
    for name, df in variants.items():
        row = {
            "variant": name,
            "path_length": path_length(df),
            "mean_curvature_variation": mean_curvature_variation(df),
            "mean_heading_error_vs_raw": mean_heading_error(raw_df, df),
        }
        if "smoothing_ms" in df.columns:
            row["smoothing_time_ms"] = float(df["smoothing_ms"].mean())
        rows.append(row)
    return pd.DataFrame(rows)


def save_plot(raw_df: pd.DataFrame, variants: Dict[str, pd.DataFrame], output: Path) -> None:
    plt.figure(figsize=(7, 6))
    plt.plot(raw_df["x"], raw_df["y"], "k--", label="raw", linewidth=2)
    for name, df in variants.items():
        plt.plot(df["x"], df["y"], label=name)
    plt.gca().set_aspect("equal", adjustable="box")
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title("Path comparison")
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.savefig(output)
    plt.close()


def parse_variant(arg: str) -> Tuple[str, Path]:
    name, path = arg.split(":", maxsplit=1)
    return name, Path(path)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--raw", type=Path, required=True)
    parser.add_argument("--variant", action="append", default=[], help="name:path.csv")
    parser.add_argument("--outdir", type=Path, default=Path("benchmark_out"))
    args = parser.parse_args()

    args.outdir.mkdir(parents=True, exist_ok=True)

    raw_df = load_path(args.raw)
    variants = {}
    for v in args.variant:
        name, path = parse_variant(v)
        variants[name] = load_path(path)

    results = benchmark(raw_df, variants)
    results_path = args.outdir / "metrics.csv"
    results.to_csv(results_path, index=False)

    plot_path = args.outdir / "paths.png"
    save_plot(raw_df, variants, plot_path)

    print(f"Saved metrics: {results_path}")
    print(f"Saved plot: {plot_path}")
    print(results.to_string(index=False))


if __name__ == "__main__":
    main()
