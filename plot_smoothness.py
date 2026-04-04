#!/usr/bin/env python3

from pathlib import Path
import argparse
from typing import Optional

import matplotlib.pyplot as plt
import pandas as pd


REQUIRED_COLUMNS = {
    "time_s",
    "vx_cmd_smooth",
    "vy_cmd_smooth",
    "vx_smooth",
    "vy_smooth",
    "cmd_speed_smooth",
    "odom_speed_smooth",
}


def load_merged(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path).sort_values("time_s").reset_index(drop=True)
    missing = sorted(REQUIRED_COLUMNS - set(df.columns))
    if missing:
        raise ValueError(f"Missing columns in {path}: {missing}")
    return df


def slice_window(df: pd.DataFrame, start_s: Optional[float], end_s: Optional[float]) -> pd.DataFrame:
    sliced = df.copy()
    if start_s is not None:
        sliced = sliced[sliced["time_s"] >= start_s]
    if end_s is not None:
        sliced = sliced[sliced["time_s"] <= end_s]
    sliced = sliced.reset_index(drop=True)
    if sliced.empty:
        raise ValueError("Selected time window is empty. Please adjust --*-start/--*-end.")
    sliced["time_s"] = sliced["time_s"] - float(sliced["time_s"].iloc[0])
    return sliced


def make_vx_comparison_figure(baseline_df: pd.DataFrame, full_df: pd.DataFrame, out_path: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(9.2, 5.8), sharex=False, constrained_layout=True)

    axes[0].plot(
        baseline_df["time_s"],
        baseline_df["vx_cmd_smooth"],
        color="#c44e52",
        linewidth=2.4,
        label="vx cmd",
    )
    axes[0].plot(
        baseline_df["time_s"],
        baseline_df["vx_smooth"],
        color="#4c72b0",
        linewidth=1.9,
        linestyle="--",
        label="vx odom",
    )
    axes[0].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[0].set_title("Baseline")
    axes[0].set_ylabel("Longitudinal velocity (m/s)")
    axes[0].grid(alpha=0.25)
    axes[0].legend(frameon=False, ncol=2, loc="upper right")

    axes[1].plot(
        full_df["time_s"],
        full_df["vx_cmd_smooth"],
        color="#c44e52",
        linewidth=2.4,
        label="vx cmd",
    )
    axes[1].plot(
        full_df["time_s"],
        full_df["vx_smooth"],
        color="#4c72b0",
        linewidth=1.9,
        linestyle="--",
        label="vx odom",
    )
    axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[1].set_title("Full Method")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Longitudinal velocity (m/s)")
    axes[1].grid(alpha=0.25)
    axes[1].legend(frameon=False, ncol=2, loc="upper right")

    fig.savefig(out_path, dpi=220)
    fig.savefig(out_path.with_suffix(".pdf"))
    plt.close(fig)


def make_vy_support_figure(baseline_df: pd.DataFrame, full_df: pd.DataFrame, out_path: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(9.2, 5.8), sharex=False, constrained_layout=True)

    axes[0].plot(
        baseline_df["time_s"],
        baseline_df["vy_cmd_smooth"],
        color="#55a868",
        linewidth=2.2,
        label="vy cmd",
    )
    axes[0].plot(
        baseline_df["time_s"],
        baseline_df["vy_smooth"],
        color="#8172b2",
        linewidth=1.8,
        linestyle="--",
        label="vy odom",
    )
    axes[0].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[0].set_title("Baseline")
    axes[0].set_ylabel("Lateral velocity (m/s)")
    axes[0].grid(alpha=0.25)
    axes[0].legend(frameon=False, ncol=2, loc="upper right")

    axes[1].plot(
        full_df["time_s"],
        full_df["vy_cmd_smooth"],
        color="#55a868",
        linewidth=2.2,
        label="vy cmd",
    )
    axes[1].plot(
        full_df["time_s"],
        full_df["vy_smooth"],
        color="#8172b2",
        linewidth=1.8,
        linestyle="--",
        label="vy odom",
    )
    axes[1].axhline(0.0, color="black", linewidth=0.8, alpha=0.5)
    axes[1].set_title("Full Method")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Lateral velocity (m/s)")
    axes[1].grid(alpha=0.25)
    axes[1].legend(frameon=False, ncol=2, loc="upper right")

    fig.savefig(out_path, dpi=220)
    fig.savefig(out_path.with_suffix(".pdf"))
    plt.close(fig)


def make_speed_support_figure(baseline_df: pd.DataFrame, full_df: pd.DataFrame, out_path: Path) -> None:
    fig, axes = plt.subplots(2, 1, figsize=(9.2, 5.8), sharex=False, constrained_layout=True)

    axes[0].plot(
        baseline_df["time_s"],
        baseline_df["cmd_speed_smooth"],
        color="#dd8452",
        linewidth=2.2,
        label="cmd speed",
    )
    axes[0].plot(
        baseline_df["time_s"],
        baseline_df["odom_speed_smooth"],
        color="#4c72b0",
        linewidth=1.8,
        linestyle="--",
        label="odom speed",
    )
    axes[0].set_title("Baseline")
    axes[0].set_ylabel("Speed magnitude (m/s)")
    axes[0].grid(alpha=0.25)
    axes[0].legend(frameon=False, ncol=2, loc="upper right")

    axes[1].plot(
        full_df["time_s"],
        full_df["cmd_speed_smooth"],
        color="#dd8452",
        linewidth=2.2,
        label="cmd speed",
    )
    axes[1].plot(
        full_df["time_s"],
        full_df["odom_speed_smooth"],
        color="#4c72b0",
        linewidth=1.8,
        linestyle="--",
        label="odom speed",
    )
    axes[1].set_title("Full Method")
    axes[1].set_xlabel("Time (s)")
    axes[1].set_ylabel("Speed magnitude (m/s)")
    axes[1].grid(alpha=0.25)
    axes[1].legend(frameon=False, ncol=2, loc="upper right")

    fig.savefig(out_path, dpi=220)
    fig.savefig(out_path.with_suffix(".pdf"))
    plt.close(fig)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Generate fair smoothness comparison plots from processed CSVs."
    )
    parser.add_argument(
        "--processed-root",
        default="ablation_eval_output/smoothness_processed",
        help="Directory produced by prepare_smoothness_data.py",
    )
    parser.add_argument(
        "--baseline",
        default="baseline_smoothness_20260404_183836_csv",
        help="Processed baseline CSV directory name",
    )
    parser.add_argument(
        "--full",
        default="full_smoothness_20260404_183932_csv",
        help="Processed full-method CSV directory name",
    )
    parser.add_argument(
        "--baseline-start",
        type=float,
        default=None,
        help="Start time (seconds, processed time base) for the baseline window",
    )
    parser.add_argument(
        "--baseline-end",
        type=float,
        default=None,
        help="End time (seconds, processed time base) for the baseline window",
    )
    parser.add_argument(
        "--full-start",
        type=float,
        default=None,
        help="Start time (seconds, processed time base) for the full-method window",
    )
    parser.add_argument(
        "--full-end",
        type=float,
        default=None,
        help="End time (seconds, processed time base) for the full-method window",
    )
    parser.add_argument(
        "--output-dir",
        default="ablation_eval_output/smoothness_plots",
        help="Where to save generated plot files",
    )
    args = parser.parse_args()

    processed_root = Path(args.processed_root)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    baseline_df = load_merged(processed_root / args.baseline / "smoothness_merged.csv")
    full_df = load_merged(processed_root / args.full / "smoothness_merged.csv")

    baseline_df = slice_window(baseline_df, args.baseline_start, args.baseline_end)
    full_df = slice_window(full_df, args.full_start, args.full_end)

    make_vx_comparison_figure(baseline_df, full_df, output_dir / "smoothness_vx_comparison.png")
    make_vy_support_figure(baseline_df, full_df, output_dir / "smoothness_vy_support.png")
    make_speed_support_figure(baseline_df, full_df, output_dir / "smoothness_speed_support.png")

    print(f"Saved plots to {output_dir}")
    print("Recommended main paper figure: smoothness_vx_comparison.png")
    print("Support figures: smoothness_vy_support.png and smoothness_speed_support.png")
    print(
        "Tip: if Baseline is hit and then becomes artificially flat, use --baseline-end "
        "to truncate at the collision moment."
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
