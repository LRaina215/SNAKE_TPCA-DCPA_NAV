#!/usr/bin/env python3

from pathlib import Path
import argparse

import numpy as np
import pandas as pd


def compute_speed(df: pd.DataFrame, x_col: str, y_col: str) -> pd.Series:
    return np.sqrt(df[x_col].fillna(0.0) ** 2 + df[y_col].fillna(0.0) ** 2)


def trim_active_window(cmd_df: pd.DataFrame, threshold: float) -> pd.DataFrame:
    mag = compute_speed(cmd_df, "vx_cmd", "vy_cmd")
    active_idx = np.flatnonzero(mag.to_numpy() > threshold)
    if active_idx.size == 0:
        return cmd_df.copy()
    return cmd_df.iloc[active_idx[0] : active_idx[-1] + 1].copy()


def rolling_seconds(series: pd.Series, time_s: pd.Series, window_s: float) -> pd.Series:
    if len(series) < 2:
        return series.copy()
    dt = np.median(np.diff(time_s.to_numpy()))
    if not np.isfinite(dt) or dt <= 1e-6:
        return series.copy()
    window = max(3, int(round(window_s / dt)))
    return series.rolling(window=window, center=True, min_periods=1).mean()


def process_one(csv_dir: Path, output_dir: Path, active_threshold: float, smooth_window_s: float) -> dict:
    cmd_path = csv_dir / "cmd_vel.csv"
    odom_path = csv_dir / "robot_odom.csv"
    if not cmd_path.exists() or not odom_path.exists():
        raise FileNotFoundError(f"Missing cmd_vel.csv or robot_odom.csv under {csv_dir}")

    cmd_df = pd.read_csv(cmd_path).sort_values("time").reset_index(drop=True)
    odom_df = pd.read_csv(odom_path).sort_values("time").reset_index(drop=True)

    cmd_df = trim_active_window(cmd_df, active_threshold)
    start_t = float(cmd_df["time"].iloc[0])
    end_t = float(cmd_df["time"].iloc[-1])

    cmd_df = cmd_df[(cmd_df["time"] >= start_t) & (cmd_df["time"] <= end_t)].copy()
    odom_df = odom_df[(odom_df["time"] >= start_t) & (odom_df["time"] <= end_t)].copy()

    cmd_df["time_s"] = cmd_df["time"] - start_t
    odom_df["time_s"] = odom_df["time"] - start_t

    cmd_df["cmd_speed"] = compute_speed(cmd_df, "vx_cmd", "vy_cmd")
    odom_df["odom_speed"] = compute_speed(odom_df, "vx", "vy")

    for col in ["vx_cmd", "vy_cmd", "cmd_speed"]:
        cmd_df[f"{col}_smooth"] = rolling_seconds(cmd_df[col], cmd_df["time_s"], smooth_window_s)
    for col in ["vx", "vy", "odom_speed"]:
        odom_df[f"{col}_smooth"] = rolling_seconds(odom_df[col], odom_df["time_s"], smooth_window_s)

    merged = pd.merge_asof(
        odom_df.sort_values("time_s"),
        cmd_df[
            [
                "time_s",
                "vx_cmd",
                "vy_cmd",
                "cmd_speed",
                "vx_cmd_smooth",
                "vy_cmd_smooth",
                "cmd_speed_smooth",
            ]
        ].sort_values("time_s"),
        on="time_s",
        direction="nearest",
        tolerance=0.1,
    )

    out_dir = output_dir / csv_dir.name
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd_df.to_csv(out_dir / "cmd_vel_processed.csv", index=False)
    odom_df.to_csv(out_dir / "robot_odom_processed.csv", index=False)
    merged.to_csv(out_dir / "smoothness_merged.csv", index=False)

    group = "baseline" if csv_dir.name.startswith("baseline_") else "full"
    return {
        "group": group,
        "csv_dir": csv_dir.name,
        "processed_dir": str(out_dir),
        "cmd_rows": len(cmd_df),
        "odom_rows": len(odom_df),
        "active_duration_s": round(end_t - start_t, 3),
        "cmd_speed_mean": round(float(cmd_df["cmd_speed"].mean()), 3),
        "cmd_speed_std": round(float(cmd_df["cmd_speed"].std()), 3),
        "odom_speed_mean": round(float(odom_df["odom_speed"].mean()), 3),
        "odom_speed_std": round(float(odom_df["odom_speed"].std()), 3),
        "wz_all_zero": bool(
            np.allclose(cmd_df["wz_cmd"].fillna(0.0).to_numpy(), 0.0)
            and np.allclose(odom_df["wz"].fillna(0.0).to_numpy(), 0.0)
        ),
    }


def main() -> int:
    parser = argparse.ArgumentParser(description="Prepare smoothness experiment CSVs for plotting.")
    parser.add_argument(
        "--input-root",
        default="/home/lraina/auto_shao/data/smoothness_bags",
        help="Directory that contains per-bag folders with *_csv exports.",
    )
    parser.add_argument(
        "--output-dir",
        default="ablation_eval_output/smoothness_processed",
        help="Directory for processed CSV outputs.",
    )
    parser.add_argument(
        "--active-threshold",
        type=float,
        default=0.05,
        help="Minimum command speed to define the active motion window.",
    )
    parser.add_argument(
        "--smooth-window-s",
        type=float,
        default=0.35,
        help="Rolling smoothing window in seconds for plot-friendly curves.",
    )
    args = parser.parse_args()

    input_root = Path(args.input_root)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    for bag_dir in sorted(input_root.iterdir()):
        if not bag_dir.is_dir():
            continue
        for csv_dir in sorted(p for p in bag_dir.iterdir() if p.is_dir() and p.name.endswith("_csv")):
            rows.append(process_one(csv_dir, output_dir, args.active_threshold, args.smooth_window_s))

    summary = pd.DataFrame(rows).sort_values(["group", "csv_dir"]).reset_index(drop=True)

    # Manual recommendation based on this round's data inspection:
    summary["recommended_for_paper"] = summary["csv_dir"].isin(
        [
            "baseline_smoothness_20260404_183836_csv",
            "full_smoothness_20260404_183932_csv",
        ]
    )
    summary.to_csv(output_dir / "smoothness_summary.csv", index=False)
    print(f"Saved {output_dir / 'smoothness_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
