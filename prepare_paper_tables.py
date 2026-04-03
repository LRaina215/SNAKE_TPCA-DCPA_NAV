#!/usr/bin/env python3

from pathlib import Path
from typing import Dict, List

import pandas as pd


ROOT = Path(__file__).resolve().parent
OUTPUT_DIR = ROOT / "ablation_eval_output"
GROUP_ORDER = ["Baseline", "RiskOnly", "Full", "TEB"]
SCENARIO_ORDER = ["dynamic_test", "narrow_corridor", "random_crowd"]


def _ordered_categorical(series: pd.Series, order: List[str]) -> pd.Series:
    return pd.Categorical(series, categories=order, ordered=True)


def _read_csv(path: Path) -> pd.DataFrame:
    if not path.exists():
        raise FileNotFoundError(f"Missing input CSV: {path}")
    return pd.read_csv(path)


def _round_columns(df: pd.DataFrame, digits_map: Dict[str, int]) -> pd.DataFrame:
    rounded = df.copy()
    for column, digits in digits_map.items():
        if column in rounded.columns:
            rounded[column] = rounded[column].round(digits)
    return rounded


def build_dynamic_test_table(trials_df: pd.DataFrame, summary_df: pd.DataFrame) -> pd.DataFrame:
    trial_counts = (
        trials_df.groupby("group", dropna=False)
        .agg(
            trial_count=("trial_index", "count"),
            success_count=("success", "sum"),
        )
        .reset_index()
    )
    merged = summary_df.merge(trial_counts, on="group", how="left")

    paper_df = merged[
        [
            "group",
            "trial_count",
            "success_count",
            "success_rate_pct_mean",
            "mean_navigation_time_s_mean",
            "average_translational_speed_mps_mean",
            "algorithm_latency_ms_mean",
            "algorithm_latency_source",
        ]
    ].copy()
    paper_df = paper_df.rename(
        columns={
            "success_rate_pct_mean": "goal_reaching_rate_pct",
            "mean_navigation_time_s_mean": "mean_navigation_time_s",
            "average_translational_speed_mps_mean": "average_translational_speed_mps",
            "algorithm_latency_ms_mean": "algorithm_latency_ms",
        }
    )
    paper_df["group"] = _ordered_categorical(paper_df["group"], GROUP_ORDER)
    paper_df = paper_df.sort_values("group").reset_index(drop=True)
    paper_df = _round_columns(
        paper_df,
        {
            "goal_reaching_rate_pct": 1,
            "mean_navigation_time_s": 3,
            "average_translational_speed_mps": 3,
            "algorithm_latency_ms": 3,
        },
    )
    paper_df["group"] = paper_df["group"].astype(str)
    return paper_df


def build_overhead_table(summary_df: pd.DataFrame) -> pd.DataFrame:
    paper_df = summary_df[
        [
            "group",
            "tracker_latency_ms_mean",
            "algorithm_latency_ms_mean",
            "algorithm_latency_source",
        ]
    ].copy()
    paper_df = paper_df.rename(
        columns={
            "tracker_latency_ms_mean": "tracker_latency_ms",
            "algorithm_latency_ms_mean": "algorithm_latency_ms",
        }
    )
    paper_df["group"] = _ordered_categorical(paper_df["group"], GROUP_ORDER)
    paper_df = paper_df.sort_values("group").reset_index(drop=True)
    paper_df = _round_columns(
        paper_df,
        {
            "tracker_latency_ms": 3,
            "algorithm_latency_ms": 3,
        },
    )
    paper_df["group"] = paper_df["group"].astype(str)
    return paper_df


def build_multi_scenario_table(trials_df: pd.DataFrame, summary_df: pd.DataFrame) -> pd.DataFrame:
    trial_counts = (
        trials_df.groupby(["scenario", "group"], dropna=False)
        .agg(
            trial_count=("trial_index", "count"),
            success_count=("success", "sum"),
        )
        .reset_index()
    )
    merged = summary_df.merge(trial_counts, on=["scenario", "group"], how="left")
    paper_df = merged[
        [
            "scenario",
            "group",
            "trial_count",
            "success_count",
            "success_rate_pct_mean",
        ]
    ].copy()
    paper_df = paper_df.rename(columns={"success_rate_pct_mean": "goal_reaching_rate_pct"})
    paper_df["scenario"] = _ordered_categorical(paper_df["scenario"], SCENARIO_ORDER)
    paper_df["group"] = _ordered_categorical(paper_df["group"], GROUP_ORDER)
    paper_df = paper_df.sort_values(["scenario", "group"]).reset_index(drop=True)
    paper_df = _round_columns(paper_df, {"goal_reaching_rate_pct": 1})
    paper_df["scenario"] = paper_df["scenario"].astype(str)
    paper_df["group"] = paper_df["group"].astype(str)
    return paper_df


def build_multi_scenario_wide_table(multi_table: pd.DataFrame) -> pd.DataFrame:
    wide_df = multi_table.pivot(
        index="group",
        columns="scenario",
        values="goal_reaching_rate_pct",
    ).reset_index()
    preferred_columns = ["group"] + [c for c in SCENARIO_ORDER if c in wide_df.columns]
    wide_df = wide_df[preferred_columns]
    wide_df["group"] = _ordered_categorical(wide_df["group"], GROUP_ORDER)
    wide_df = wide_df.sort_values("group").reset_index(drop=True)
    wide_df["group"] = wide_df["group"].astype(str)
    return wide_df


def main() -> int:
    ablation_trials = _read_csv(OUTPUT_DIR / "ablation_trials.csv")
    ablation_summary = _read_csv(OUTPUT_DIR / "ablation_results.csv")

    dynamic_table = build_dynamic_test_table(ablation_trials, ablation_summary)
    overhead_table = build_overhead_table(ablation_summary)

    dynamic_path = OUTPUT_DIR / "paper_dynamic_test_table.csv"
    overhead_path = OUTPUT_DIR / "paper_overhead_table.csv"
    dynamic_table.to_csv(dynamic_path, index=False)
    overhead_table.to_csv(overhead_path, index=False)
    print(f"Saved {dynamic_path}")
    print(f"Saved {overhead_path}")

    multi_trials_path = OUTPUT_DIR / "multi_scenario_trials.csv"
    multi_summary_path = OUTPUT_DIR / "multi_scenario_results.csv"
    if multi_trials_path.exists() and multi_summary_path.exists():
        multi_trials = _read_csv(multi_trials_path)
        multi_summary = _read_csv(multi_summary_path)
        multi_table = build_multi_scenario_table(multi_trials, multi_summary)
        multi_path = OUTPUT_DIR / "paper_multi_scenario_table.csv"
        multi_wide_path = OUTPUT_DIR / "paper_multi_scenario_wide.csv"
        multi_table.to_csv(multi_path, index=False)
        build_multi_scenario_wide_table(multi_table).to_csv(multi_wide_path, index=False)
        print(f"Saved {multi_path}")
        print(f"Saved {multi_wide_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
