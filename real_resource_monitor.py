#!/usr/bin/env python3
import argparse
import csv
import os
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Set, Tuple


CLK_TCK = os.sysconf(os.sysconf_names["SC_CLK_TCK"])
PAGE_SIZE = os.sysconf("SC_PAGE_SIZE")


PROCESS_PATTERNS: Dict[str, Tuple[str, ...]] = {
    "livox_driver": ("livox_ros_driver2_node",),
    "point_lio": ("pointlio_mapping",),
    "odom_bridge": ("odom_to_base_node.py",),
    "lidar_filter": ("rm_lidar_filter", "lidar_filter"),
    "segmentation": ("ground_segmentation_node",),
    "tracker": ("predictive_tracker", "dynamic_tracker"),
    "scan_bridge": ("pointcloud_to_laserscan",),
    "icp": ("icp_registration_node",),
    "map_server": ("map_server",),
    "planner_server": ("planner_server",),
    "controller_server": ("controller_server",),
    "bt_navigator": ("bt_navigator",),
    "recoveries_server": ("recoveries_server",),
    "waypoint_follower": ("waypoint_follower",),
    "lifecycle_manager": ("lifecycle_manager",),
    "rviz": ("rviz2",),
}

NAV_COMPONENTS = {
    "icp",
    "map_server",
    "planner_server",
    "controller_server",
    "bt_navigator",
    "recoveries_server",
    "waypoint_follower",
    "lifecycle_manager",
}


@dataclass
class ProcSnapshot:
    pid: int
    ppid: int
    comm: str
    cmdline: str
    total_time_ticks: int
    rss_bytes: int


def read_proc_snapshot(pid: int) -> Optional[ProcSnapshot]:
    stat_path = Path(f"/proc/{pid}/stat")
    cmdline_path = Path(f"/proc/{pid}/cmdline")
    try:
        stat_text = stat_path.read_text()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        return None

    lpar = stat_text.find("(")
    rpar = stat_text.rfind(")")
    if lpar == -1 or rpar == -1:
        return None

    comm = stat_text[lpar + 1:rpar]
    fields = stat_text[rpar + 2:].split()
    if len(fields) < 22:
        return None

    try:
        ppid = int(fields[1])
        utime = int(fields[11])
        stime = int(fields[12])
        rss_pages = int(fields[21])
    except ValueError:
        return None

    try:
        cmdline_raw = cmdline_path.read_bytes()
        cmdline = cmdline_raw.replace(b"\x00", b" ").decode("utf-8", errors="replace").strip()
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        cmdline = comm

    return ProcSnapshot(
        pid=pid,
        ppid=ppid,
        comm=comm,
        cmdline=cmdline or comm,
        total_time_ticks=utime + stime,
        rss_bytes=rss_pages * PAGE_SIZE,
    )


def list_all_snapshots() -> Dict[int, ProcSnapshot]:
    snapshots: Dict[int, ProcSnapshot] = {}
    for entry in os.listdir("/proc"):
        if not entry.isdigit():
            continue
        snap = read_proc_snapshot(int(entry))
        if snap is not None:
            snapshots[snap.pid] = snap
    return snapshots


def get_children_map(snapshots: Dict[int, ProcSnapshot]) -> Dict[int, List[int]]:
    children: Dict[int, List[int]] = {}
    for snap in snapshots.values():
        children.setdefault(snap.ppid, []).append(snap.pid)
    return children


def expand_descendants(root_pids: Iterable[int], children_map: Dict[int, List[int]]) -> Set[int]:
    seen: Set[int] = set()
    queue = list(root_pids)
    while queue:
        pid = queue.pop()
        if pid in seen:
            continue
        seen.add(pid)
        queue.extend(children_map.get(pid, []))
    return seen


def infer_component(cmdline: str, comm: str) -> str:
    text = f"{comm} {cmdline}"
    for component, patterns in PROCESS_PATTERNS.items():
        if all(pattern in text for pattern in patterns):
            return component
        if any(pattern in text for pattern in patterns):
            return component
    return "other"


def read_mem_total_bytes() -> int:
    with open("/proc/meminfo", "r", encoding="utf-8") as f:
        for line in f:
            if line.startswith("MemTotal:"):
                parts = line.split()
                return int(parts[1]) * 1024
    raise RuntimeError("Failed to read MemTotal from /proc/meminfo")


def now_stamp() -> str:
    return time.strftime("%Y%m%d_%H%M%S")


def start_headless_script(
    script_path: Path,
    log_dir: Path,
    pid_file: Path,
    extra_env: Optional[Dict[str, str]] = None,
) -> subprocess.Popen:
    env = os.environ.copy()
    env["AUTO_EVAL_HEADLESS"] = "1"
    env["AUTO_EVAL_PID_FILE"] = str(pid_file)
    env["AUTO_EVAL_LOG_DIR"] = str(log_dir)
    env["AUTO_EVAL_SKIP_RVIZ"] = "1"
    if extra_env:
        env.update(extra_env)

    return subprocess.Popen(
        ["/bin/bash", str(script_path)],
        cwd=str(script_path.parent),
        env=env,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        start_new_session=True,
    )


def wait_for_pid_file(pid_file: Path, timeout_s: float = 20.0) -> List[int]:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if pid_file.exists():
            pids = []
            for line in pid_file.read_text().splitlines():
                line = line.strip()
                if line.isdigit():
                    pids.append(int(line))
            if pids:
                return pids
        time.sleep(0.5)
    raise RuntimeError(f"Timed out waiting for PID file: {pid_file}")


def terminate_pid_tree(root_pids: Sequence[int]) -> None:
    snapshots = list_all_snapshots()
    children_map = get_children_map(snapshots)
    all_pids = sorted(expand_descendants(root_pids, children_map), reverse=True)

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        for pid in all_pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass
            except PermissionError:
                pass
        time.sleep(1.0)
        snapshots = list_all_snapshots()
        alive = [pid for pid in all_pids if pid in snapshots]
        if not alive:
            return


def discover_target_pids(root_pids: Optional[Sequence[int]]) -> Dict[int, ProcSnapshot]:
    snapshots = list_all_snapshots()
    if root_pids:
        children_map = get_children_map(snapshots)
        target_pids = expand_descendants(root_pids, children_map)
        return {pid: snap for pid, snap in snapshots.items() if pid in target_pids}

    selected: Dict[int, ProcSnapshot] = {}
    for pid, snap in snapshots.items():
        component = infer_component(snap.cmdline, snap.comm)
        if component != "other":
            selected[pid] = snap
    return selected


def sample_usage(
    method: str,
    root_pids: Optional[Sequence[int]],
    duration_s: float,
    interval_s: float,
    output_dir: Path,
) -> Tuple[Path, Path]:
    samples_path = output_dir / f"{method}_resource_samples.csv"
    summary_path = output_dir / f"{method}_resource_summary.csv"
    mem_total = read_mem_total_bytes()
    cpu_count = max(1, os.cpu_count() or 1)

    prev_wall = time.monotonic()
    prev_snapshots = discover_target_pids(root_pids)
    rows: List[Dict[str, object]] = []
    aggregate_rows: List[Dict[str, float]] = []

    deadline = prev_wall + duration_s
    while time.monotonic() < deadline:
        time.sleep(interval_s)
        current_wall = time.monotonic()
        current_snapshots = discover_target_pids(root_pids)
        elapsed = max(current_wall - prev_wall, 1e-6)

        total_cpu = 0.0
        total_rss = 0
        nav_cpu = 0.0
        nav_rss = 0

        for pid, snap in current_snapshots.items():
            prev = prev_snapshots.get(pid)
            if prev is None:
                cpu_percent = 0.0
            else:
                delta_ticks = max(0, snap.total_time_ticks - prev.total_time_ticks)
                cpu_percent = 100.0 * (delta_ticks / CLK_TCK) / (elapsed * cpu_count)

            rss_mb = snap.rss_bytes / (1024.0 * 1024.0)
            mem_percent = 100.0 * snap.rss_bytes / mem_total
            component = infer_component(snap.cmdline, snap.comm)

            rows.append(
                {
                    "timestamp_s": round(current_wall, 3),
                    "method": method,
                    "pid": pid,
                    "component": component,
                    "comm": snap.comm,
                    "cpu_percent": round(cpu_percent, 4),
                    "rss_mb": round(rss_mb, 4),
                    "mem_percent": round(mem_percent, 6),
                    "cmdline": snap.cmdline,
                }
            )

            total_cpu += cpu_percent
            total_rss += snap.rss_bytes
            if component in NAV_COMPONENTS:
                nav_cpu += cpu_percent
                nav_rss += snap.rss_bytes

        aggregate_rows.append(
            {
                "stack_cpu_percent": total_cpu,
                "stack_rss_mb": total_rss / (1024.0 * 1024.0),
                "stack_mem_percent": 100.0 * total_rss / mem_total,
                "nav_cpu_percent": nav_cpu,
                "nav_rss_mb": nav_rss / (1024.0 * 1024.0),
                "nav_mem_percent": 100.0 * nav_rss / mem_total,
                "process_count": float(len(current_snapshots)),
            }
        )

        prev_wall = current_wall
        prev_snapshots = current_snapshots

    with samples_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=[
                "timestamp_s",
                "method",
                "pid",
                "component",
                "comm",
                "cpu_percent",
                "rss_mb",
                "mem_percent",
                "cmdline",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)

    def mean(values: Sequence[float]) -> float:
        return sum(values) / len(values) if values else 0.0

    summary = {
        "method": method,
        "duration_s": duration_s,
        "sample_interval_s": interval_s,
        "sample_count": len(aggregate_rows),
        "stack_cpu_percent_mean": mean([r["stack_cpu_percent"] for r in aggregate_rows]),
        "stack_cpu_percent_peak": max((r["stack_cpu_percent"] for r in aggregate_rows), default=0.0),
        "stack_rss_mb_mean": mean([r["stack_rss_mb"] for r in aggregate_rows]),
        "stack_rss_mb_peak": max((r["stack_rss_mb"] for r in aggregate_rows), default=0.0),
        "stack_mem_percent_mean": mean([r["stack_mem_percent"] for r in aggregate_rows]),
        "stack_mem_percent_peak": max((r["stack_mem_percent"] for r in aggregate_rows), default=0.0),
        "nav_cpu_percent_mean": mean([r["nav_cpu_percent"] for r in aggregate_rows]),
        "nav_cpu_percent_peak": max((r["nav_cpu_percent"] for r in aggregate_rows), default=0.0),
        "nav_rss_mb_mean": mean([r["nav_rss_mb"] for r in aggregate_rows]),
        "nav_rss_mb_peak": max((r["nav_rss_mb"] for r in aggregate_rows), default=0.0),
        "nav_mem_percent_mean": mean([r["nav_mem_percent"] for r in aggregate_rows]),
        "nav_mem_percent_peak": max((r["nav_mem_percent"] for r in aggregate_rows), default=0.0),
        "process_count_mean": mean([r["process_count"] for r in aggregate_rows]),
    }

    with summary_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(summary.keys()))
        writer.writeheader()
        writer.writerow({k: round(v, 6) if isinstance(v, float) else v for k, v in summary.items()})

    return samples_path, summary_path


def append_comparison_row(
    comparison_csv: Path,
    method: str,
    summary_csv: Path,
) -> None:
    with summary_csv.open("r", newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        row = next(reader)

    write_header = not comparison_csv.exists()
    with comparison_csv.open("a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=list(row.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Monitor real-robot CPU/memory usage for Baseline / Full navigation methods."
    )
    parser.add_argument("--method", choices=["baseline", "full"], required=True,
                        help="Method label for this recording session.")
    parser.add_argument("--duration", type=float, default=60.0,
                        help="Sampling duration in seconds.")
    parser.add_argument("--interval", type=float, default=0.5,
                        help="Sampling interval in seconds.")
    parser.add_argument("--output-dir", default="real_resource_eval_output",
                        help="Directory to save CSV outputs.")
    parser.add_argument("--launch-stack", action="store_true",
                        help="Launch pre_real/nav_real automatically in headless mode before sampling.")
    parser.add_argument("--pre-script", default="./pre_real.sh",
                        help="Real pre-stack script.")
    parser.add_argument("--nav-script", default="./nav_real.sh",
                        help="Real nav script.")
    parser.add_argument("--warmup", type=float, default=10.0,
                        help="Warmup time after launch before sampling.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parent
    output_dir = (repo_root / args.output_dir / f"{args.method}_{now_stamp()}").resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    root_pids: Optional[List[int]] = None
    launcher_processes: List[subprocess.Popen] = []
    pid_files: List[Path] = []

    try:
        if args.launch_stack:
            pre_script = (repo_root / args.pre_script).resolve()
            nav_script = (repo_root / args.nav_script).resolve()

            pre_pid_file = output_dir / "pre_real_pids.txt"
            nav_pid_file = output_dir / "nav_real_pids.txt"
            pre_log_dir = output_dir / "pre_real_logs"
            nav_log_dir = output_dir / "nav_real_logs"

            launcher_processes.append(start_headless_script(pre_script, pre_log_dir, pre_pid_file))
            pid_files.append(pre_pid_file)

            nav_env = {}
            if args.method == "baseline":
                nav_env["REAL_NAV_PARAMS_FILE"] = str(
                    (repo_root / "rm_navi/rm_navigation/navi/params/nav2_params_real_baseline.yaml").resolve()
                )
            else:
                nav_env["REAL_NAV_PARAMS_FILE"] = str(
                    (repo_root / "rm_navi/rm_navigation/navi/params/nav2_params_real_full.yaml").resolve()
                )

            launcher_processes.append(start_headless_script(nav_script, nav_log_dir, nav_pid_file, nav_env))
            pid_files.append(nav_pid_file)

            root_pids = []
            for pid_file in pid_files:
                root_pids.extend(wait_for_pid_file(pid_file))

            print(f"[INFO] launched stack for {args.method}, warmup {args.warmup:.1f}s")
            time.sleep(max(0.0, args.warmup))
        else:
            print("[INFO] attach mode: sampling currently running stack")

        samples_csv, summary_csv = sample_usage(
            method=args.method,
            root_pids=root_pids,
            duration_s=args.duration,
            interval_s=args.interval,
            output_dir=output_dir,
        )

        comparison_csv = (repo_root / args.output_dir / "real_resource_comparison.csv").resolve()
        append_comparison_row(comparison_csv, args.method, summary_csv)

        print(f"[INFO] saved samples to {samples_csv}")
        print(f"[INFO] saved summary to {summary_csv}")
        print(f"[INFO] updated comparison table {comparison_csv}")
        return 0
    finally:
        if root_pids:
            terminate_pid_tree(root_pids)
        for proc in launcher_processes:
            if proc.poll() is None:
                try:
                    proc.terminate()
                except ProcessLookupError:
                    pass


if __name__ == "__main__":
    sys.exit(main())
