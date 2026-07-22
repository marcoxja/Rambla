#!/usr/bin/env python3
"""Sample CPU%/RSS per process for the resource-budget benchmark.

Usage:
    python3 scripts/sample_resources.py --duration 60 --interval 1 --out sample.csv

Meant to run wherever the target processes live (Mac, rambla-vm, eventually
the Pi). For a remote VM with no copy of this file, pipe it over SSH stdin
instead of scp'ing it into the target's own git checkout:

    ssh rambla-vm python3 - --duration 60 --out /tmp/sample.csv \\
        < scripts/sample_resources.py
    scp rambla-vm:/tmp/sample.csv .

Watches a fixed set of process command-substrings (see PATTERNS below) and
tags each sample `sim_only` per the classification in
.claude/internal-docs/robot/resource-budget/CLAUDE.md's "Separating
sim-only cost from real-node cost" section: Gazebo + the ros_gz bridge/spawn
processes + covariance_injector are sim-only; everything else (EKF, safety,
control panel) has a real claim on the Pi budget. Writes one row per
(pid, interval) to CSV -- raw samples, not pre-aggregated -- so mean/max can
be computed afterward per the doc's methodology.
"""

import argparse
import csv
import json
import os
import sys
import time

import psutil

# (substring to match in the process's joined cmdline, sim_only, label)
PATTERNS = [
    ("gz sim", True, "gz_sim_server"),
    ("parameter_bridge", True, "ros_gz_bridge"),
    ("ros_gz_sim", True, "ros_gz_sim_create"),
    ("covariance_injector", True, "covariance_injector"),
    ("ekf_node", False, "ekf_filter_node"),
    ("safety_node", False, "safety_node"),
    # M4 (CTL-009): the FastAPI/uvicorn server.py this used to match was
    # retired -- the process is now the outbound-dialing gateway, relabeled
    # to match.
    ("rambla_control_panel", False, "gateway"),
    # M5 localization stack: real nodes (not sim-only), same classification
    # as EKF/safety -- this is what M5_PLAN.md's resource-budget
    # verification item samples.
    ("map_server", False, "map_server"),
    ("amcl", False, "amcl"),
    ("localization_monitor", False, "localization_monitor"),
    ("localization_probe", False, "localization_probe"),
]

# Bandwidth: the gateway writes its own per-channel
# byte-rate counters here (bandwidth_stats.py) since this script, as an
# external process, has no way to read them out of the gateway's memory --
# see gateway_client.py's RAMBLA_GATEWAY_STATS_FILE. Only ever attributed to
# the "gateway" row; every other tracked process gets blank columns.
GATEWAY_STATS_LABEL = "gateway"
STATS_STALE_AFTER_S = 5.0


def label_for(cmdline):
    for pattern, sim_only, label in PATTERNS:
        if pattern in cmdline:
            return sim_only, label
    return None


def read_gateway_stats(path):
    """Read the gateway's bandwidth_stats.py JSON file, if fresh.

    Returns None if the file doesn't exist or its timestamp is stale
    (gateway not running / not instrumented) rather than raising -- absent
    bandwidth numbers just means blank columns for that tick, not a script
    failure.
    """
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        return None
    if time.time() - data.get("timestamp", 0) > STATS_STALE_AFTER_S:
        return None
    return data


def refresh_targets(tracked):
    for p in psutil.process_iter(["pid", "cmdline"]):
        if p.info["pid"] in tracked:
            continue
        cmdline = " ".join(p.info["cmdline"] or [])
        match = label_for(cmdline)
        if match is None:
            continue
        sim_only, label = match
        try:
            proc = psutil.Process(p.info["pid"])
            proc.cpu_percent(interval=None)  # prime the internal delta timer
        except psutil.NoSuchProcess:
            continue
        tracked[p.info["pid"]] = (proc, sim_only, label)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--duration", type=float, default=60.0, help="seconds to sample (default 60)")
    parser.add_argument("--interval", type=float, default=1.0, help="seconds between samples (default 1)")
    parser.add_argument("--out", default="resource_samples.csv", help="CSV output path")
    parser.add_argument("--gateway-stats-file", default="/tmp/rambla_gateway_stats.json",
                         help="path the gateway writes bandwidth counters to "
                              "(RAMBLA_GATEWAY_STATS_FILE, see gateway_client.py)")
    args = parser.parse_args()

    tracked = {}
    refresh_targets(tracked)
    if not tracked:
        print("warning: no matching processes found yet -- will keep looking each tick", file=sys.stderr)

    time.sleep(args.interval)  # let the primed cpu_percent() timers accumulate a real delta

    # Secondary aggregate cross-check only -- host-wide
    # interface totals, not the per-channel number the gateway's own
    # counters provide. io_start/io_end bracket the whole sampling run.
    io_start = psutil.net_io_counters(pernic=False)
    run_start = time.time()

    rows = []
    start = time.time()
    while time.time() - start < args.duration:
        refresh_targets(tracked)  # picks up late/one-shot processes (e.g. ros_gz_sim create)
        loadavg_1, loadavg_5, loadavg_15 = os.getloadavg()
        mem = psutil.virtual_memory()
        timestamp = time.time()
        gateway_stats = read_gateway_stats(args.gateway_stats_file)

        for pid, (proc, sim_only, label) in list(tracked.items()):
            try:
                cpu_percent = proc.cpu_percent(interval=None)
                rss_bytes = proc.memory_info().rss
            except psutil.NoSuchProcess:
                del tracked[pid]
                continue
            is_gateway = label == GATEWAY_STATS_LABEL and gateway_stats is not None
            rows.append({
                "timestamp": timestamp,
                "pid": pid,
                "label": label,
                "sim_only": sim_only,
                "cpu_percent": cpu_percent,
                "rss_bytes": rss_bytes,
                "loadavg_1m": loadavg_1,
                "mem_used_bytes": mem.used,
                "mem_available_bytes": mem.available,
                "mem_total_bytes": mem.total,
                "ctrl_bytes_out_per_sec": gateway_stats["ctrl_bytes_out_per_sec"] if is_gateway else "",
                "ctrl_bytes_in_per_sec": gateway_stats["ctrl_bytes_in_per_sec"] if is_gateway else "",
                "video_bytes_out_per_sec": gateway_stats["video_bytes_out_per_sec"] if is_gateway else "",
            })

        time.sleep(args.interval)

    io_end = psutil.net_io_counters(pernic=False)
    run_elapsed = max(time.time() - run_start, 1e-6)

    with open(args.out, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "timestamp", "pid", "label", "sim_only", "cpu_percent",
            "rss_bytes", "loadavg_1m", "mem_used_bytes",
            "mem_available_bytes", "mem_total_bytes",
            "ctrl_bytes_out_per_sec", "ctrl_bytes_in_per_sec", "video_bytes_out_per_sec",
        ])
        writer.writeheader()
        writer.writerows(rows)

    print(f"wrote {len(rows)} rows across {len(tracked)} tracked processes to {args.out}", file=sys.stderr)
    print(
        "secondary cross-check (host-wide net_io_counters, all interfaces): "
        f"{(io_end.bytes_sent - io_start.bytes_sent) / run_elapsed:.0f} bytes/sec sent, "
        f"{(io_end.bytes_recv - io_start.bytes_recv) / run_elapsed:.0f} bytes/sec recv "
        "-- aggregate only, not the per-channel gateway figure above",
        file=sys.stderr,
    )


if __name__ == "__main__":
    main()
