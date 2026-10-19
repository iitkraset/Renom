"""Bench run: powers the MK-1A, runs flightsw.controller.main(board) for one mission and prints
the bench report.

usage: python run.py [--seed N] [--json]
"""
import argparse
import importlib
import json
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from flightsw import board as bsp  # noqa: E402


def fmt(v):
    if v is None:
        return "-"
    if isinstance(v, float):
        return "nan" if math.isnan(v) else "%.2f" % v
    return str(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--module", default="flightsw.controller", help="module providing main(board)")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()
    mod = importlib.import_module(a.module)
    print("MK-1A bench run, seed %d (about 40 s)..." % a.seed, flush=True)
    r = bsp.run_mission(mod.main, seed=a.seed, outdir=os.path.join(os.path.dirname(os.path.abspath(__file__)), "out"))
    if a.json:
        print(json.dumps(r, indent=1, default=str))
        return
    groups = [
        ("Control loop", ["loop_hz", "loop_p50_ms", "loop_p99_ms", "loop_max_ms", "deadline_misses"]),
        ("Sensor -> fin latency", ["act_latency_p50_ms", "act_latency_p99_ms", "act_latency_max_ms",
                                   "unstamped_cmds", "yaw_rms_deg", "yaw_max_deg"]),
        ("Flight events", ["launch_detect_ms", "true_apogee_s", "deploy_s", "deploy_after_apogee_ms",
                           "deploy_speed_mps", "chute_ok", "landed_s", "landed_detect_s", "state_seq"]),
        ("Telemetry (ground station)", ["tm_frames_ok", "tm_frames_bad", "tm_samples", "tm_coverage",
                                        "tm_alt_err_p95_m", "tm_stale_samples"]),
        ("SD log (after power cut)", ["sd_lines", "sd_coverage", "sd_max_gap_ms"]),
        ("Health", ["bus_collisions", "thread_errors", "threads_left", "imu_errors", "mem_peak_kib"]),
    ]
    for title, keys in groups:
        print("\n" + title)
        for k in keys:
            print("  %-24s %s" % (k, fmt(r.get(k))))


if __name__ == "__main__":
    main()
