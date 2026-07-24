"""
Robustness sweep -- scores a single gain set against many independent,
reproducible sensor-noise realizations, instead of the handful of anecdotal
seeds used for spot-checking during development. A Monte Carlo *search*
trial only ever sees one noise draw per candidate; this answers a different
question -- "given the winning gains, how often do they actually hold up?" --
which is the number that should drive a go/no-go decision on real hardware.

Usage:
    python -m tools.robustness                       # gains from Drone_Main/main.h
    python -m tools.robustness --seeds 200
    python -m tools.robustness --gains-file path/to/winning_gains.txt
"""

import argparse
import csv
import os
import time
from datetime import datetime

import numpy as np

from sim.vehicle import load_vehicle
from sim.motor_model import MotorModel
from sim.physics_engine import PhysicsEngine
from tools.tune_gains import evaluate, DEFAULT_MISSION, SAFETY_HALT_RAD, _AXES

_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'outputs', 'robustness')

_METRICS = ['cost', 'roll_rms_deg', 'pitch_rms_deg', 'yaw_err_rms_deg',
           'alt_rms_cm', 'xy_rms_cm', 'jerk_pct', 'progress']


def sweep(gains: dict, waypoints=None, dt: float = 0.004, max_duration: float = 45.0,
          n_seeds: int = 100, seed_offset: int = 0, verbose: bool = True) -> tuple:
    """
    Runs `gains` through `waypoints` n_seeds times, once per reproducible
    noise seed (seed_offset .. seed_offset+n_seeds-1). Returns (summary, results)
    where results is the list of raw per-seed info dicts and summary is the
    aggregated statistics used for the report/plot.
    """
    waypoints = waypoints or DEFAULT_MISSION
    vehicle = load_vehicle(verbose=False)
    motor   = MotorModel()
    engine  = PhysicsEngine(vehicle, motor)

    results = []
    t0 = time.time()
    for i in range(n_seeds):
        seed = seed_offset + i
        cost, info = evaluate(gains, engine, waypoints, dt, max_duration, seed=seed)
        info = dict(info)
        info['seed'] = seed
        results.append(info)
        if verbose and (i + 1) % 20 == 0:
            print(f"  [{i+1:>4}/{n_seeds}]  elapsed={time.time()-t0:5.0f}s")

    summary = _summarize(results)
    if verbose:
        print(f"\n[robustness] {n_seeds} seeds in {time.time()-t0:.0f}s")
        print_report(summary)

    return summary, results


def _summarize(results: list) -> dict:
    n = len(results)
    n_diverged      = sum(r['diverged'] for r in results)
    n_safety_halted = sum(r['safety_halted'] for r in results)
    n_landed        = sum(r['landed'] for r in results)
    # "safe" = never lost real-hardware motor authority, regardless of
    # whether it finished the mission in time -- this is the number that
    # actually matters for "will this crash", separate from "did it land".
    n_safe = sum(not (r['diverged'] or r['safety_halted']) for r in results)

    summary = dict(
        n_seeds=n,
        n_diverged=n_diverged,           pct_diverged=100.0 * n_diverged / n,
        n_safety_halted=n_safety_halted, pct_safety_halted=100.0 * n_safety_halted / n,
        n_landed=n_landed,               pct_landed=100.0 * n_landed / n,
        n_safe=n_safe,                   pct_safe=100.0 * n_safe / n,
    )

    for m in _METRICS:
        vals = np.array([r[m] for r in results], dtype=float)
        summary[m] = dict(
            mean=float(np.mean(vals)), std=float(np.std(vals)),
            min=float(np.min(vals)), max=float(np.max(vals)),
            p50=float(np.percentile(vals, 50)), p90=float(np.percentile(vals, 90)),
        )
    return summary


def print_report(summary: dict):
    print("\n--- Robustness sweep ---")
    print(f"  seeds tested        : {summary['n_seeds']}")
    print(f"  SAFE (no divergence, no 45deg safety-halt) : "
          f"{summary['n_safe']}/{summary['n_seeds']}  ({summary['pct_safe']:.0f}%)")
    print(f"  diverged (>90deg tumble)                   : "
          f"{summary['n_diverged']}/{summary['n_seeds']}  ({summary['pct_diverged']:.0f}%)")
    print(f"  tripped 45deg safety-halt                  : "
          f"{summary['n_safety_halted']}/{summary['n_seeds']}  ({summary['pct_safety_halted']:.0f}%)")
    print(f"  landed (finished mission)                  : "
          f"{summary['n_landed']}/{summary['n_seeds']}  ({summary['pct_landed']:.0f}%)")

    print("\n  metric               mean     std      p50      p90      max")
    labels = dict(cost='cost', roll_rms_deg='roll RMS [deg]', pitch_rms_deg='pitch RMS [deg]',
                 yaw_err_rms_deg='yaw err RMS [deg]', alt_rms_cm='alt RMS [cm]',
                 xy_rms_cm='xy RMS [cm]', jerk_pct='jerk [%/step]', progress='mission progress')
    for m in _METRICS:
        s = summary[m]
        print(f"  {labels[m]:<20} {s['mean']:8.2f} {s['std']:8.2f} {s['p50']:8.2f} {s['p90']:8.2f} {s['max']:8.2f}")

    if summary['pct_safe'] < 100.0:
        print(f"\n  [!] {100.0 - summary['pct_safe']:.0f}% of noise draws were UNSAFE for these gains "
              f"-- do not fly without addressing this.")


def save_outputs(results: list, summary: dict, gains: dict, out_dir: str = None) -> str:
    out_dir = out_dir or os.path.join(_OUTPUT_DIR, datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(out_dir, exist_ok=True)

    csv_path = os.path.join(out_dir, 'sweep_results.csv')
    fieldnames = list(results[0].keys()) if results else []
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(results)

    summary_path = os.path.join(out_dir, 'summary.txt')
    with open(summary_path, 'w') as f:
        f.write(f"SAFE: {summary['n_safe']}/{summary['n_seeds']} ({summary['pct_safe']:.0f}%)\n")
        f.write(f"diverged: {summary['pct_diverged']:.0f}%  safety_halted: {summary['pct_safety_halted']:.0f}%  "
                f"landed: {summary['pct_landed']:.0f}%\n\n")
        f.write("\n--- Gains tested ---\n")
        for axis, keys in _AXES.items():
            vals = "  ".join(f"{k}={gains[k]:.4f}" for k in keys)
            f.write(f"  {axis:<6}: {vals}\n")

    return out_dir


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Robustness sweep for a fixed gain set')
    parser.add_argument('--seeds', type=int, default=100, help='Number of noise seeds to test')
    parser.add_argument('--seed-offset', type=int, default=0)
    parser.add_argument('--dt', type=float, default=0.004)
    parser.add_argument('--duration', type=float, default=45.0)
    parser.add_argument('--gains-file', default=None,
                        help='Path to a winning_gains.txt-style file; default is Drone_Main/main.h')
    args = parser.parse_args()

    if args.gains_file:
        import re
        text = open(args.gains_file).read()
        # Anchored to k[pid]_* so this only picks up actual gains, not the
        # header comment fields winning_gains.txt also writes (cost=...,
        # roll_rms_deg=..., etc.) -- those happen to match "word = number"
        # too and were silently polluting the returned dict with junk keys
        # (harmless since DroneController only reads known kp_*/ki_*/kd_*
        # keys by name, but confusing in the printed/logged gains).
        gains = {m.group(1): float(m.group(2)) for m in
                 re.finditer(r'\b(k[pid]_\w+)\s*=\s*([\d.eE+-]+)f?', text)}
    else:
        from controller.arduino_parser import parse_gains
        gains = parse_gains(verbose=True)

    print(f"\n[robustness] Testing gains: {gains}")
    summary, results = sweep(gains, n_seeds=args.seeds, seed_offset=args.seed_offset,
                             dt=args.dt, max_duration=args.duration, verbose=True)
    out_dir = save_outputs(results, summary, gains)
    print(f"\n[robustness] Results saved to {out_dir}")
