"""
Monte Carlo gain tuner.

Randomly samples full 18-gain sets, scores each one by flying the tuning
mission in the physics sim, and keeps the best. Two phases:

  1. Global search  -- uniform random samples across wide bounds, to find a
                        good region of gain-space and reject unstable ones.
  2. Local refinement -- Gaussian perturbations around the best-so-far, with
                        the perturbation radius shrinking over time (simple
                        simulated-annealing-style local search), to polish it.

Works with whichever controller backend is active (compiled DLL or the pure
-Python fallback) -- it only touches DroneController's public interface
(add_waypoint / reset / update / mission_complete / current_waypoint_index),
so it never has to know which one it's driving.
"""

import csv
import os
import time
from datetime import datetime

import numpy as np

from sim.vehicle import load_vehicle
from sim.motor_model import MotorModel
from sim.physics_engine import PhysicsEngine
from sim.timing import ControlLoopTiming
from controller.pid_controller import DroneController
from controller.waypoint_controller import Waypoint

_OUTPUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'outputs', 'tuning')

# ---------------------------------------------------------------------------
# Default mission -- matches Drone_Main/Waypoints.h exactly, so gains tuned
# here are being evaluated against the same mission the real firmware flies.
# Hover altitude capped at 0.4 m: bench testing found the VL53L1X rangefinder
# reads accurately near 0.19 m but showed a growing undershoot at 0.58 m+
# (see Waypoints.h), so the mission stays well clear of that degraded range.
# ---------------------------------------------------------------------------
DEFAULT_MISSION = [
    Waypoint(0.0, 0.0, 0.4, True),
    Waypoint(1.0, 0.0, 0.4, False),
    Waypoint(1.0, 1.0, 0.4, False),
    Waypoint(0.0, 0.0, 0.4, True),
    Waypoint(0.0, 0.0, 0.0, True),
]

# Current Drone_Main/main.h defaults -- used as the starting point.
DEFAULT_GAINS = dict(
    kp_roll=3.5,  ki_roll=0.04,  kd_roll=0.15,
    kp_pitch=3.5, ki_pitch=0.04, kd_pitch=0.15,
    kp_yaw=2.0,   ki_yaw=0.02,   kd_yaw=0.08,
    kp_x=0.25,    ki_x=0.005,    kd_x=0.15,
    kp_y=0.25,    ki_y=0.005,    kd_y=0.15,
    kp_z=0.40,    ki_z=0.02,     kd_z=0.20,
)

# Search bounds per gain (lo, hi). Hand-picked engineering ranges around the
# defaults -- wide enough to find something meaningfully better, narrow
# enough that most samples are at least plausible PID gains.
GAIN_BOUNDS = dict(
    kp_roll=(0.5, 8.0),   ki_roll=(0.0, 0.30),  kd_roll=(0.0, 0.60),
    kp_pitch=(0.5, 8.0),  ki_pitch=(0.0, 0.30), kd_pitch=(0.0, 0.60),
    kp_yaw=(0.3, 5.0),    ki_yaw=(0.0, 0.20),   kd_yaw=(0.0, 0.40),
    kp_x=(0.05, 1.0),     ki_x=(0.0, 0.05),     kd_x=(0.0, 0.60),
    kp_y=(0.05, 1.0),     ki_y=(0.0, 0.05),     kd_y=(0.0, 0.60),
    kp_z=(0.10, 1.5),     ki_z=(0.0, 0.10),     kd_z=(0.0, 0.80),
)

_AXES = {
    'roll':  ['kp_roll', 'ki_roll', 'kd_roll'],
    'pitch': ['kp_pitch', 'ki_pitch', 'kd_pitch'],
    'yaw':   ['kp_yaw', 'ki_yaw', 'kd_yaw'],
    'x':     ['kp_x', 'ki_x', 'kd_x'],
    'y':     ['kp_y', 'ki_y', 'kd_y'],
    'z':     ['kp_z', 'ki_z', 'kd_z'],
}

# Real hardware cuts the motors above this tilt (controller.cpp / pid_controller.py
# safety cut, 0.785 rad = 45 deg) -- a trial that crosses it would have lost power
# on the real drone even if the *rigid-body* angle never reaches the 90 deg
# "diverged" threshold below. Tracked as its own cost tier, see evaluate().
SAFETY_HALT_RAD = 0.785


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def _leg_yaw_targets(waypoints):
    """Mirrors Drone_Main.ino::_startLeg() -- bearing from the previous
    waypoint (or origin) to each waypoint, for keep_heading=False legs."""
    targets = []
    for i, wp in enumerate(waypoints):
        if wp.keep_heading:
            targets.append(0.0)
            continue
        fx, fy = (0.0, 0.0) if i == 0 else (waypoints[i - 1].x, waypoints[i - 1].y)
        dx, dy = wp.x - fx, wp.y - fy
        targets.append(float(np.arctan2(dy, dx)) if dx * dx + dy * dy > 1e-6 else 0.0)
    return targets


def sample_gains(rng: np.random.Generator) -> dict:
    return {k: float(rng.uniform(lo, hi)) for k, (lo, hi) in GAIN_BOUNDS.items()}


def perturb_gains(base: dict, rng: np.random.Generator, sigma_frac: float) -> dict:
    out = {}
    for k, (lo, hi) in GAIN_BOUNDS.items():
        span = hi - lo
        v = base[k] + rng.normal(0.0, sigma_frac * span)
        out[k] = float(np.clip(v, lo, hi))
    return out


def evaluate(gains: dict, engine: PhysicsEngine, waypoints, dt: float, max_duration: float,
            seed: int = None) -> tuple:
    """Fly `waypoints` with `gains` and score the result. Lower cost = better.
    Returns (cost, info_dict).

    seed: if given, makes the sensor-noise realization reproducible (used by
    tools/robustness.py to sweep many *specific* noise draws); if None
    (default, used during the Monte Carlo search itself), each call draws
    fresh OS-entropy noise so trials aren't overfit to one draw."""
    controller = DroneController(engine, gains=gains)
    for wp in waypoints:
        controller.add_waypoint(wp.x, wp.y, wp.z, keep_heading=wp.keep_heading)

    leg_yaws = _leg_yaw_targets(waypoints)

    state = engine.reset()
    controller.reset()
    rng = np.random.default_rng(seed) if seed is not None else None
    timing = ControlLoopTiming(engine, rng=rng)
    timing.reset(state)

    n_steps = int(max_duration / dt)
    roll_sq = pitch_sq = yaw_err_sq = alt_err_sq = xy_err_sq = 0.0
    n_samples = 0
    jerk_sum = 0.0
    n_jerk = 0
    prev_cmd = None
    diverged = False
    safety_halted = False
    t_final = max_duration
    wp_idx_final = 0

    for i in range(n_steps):
        cmd = timing.step(controller, state, dt)
        if prev_cmd is not None:
            jerk_sum += float(np.mean(np.abs(cmd - prev_cmd)))
            n_jerk += 1
        prev_cmd = cmd
        state = engine.step(state, cmd, dt)

        wp_idx = controller.current_waypoint_index
        t = (i + 1) * dt

        if np.any(np.abs(state[6:8]) > SAFETY_HALT_RAD):
            # Real hardware cuts the motors here -- don't stop the trial (the
            # real controller can recover if the tilt drops back down), but
            # this flags the whole trial as unsafe regardless of how it ends.
            safety_halted = True

        if t > 1.0:   # skip the first second (takeoff transient)
            target = waypoints[min(wp_idx, len(waypoints) - 1)]
            roll_sq  += state[6] ** 2
            pitch_sq += state[7] ** 2
            yaw_err_sq += _wrap(leg_yaws[min(wp_idx, len(leg_yaws) - 1)] - state[8]) ** 2
            alt_err_sq += (state[2] - target.z) ** 2
            xy_err_sq  += (state[0] - target.x) ** 2 + (state[1] - target.y) ** 2
            n_samples += 1

        if np.any(np.abs(state[6:8]) > np.radians(90)):
            diverged = True
            t_final = t
            wp_idx_final = wp_idx
            break
        if controller.mission_complete:
            t_final = t
            wp_idx_final = wp_idx
            break
    else:
        wp_idx_final = controller.current_waypoint_index

    # Same tier as the tilt-based safety halt above: on real hardware this
    # trips Position::altitudeStale() and cuts the motors (see
    # Drone_Main/controller.cpp, diagnosed 2026-07-19), so a trial that
    # triggers it is exactly as unsafe as one that tripped the tilt cut,
    # not just "somewhat worse accuracy" buried in the RMS terms below.
    safety_halted = safety_halted or timing.altitude_ever_stale

    landed = bool(controller.mission_complete)
    progress = (wp_idx_final + 1) / len(waypoints)
    n_samples = max(n_samples, 1)
    n_jerk = max(n_jerk, 1)

    roll_rms_deg    = np.degrees(np.sqrt(roll_sq / n_samples))
    pitch_rms_deg   = np.degrees(np.sqrt(pitch_sq / n_samples))
    yaw_err_rms_deg = np.degrees(np.sqrt(yaw_err_sq / n_samples))
    alt_rms_cm      = 100.0 * np.sqrt(alt_err_sq / n_samples)
    xy_rms_cm       = 100.0 * np.sqrt(xy_err_sq / n_samples)
    jerk_pct        = 100.0 * (jerk_sum / n_jerk)

    # Tiered failure cost: full divergence (rigid-body tumble) is worst,
    # tripping the real 45 deg safety cut is next (motors would have cut on
    # hardware even though the sim didn't tumble past 90 deg), then simply
    # not finishing the mission in time.
    cost = 0.0
    if diverged:
        cost += 100_000.0 + (1.0 - progress) * 50_000.0
    elif safety_halted:
        cost += 40_000.0 + (1.0 - progress) * 20_000.0
    elif not landed:
        cost += 20_000.0 * (1.0 - progress)

    # Accuracy-only objective -- no reward for finishing faster (no t_final
    # term). Position tracking (xy + altitude) carries the most weight since
    # "follows waypoints accurately" is the actual goal; roll/pitch RMS and
    # jerk are secondary smoothness signals, not the primary objective.
    cost += (2.0 * alt_rms_cm + 2.0 * xy_rms_cm +
             2.0 * roll_rms_deg + 2.0 * pitch_rms_deg +
             1.5 * yaw_err_rms_deg + 2.0 * jerk_pct)

    info = dict(diverged=diverged, safety_halted=safety_halted, landed=landed,
                progress=progress, t_final=t_final,
                roll_rms_deg=roll_rms_deg, pitch_rms_deg=pitch_rms_deg,
                yaw_err_rms_deg=yaw_err_rms_deg, alt_rms_cm=alt_rms_cm,
                xy_rms_cm=xy_rms_cm, jerk_pct=jerk_pct, cost=cost)
    return cost, info


def _record(history, phase, cost, info, best_cost, best_info):
    history.append(dict(
        trial=len(history) + 1, phase=phase, cost=cost, diverged=info['diverged'],
        safety_halted=info['safety_halted'], landed=info['landed'], best_cost=best_cost,
        best_roll_rms_deg=best_info['roll_rms_deg'],
        best_pitch_rms_deg=best_info['pitch_rms_deg'],
        best_yaw_err_rms_deg=best_info['yaw_err_rms_deg'],
        best_alt_rms_cm=best_info['alt_rms_cm'],
        best_xy_rms_cm=best_info['xy_rms_cm'],
        best_jerk_pct=best_info['jerk_pct'],
        best_progress=best_info['progress'],
    ))


def tune(waypoints=None, n_global=250, n_local=250, dt=0.004, max_duration=45.0,
         seed=None, verbose=True, plot=True, save_history=True):
    waypoints = waypoints or DEFAULT_MISSION
    rng = np.random.default_rng(seed)

    vehicle = load_vehicle(verbose=verbose)
    motor   = MotorModel()
    engine  = PhysicsEngine(vehicle, motor)

    best_gains, best_cost, best_info = None, float('inf'), None
    history = []
    t0 = time.time()

    if verbose:
        print(f"\n[tune_gains] Phase 1/2: global search ({n_global} trials)...")
    for i in range(n_global):
        g = sample_gains(rng)
        cost, info = evaluate(g, engine, waypoints, dt, max_duration)
        if cost < best_cost:
            best_cost, best_gains, best_info = cost, g, info
        _record(history, 'global', cost, info, best_cost, best_info)
        if verbose and (i + 1) % 25 == 0:
            print(f"  [{i+1:>4}/{n_global}]  best_cost={best_cost:9.1f}  "
                  f"elapsed={time.time()-t0:5.0f}s")

    if verbose:
        print(f"\n[tune_gains] Phase 2/2: local refinement ({n_local} trials)...")
    sigma, sigma_min = 0.25, 0.03
    decay = (sigma_min / sigma) ** (1.0 / max(1, n_local))
    for i in range(n_local):
        g = perturb_gains(best_gains, rng, sigma)
        cost, info = evaluate(g, engine, waypoints, dt, max_duration)
        if cost < best_cost:
            best_cost, best_gains, best_info = cost, g, info
        _record(history, 'local', cost, info, best_cost, best_info)
        sigma *= decay
        if verbose and (i + 1) % 25 == 0:
            print(f"  [{i+1:>4}/{n_local}]  best_cost={best_cost:9.1f}  "
                  f"sigma={sigma:.3f}  elapsed={time.time()-t0:5.0f}s")

    if verbose:
        print(f"\n[tune_gains] Done in {time.time()-t0:.0f}s. Best cost = {best_cost:.1f}")
        print_report(best_gains, best_info)

    if save_history:
        out_dir = save_tuning_outputs(history, best_gains, best_info)
        if verbose:
            print(f"\n[tune_gains] Trial history + winning gains saved to {out_dir}")

    if plot:
        from visualizer.plotter import plot_tuning_progress
        plot_tuning_progress(history, best_gains,
                             title=f"Monte Carlo Gain Search — {n_global}+{n_local} trials, "
                                   f"best cost={best_cost:.0f}")

    return best_gains, best_cost, best_info


def format_main_h_gains(gains: dict) -> str:
    """Renders `gains` as a Drone_Main/main.h `struct Gains { ... };` block."""
    lines = ["struct Gains {"]
    lines.append(f"  float kp_roll  = {gains['kp_roll']:.4f}f,  ki_roll  = {gains['ki_roll']:.4f}f,  kd_roll  = {gains['kd_roll']:.4f}f;")
    lines.append(f"  float kp_pitch = {gains['kp_pitch']:.4f}f,  ki_pitch = {gains['ki_pitch']:.4f}f,  kd_pitch = {gains['kd_pitch']:.4f}f;")
    lines.append(f"  float kp_yaw   = {gains['kp_yaw']:.4f}f,  ki_yaw   = {gains['ki_yaw']:.4f}f,  kd_yaw   = {gains['kd_yaw']:.4f}f;")
    lines.append(f"  float kp_x = {gains['kp_x']:.4f}f,  ki_x = {gains['ki_x']:.4f}f,  kd_x = {gains['kd_x']:.4f}f;")
    lines.append(f"  float kp_y = {gains['kp_y']:.4f}f,  ki_y = {gains['ki_y']:.4f}f,  kd_y = {gains['kd_y']:.4f}f;")
    lines.append(f"  float kp_z = {gains['kp_z']:.4f}f,  ki_z = {gains['ki_z']:.4f}f,   kd_z = {gains['kd_z']:.4f}f;")
    lines.append("};")
    return "\n".join(lines)


def print_report(gains: dict, info: dict):
    print("\n--- Winning run ---")
    print(f"  diverged            : {info['diverged']}")
    print(f"  safety-halt tripped : {info['safety_halted']}  (would have cut motors on real hardware)")
    print(f"  landed              : {info['landed']}")
    print(f"  mission progress    : {info['progress']*100:.0f}%")
    print(f"  time to complete    : {info['t_final']:.1f} s  (not scored -- speed is not an objective)")
    print(f"  position RMS (xy)   : {info['xy_rms_cm']:.1f} cm")
    print(f"  altitude RMS error  : {info['alt_rms_cm']:.1f} cm")
    print(f"  roll RMS            : {info['roll_rms_deg']:.2f} deg")
    print(f"  pitch RMS           : {info['pitch_rms_deg']:.2f} deg")
    print(f"  yaw tracking error  : {info['yaw_err_rms_deg']:.2f} deg (RMS)")
    print(f"  motor jerk          : {info['jerk_pct']:.2f} %/step")

    print("\n--- Gains, grouped by axis ---")
    for axis, keys in _AXES.items():
        vals = "  ".join(f"{k}={gains[k]:.4f}" for k in keys)
        print(f"  {axis:<6}: {vals}")

    print("\n--- Paste into Drone_Main/main.h ---")
    print(format_main_h_gains(gains))


def save_tuning_outputs(history: list, gains: dict, info: dict, out_dir: str = None) -> str:
    """
    Writes the per-trial search history (CSV) and the winning gains (as a
    ready-to-paste main.h struct block) to outputs/tuning/<timestamp>/.
    Returns the directory path.
    """
    out_dir = out_dir or os.path.join(_OUTPUT_DIR, datetime.now().strftime('%Y%m%d_%H%M%S'))
    os.makedirs(out_dir, exist_ok=True)

    csv_path = os.path.join(out_dir, 'trial_history.csv')
    fieldnames = list(history[0].keys()) if history else []
    with open(csv_path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(history)

    gains_path = os.path.join(out_dir, 'winning_gains.txt')
    with open(gains_path, 'w') as f:
        f.write(f"# cost={info['cost']:.1f}  diverged={info['diverged']}  landed={info['landed']}\n")
        f.write(f"# roll_rms_deg={info['roll_rms_deg']:.2f}  pitch_rms_deg={info['pitch_rms_deg']:.2f}  "
                f"yaw_err_rms_deg={info['yaw_err_rms_deg']:.2f}  alt_rms_cm={info['alt_rms_cm']:.1f}\n\n")
        f.write(format_main_h_gains(gains) + "\n")

    return out_dir
