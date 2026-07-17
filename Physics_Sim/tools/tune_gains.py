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

import time
import numpy as np

from sim.vehicle import load_vehicle
from sim.motor_model import MotorModel
from sim.physics_engine import PhysicsEngine
from controller.pid_controller import DroneController
from controller.waypoint_controller import Waypoint

# ---------------------------------------------------------------------------
# Default mission -- matches Drone_Main/Waypoints.h exactly, so gains tuned
# here are being evaluated against the same mission the real firmware flies.
# ---------------------------------------------------------------------------
DEFAULT_MISSION = [
    Waypoint(0.0, 0.0, 1.0, True),
    Waypoint(1.0, 0.0, 1.0, False),
    Waypoint(1.0, 1.0, 1.0, False),
    Waypoint(0.0, 0.0, 1.0, True),
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


def evaluate(gains: dict, engine: PhysicsEngine, waypoints, dt: float, max_duration: float) -> tuple:
    """Fly `waypoints` with `gains` and score the result. Lower cost = better.
    Returns (cost, info_dict)."""
    controller = DroneController(engine, gains=gains)
    for wp in waypoints:
        controller.add_waypoint(wp.x, wp.y, wp.z, keep_heading=wp.keep_heading)

    leg_yaws = _leg_yaw_targets(waypoints)

    state = engine.reset()
    controller.reset()

    n_steps = int(max_duration / dt)
    roll_sq = pitch_sq = yaw_err_sq = alt_err_sq = 0.0
    n_samples = 0
    jerk_sum = 0.0
    n_jerk = 0
    prev_cmd = None
    diverged = False
    t_final = max_duration
    wp_idx_final = 0

    for i in range(n_steps):
        cmd = controller.update(state, dt)
        if prev_cmd is not None:
            jerk_sum += float(np.mean(np.abs(cmd - prev_cmd)))
            n_jerk += 1
        prev_cmd = cmd
        state = engine.step(state, cmd, dt)

        wp_idx = controller.current_waypoint_index
        t = (i + 1) * dt

        if t > 1.0:   # skip the first second (takeoff transient)
            roll_sq  += state[6] ** 2
            pitch_sq += state[7] ** 2
            yaw_err_sq += _wrap(leg_yaws[min(wp_idx, len(leg_yaws) - 1)] - state[8]) ** 2
            alt_err_sq += (state[2] - waypoints[min(wp_idx, len(waypoints) - 1)].z) ** 2
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

    landed = bool(controller.mission_complete)
    progress = (wp_idx_final + 1) / len(waypoints)
    n_samples = max(n_samples, 1)
    n_jerk = max(n_jerk, 1)

    roll_rms_deg    = np.degrees(np.sqrt(roll_sq / n_samples))
    pitch_rms_deg   = np.degrees(np.sqrt(pitch_sq / n_samples))
    yaw_err_rms_deg = np.degrees(np.sqrt(yaw_err_sq / n_samples))
    alt_rms_cm      = 100.0 * np.sqrt(alt_err_sq / n_samples)
    jerk_pct        = 100.0 * (jerk_sum / n_jerk)

    cost = 0.0
    if diverged:
        cost += 100_000.0 + (1.0 - progress) * 50_000.0
    elif not landed:
        cost += 20_000.0 * (1.0 - progress)

    cost += (1.0 * alt_rms_cm + 3.0 * roll_rms_deg + 3.0 * pitch_rms_deg +
             1.5 * yaw_err_rms_deg + 2.0 * jerk_pct + 0.3 * t_final)

    info = dict(diverged=diverged, landed=landed, progress=progress, t_final=t_final,
                roll_rms_deg=roll_rms_deg, pitch_rms_deg=pitch_rms_deg,
                yaw_err_rms_deg=yaw_err_rms_deg, alt_rms_cm=alt_rms_cm,
                jerk_pct=jerk_pct, cost=cost)
    return cost, info


def tune(waypoints=None, n_global=250, n_local=250, dt=0.004, max_duration=30.0,
         seed=None, verbose=True):
    waypoints = waypoints or DEFAULT_MISSION
    rng = np.random.default_rng(seed)

    vehicle = load_vehicle(verbose=verbose)
    motor   = MotorModel()
    engine  = PhysicsEngine(vehicle, motor)

    best_gains, best_cost, best_info = None, float('inf'), None
    t0 = time.time()

    if verbose:
        print(f"\n[tune_gains] Phase 1/2: global search ({n_global} trials)...")
    for i in range(n_global):
        g = sample_gains(rng)
        cost, info = evaluate(g, engine, waypoints, dt, max_duration)
        if cost < best_cost:
            best_cost, best_gains, best_info = cost, g, info
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
        sigma *= decay
        if verbose and (i + 1) % 25 == 0:
            print(f"  [{i+1:>4}/{n_local}]  best_cost={best_cost:9.1f}  "
                  f"sigma={sigma:.3f}  elapsed={time.time()-t0:5.0f}s")

    if verbose:
        print(f"\n[tune_gains] Done in {time.time()-t0:.0f}s. Best cost = {best_cost:.1f}")
        print_report(best_gains, best_info)

    return best_gains, best_cost, best_info


def print_report(gains: dict, info: dict):
    print("\n--- Winning run ---")
    print(f"  diverged            : {info['diverged']}")
    print(f"  landed              : {info['landed']}")
    print(f"  mission progress    : {info['progress']*100:.0f}%")
    print(f"  time to complete    : {info['t_final']:.1f} s")
    print(f"  roll RMS            : {info['roll_rms_deg']:.2f} deg")
    print(f"  pitch RMS           : {info['pitch_rms_deg']:.2f} deg")
    print(f"  yaw tracking error  : {info['yaw_err_rms_deg']:.2f} deg (RMS)")
    print(f"  altitude RMS error  : {info['alt_rms_cm']:.1f} cm")
    print(f"  motor jerk          : {info['jerk_pct']:.2f} %/step")

    print("\n--- Gains, grouped by axis ---")
    for axis, keys in _AXES.items():
        vals = "  ".join(f"{k}={gains[k]:.4f}" for k in keys)
        print(f"  {axis:<6}: {vals}")

    print("\n--- Paste into Drone_Main/main.h ---")
    print("struct Gains {")
    print(f"  float kp_roll  = {gains['kp_roll']:.4f}f,  ki_roll  = {gains['ki_roll']:.4f}f,  kd_roll  = {gains['kd_roll']:.4f}f;")
    print(f"  float kp_pitch = {gains['kp_pitch']:.4f}f,  ki_pitch = {gains['ki_pitch']:.4f}f,  kd_pitch = {gains['kd_pitch']:.4f}f;")
    print(f"  float kp_yaw   = {gains['kp_yaw']:.4f}f,  ki_yaw   = {gains['ki_yaw']:.4f}f,  kd_yaw   = {gains['kd_yaw']:.4f}f;")
    print(f"  float kp_x = {gains['kp_x']:.4f}f,  ki_x = {gains['ki_x']:.4f}f,  kd_x = {gains['kd_x']:.4f}f;")
    print(f"  float kp_y = {gains['kp_y']:.4f}f,  ki_y = {gains['ki_y']:.4f}f,  kd_y = {gains['kd_y']:.4f}f;")
    print(f"  float kp_z = {gains['kp_z']:.4f}f,  ki_z = {gains['ki_z']:.4f}f,   kd_z = {gains['kd_z']:.4f}f;")
    print("};")
