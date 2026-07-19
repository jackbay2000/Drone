"""
Cross-validation: verifies the pure-Python translation (PythonDroneController)
and the real compiled Drone_Main controller (drone_bridge.DroneController)
compute the same thing, so translation drift gets caught automatically
instead of requiring a manual debugging session to notice (see project
memory "DLL bridge fixed" -- that session cost real time to a mistake this
tool exists specifically to prevent).

Method: OPEN-LOOP replay, not two independent closed-loop flights. A single
noisy sensed-state trajectory is recorded once (flying the Python controller
against the physics sim), then the *identical* (state, dt) sequence is
replayed into fresh instances of both backends and their motor outputs are
compared tick by tick. This is deliberate: two independently-run closed
loops will drift apart over time purely from float32 (C++) vs float64
(Python) arithmetic, especially for marginal gains -- that's expected
numerical sensitivity, not a bug, and comparing final states after a long
flight would conflate the two. Open-loop replay isolates "does the control
law compute the same output given the same input" from "do two closed loops
end up in the same place", which is the actual question this tool answers.

CRITICAL gotcha this tool exists to avoid repeating: never write
`from controller.pid_controller import DroneController` and treat that as
"the Python one" -- once a compiler is on PATH, that name silently resolves
to the DLL-backed class. `PythonDroneController` and `drone_bridge.
DroneController` must always be imported explicitly and separately, and
never interleaved via aliases, because the DLL's Controller/IMU/Position
state lives in C++ globals shared by every drone_bridge.DroneController
instance -- two "different" wrapper objects secretly corrupt each other if
called in alternation.

Usage:
    python -m tools.cross_validate
    python -m tools.cross_validate --ticks 1000 --seed 3
"""

import argparse

import numpy as np

from sim.vehicle import load_vehicle
from sim.motor_model import MotorModel
from sim.physics_engine import PhysicsEngine
from sim.timing import ControlLoopTiming
from controller.pid_controller import PythonDroneController

# Tolerances are looser with noise on: float32 (C++) vs float64 (Python)
# arithmetic accumulates small but real per-tick differences through chained
# PID multiply-adds and derivative divisions, even from clean ("noise off")
# inputs once the system isn't sitting exactly at zero -- that's expected
# precision drift, not a translation bug. A REAL translation bug (as found
# and fixed this project, e.g. the yaw-frame rotation bug) produces
# differences of order 0.1+, an order of magnitude above either tolerance.
_TOL_NOISE_OFF = 0.005
_TOL_NOISE_ON  = 0.03

_DEFAULT_GAINS = dict(
    kp_roll=1.5,  ki_roll=0.05,  kd_roll=0.15,
    kp_pitch=1.5, ki_pitch=0.05, kd_pitch=0.15,
    kp_yaw=1.0,   ki_yaw=0.02,   kd_yaw=0.1,
    kp_x=0.3,     ki_x=0.01,     kd_x=0.2,
    kp_y=0.3,     ki_y=0.01,     kd_y=0.2,
    kp_z=0.5,     ki_z=0.02,     kd_z=0.25,
)


def _dll_available() -> bool:
    try:
        from controller import drone_bridge
        drone_bridge.build(verbose=False)
        return True
    except Exception:
        return False


def _record_trajectory(gains: dict, engine: PhysicsEngine, n_ticks: int, dt: float,
                       seed: int, sensor_noise: bool) -> list:
    """Flies PythonDroneController against the real physics for n_ticks
    control ticks, recording the exact (sensed_state, dt) pair fed to it
    each tick -- this is what gets replayed into both backends."""
    controller = PythonDroneController(engine, gains=gains)
    controller.add_waypoint(1.0, 0.0, 1.0, keep_heading=True)
    controller.add_waypoint(1.0, 1.0, 1.0, keep_heading=False)
    state = engine.reset()
    controller.reset()
    timing = ControlLoopTiming(engine, sensor_noise=sensor_noise, rng=np.random.default_rng(seed))
    timing.reset(state)

    recorded = []
    orig_update = controller.update
    def wrapped(s, d):
        recorded.append((s.copy(), d))
        return orig_update(s, d)
    controller.update = wrapped

    physics_dt = 0.002
    ticks_done = 0
    i = 0
    while ticks_done < n_ticks and i < n_ticks * 10:
        before = len(recorded)
        cmd = timing.step(controller, state, physics_dt)
        state = engine.step(state, cmd, physics_dt)
        if len(recorded) > before:
            ticks_done += 1
        i += 1
    return recorded


def _replay(recorded: list, ctor, engine: PhysicsEngine, gains: dict) -> np.ndarray:
    """Feeds the identical recorded (state, dt) sequence into a fresh
    controller instance and returns the stacked motor-command outputs."""
    controller = ctor(engine, gains=gains)
    controller.add_waypoint(1.0, 0.0, 1.0, keep_heading=True)
    controller.add_waypoint(1.0, 1.0, 1.0, keep_heading=False)
    controller.reset()
    out = np.zeros((len(recorded), 4))
    for i, (s, d) in enumerate(recorded):
        out[i] = controller.update(s.copy(), d)
    return out


def cross_validate(gains: dict = None, n_ticks: int = 500, dt: float = 0.002,
                   seed: int = 0, verbose: bool = True) -> dict:
    """
    Returns a report dict with pass/fail for the noise-off and noise-on
    checks. If no C++ compiler is available, returns {'skipped': True}
    rather than failing -- there's nothing to cross-validate against.
    """
    gains = gains or _DEFAULT_GAINS

    if not _dll_available():
        if verbose:
            print("[cross_validate] No C++ compiler available -- nothing to cross-validate "
                  "against. Install one (see drone_bridge.py) to enable this check.")
        return {'skipped': True}

    from controller import drone_bridge

    vehicle = load_vehicle(verbose=False)
    motor   = MotorModel()
    engine  = PhysicsEngine(vehicle, motor)

    report = {'skipped': False}
    for label, noise, tol in [('noise_off', False, _TOL_NOISE_OFF),
                              ('noise_on',  True,  _TOL_NOISE_ON)]:
        recorded = _record_trajectory(gains, engine, n_ticks, dt, seed, noise)
        py_out  = _replay(recorded, PythonDroneController, engine, gains)
        dll_out = _replay(recorded, drone_bridge.DroneController, engine, gains)

        diff = np.abs(py_out - dll_out)
        max_diff = float(diff.max())
        mean_diff = float(diff.mean())
        passed = max_diff <= tol

        report[label] = dict(max_diff=max_diff, mean_diff=mean_diff, tol=tol,
                             passed=passed, n_ticks=len(recorded))
        if verbose:
            status = 'PASS' if passed else 'FAIL'
            print(f"[cross_validate] {label:10s} max_diff={max_diff:.6f} "
                  f"(tol={tol})  mean_diff={mean_diff:.6f}  [{status}]")
            if not passed:
                worst = int(diff.max(axis=1).argmax())
                print(f"    worst tick={worst}: py={np.round(py_out[worst],4)} "
                      f"dll={np.round(dll_out[worst],4)}")

    report['all_passed'] = report['noise_off']['passed'] and report['noise_on']['passed']
    return report


# ---------------------------------------------------------------------------
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Cross-validate Python translation against the compiled controller')
    parser.add_argument('--ticks', type=int, default=500)
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()

    report = cross_validate(n_ticks=args.ticks, seed=args.seed, verbose=True)
    if report.get('skipped'):
        raise SystemExit(0)
    raise SystemExit(0 if report['all_passed'] else 1)
