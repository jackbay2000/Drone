"""
Quadcopter Physics Simulation
==============================
Run:  python main.py
      python main.py --scenario altitude_step
      python main.py --scenario waypoints
      python main.py --scenario waypoints --waypoints my_mission.csv
      python main.py --scenario tune                    (Monte Carlo gain search)
      python main.py --scenario tune --tune-trials 400

Drop-in folders:
  ../Models/            -> .stl / .3mf files for printed parts
  inputs/components/    -> component_list.json  (mass + CoM per part)
  ../Drone_Main/        -> .cpp/.h files scanned automatically for PID gains
                           (also compiled to DLL if a C++ compiler is on PATH)
  inputs/waypoints/     -> waypoints.csv (or any .csv passed via --waypoints)
"""

import argparse
import numpy as np

from sim.vehicle               import load_vehicle
from sim.motor_model           import MotorModel
from sim.physics_engine        import PhysicsEngine
from controller.pid_controller import DroneController
from controller.arduino_parser import parse_gains
from controller.waypoint_controller import load_waypoints
from visualizer.plotter        import plot_results, print_metrics
from visualizer.model_viewer   import show_vehicle_model


# ------------------------------------------------------------------
# Built-in scenarios — each is a list of (x, y, z) waypoints.
# The DroneController sequences through them internally, exactly as
# Drone_Main.ino does.
# ------------------------------------------------------------------
SCENARIOS = {
    'hover': {
        'description': 'Take off and hover at (0, 0, 1 m) for 15 s.',
        'duration_s': 15.0,
        'waypoints': [(0.0, 0.0, 1.0)],
    },
    'altitude_step': {
        'description': 'Climb to 1 m, hold, then step up to 2 m.',
        'duration_s': 20.0,
        'waypoints': [(0.0, 0.0, 1.0), (0.0, 0.0, 2.0)],
    },
}


# ------------------------------------------------------------------
def run(scenario_name: str = 'hover', dt: float = 0.002,
        waypoints_file: str = None, tune_args: dict = None):

    if scenario_name == 'waypoints':
        _run_waypoints(dt=dt, waypoints_file=waypoints_file)
        return

    if scenario_name == 'tune':
        _run_tune(dt=dt, waypoints_file=waypoints_file, **(tune_args or {}))
        return

    scenario = SCENARIOS[scenario_name]
    print(f"\n=== Scenario: {scenario_name} ===")
    print(f"    {scenario['description']}")

    vehicle = load_vehicle(verbose=True)
    motor   = MotorModel()
    engine  = PhysicsEngine(vehicle, motor)

    arduino_gains = parse_gains(verbose=True)
    controller    = DroneController(engine, gains=arduino_gains)

    for (x, y, z) in scenario['waypoints']:
        controller.add_waypoint(x, y, z, keep_heading=True)

    duration = scenario['duration_s']
    n_steps  = int(duration / dt)

    states   = np.zeros((n_steps + 1, 12))
    commands = np.zeros((n_steps + 1, 4))
    t_arr    = np.linspace(0, duration, n_steps + 1)

    state     = engine.reset()
    states[0] = state
    controller.reset()

    print(f"\n    Simulating {duration}s at dt={dt*1000:.1f}ms ({n_steps} steps)...")

    for i in range(n_steps):
        cmd          = controller.update(state, dt)
        commands[i]  = cmd
        state        = engine.step(state, cmd, dt)
        states[i+1]  = state

        if np.any(np.abs(state[6:8]) > np.radians(90)):
            print(f"\n    [!] Vehicle diverged at t={t_arr[i]:.2f}s (angle > 90°). "
                  "PID gains may need tuning.")
            states   = states[:i+2]
            commands = commands[:i+2]
            t_arr    = t_arr[:i+2]
            break

    commands[-1] = commands[-2]

    target_z = scenario['waypoints'][-1][2]
    print_metrics(t_arr, states, setpoint_z=target_z)
    show_vehicle_model()
    plot_results(t_arr, states, commands,
                 title=f"Quadcopter Sim — {scenario_name}  "
                       f"(mass={vehicle.mass_kg*1000:.0f}g, source={vehicle.source})")


# ------------------------------------------------------------------
def _run_waypoints(dt: float = 0.002, waypoints_file: str = None):
    print("\n=== Scenario: waypoints ===")

    wps = load_waypoints(filepath=waypoints_file, verbose=True)

    vehicle = load_vehicle(verbose=True)
    motor   = MotorModel()
    engine  = PhysicsEngine(vehicle, motor)

    arduino_gains = parse_gains(verbose=True)
    controller    = DroneController(engine, gains=arduino_gains)

    for wp in wps:
        controller.add_waypoint(wp.x, wp.y, wp.z, keep_heading=wp.keep_heading)

    # Generous time budget: 3× estimated travel time at 1 m/s, min 30 s
    total_dist = sum(
        np.linalg.norm([wps[i+1].x - wps[i].x,
                        wps[i+1].y - wps[i].y,
                        wps[i+1].z - wps[i].z])
        for i in range(len(wps) - 1)
    ) if len(wps) > 1 else 0.0
    duration = max(30.0, total_dist * 3.0)
    n_steps  = int(duration / dt)

    states   = np.zeros((n_steps + 1, 12))
    commands = np.zeros((n_steps + 1, 4))
    t_arr    = np.linspace(0, duration, n_steps + 1)
    wp_log   = np.zeros(n_steps + 1, dtype=int)

    state     = engine.reset()
    states[0] = state
    controller.reset()

    print(f"\n    Simulating {duration:.0f}s at dt={dt*1000:.1f}ms ({n_steps} steps)...")
    print(f"  {'t(s)':>6}  {'WP':>2}  {'x':>7}  {'y':>7}  {'z':>6}"
          f"  {'roll':>6}  {'pitch':>6}  {'yaw':>6}")

    for i in range(n_steps):
        cmd          = controller.update(state, dt)
        commands[i]  = cmd
        wp_log[i]    = controller.current_waypoint_index
        state        = engine.step(state, cmd, dt)
        states[i+1]  = state

        if i % int(5.0 / dt) == 0:
            print(f"  {t_arr[i]:6.1f}  {controller.current_waypoint_index:2d}"
                  f"  {state[0]:7.2f}  {state[1]:7.2f}  {state[2]:6.2f}"
                  f"  {np.degrees(state[6]):6.1f}  {np.degrees(state[7]):6.1f}"
                  f"  {np.degrees(state[8]):6.1f}")

        if np.any(np.abs(state[6:8]) > np.radians(90)):
            print(f"\n    [!] Vehicle diverged at t={t_arr[i]:.2f}s.")
            states   = states[:i+2]
            commands = commands[:i+2]
            t_arr    = t_arr[:i+2]
            wp_log   = wp_log[:i+2]
            break

        if controller.mission_complete:
            states   = states[:i+2]
            commands = commands[:i+2]
            t_arr    = t_arr[:i+2]
            wp_log   = wp_log[:i+2]
            print(f"    Mission complete at t={t_arr[-1]:.1f}s")
            break

    commands[-1] = commands[-2]

    print_metrics(t_arr, states, setpoint_z=wps[-1].z)
    show_vehicle_model()
    plot_results(t_arr, states, commands,
                 title=f"Quadcopter Sim — waypoints  "
                       f"(mass={vehicle.mass_kg*1000:.0f}g, {len(wps)} WPs)",
                 waypoints=wps)


# ------------------------------------------------------------------
def _run_tune(dt: float = 0.002, waypoints_file: str = None,
              tune_trials: int = 250, tune_local: int = 250,
              tune_dt: float = 0.004, tune_duration: float = 30.0,
              tune_seed: int = None):
    from tools.tune_gains import tune, DEFAULT_MISSION

    print("\n=== Scenario: tune (Monte Carlo gain search) ===")

    if waypoints_file:
        wps = load_waypoints(filepath=waypoints_file, verbose=True)
    else:
        wps = DEFAULT_MISSION
        print(f"[tune] Using the built-in tuning mission "
              f"(matches Drone_Main/Waypoints.h), {len(wps)} waypoints.")

    best_gains, best_cost, best_info = tune(
        waypoints=wps, n_global=tune_trials, n_local=tune_local,
        dt=tune_dt, max_duration=tune_duration, seed=tune_seed, verbose=True)

    # Final high-fidelity verification pass with the winning gains, plotted
    # exactly like a normal --scenario waypoints run.
    print("\n[tune] Running final verification pass at full resolution...")
    vehicle = load_vehicle(verbose=True)
    motor   = MotorModel()
    engine  = PhysicsEngine(vehicle, motor)
    controller = DroneController(engine, gains=best_gains)
    for wp in wps:
        controller.add_waypoint(wp.x, wp.y, wp.z, keep_heading=wp.keep_heading)

    duration = max(30.0, best_info['t_final'] * 1.5)
    n_steps  = int(duration / dt)
    states   = np.zeros((n_steps + 1, 12))
    commands = np.zeros((n_steps + 1, 4))
    t_arr    = np.linspace(0, duration, n_steps + 1)

    state     = engine.reset()
    states[0] = state
    controller.reset()

    for i in range(n_steps):
        cmd         = controller.update(state, dt)
        commands[i] = cmd
        state       = engine.step(state, cmd, dt)
        states[i+1] = state
        if np.any(np.abs(state[6:8]) > np.radians(90)):
            states = states[:i+2]; commands = commands[:i+2]; t_arr = t_arr[:i+2]
            break
        if controller.mission_complete:
            states = states[:i+2]; commands = commands[:i+2]; t_arr = t_arr[:i+2]
            break
    commands[-1] = commands[-2]

    print_metrics(t_arr, states, setpoint_z=wps[-1].z)
    show_vehicle_model()
    plot_results(t_arr, states, commands,
                 title=f"Quadcopter Sim — tuned gains  (mass={vehicle.mass_kg*1000:.0f}g)",
                 waypoints=wps)


# ------------------------------------------------------------------
if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Quadcopter physics simulation')
    parser.add_argument('--scenario', default='hover',
                        choices=list(SCENARIOS.keys()) + ['waypoints', 'tune'],
                        help='Which flight scenario to simulate')
    parser.add_argument('--dt', type=float, default=0.002,
                        help='Simulation timestep in seconds (default 0.002)')
    parser.add_argument('--waypoints', default=None, metavar='FILE',
                        help='Path to waypoints CSV (used with --scenario waypoints/tune)')
    parser.add_argument('--tune-trials', type=int, default=250,
                        help='Global Monte Carlo trials (--scenario tune)')
    parser.add_argument('--tune-local', type=int, default=250,
                        help='Local refinement trials (--scenario tune)')
    parser.add_argument('--tune-dt', type=float, default=0.004,
                        help='Timestep used during tuning search (default 0.004, coarser for speed)')
    parser.add_argument('--tune-duration', type=float, default=30.0,
                        help='Per-trial timeout in seconds (--scenario tune)')
    parser.add_argument('--tune-seed', type=int, default=None,
                        help='RNG seed for reproducible tuning runs')
    args = parser.parse_args()
    run(scenario_name=args.scenario, dt=args.dt, waypoints_file=args.waypoints,
        tune_args=dict(tune_trials=args.tune_trials, tune_local=args.tune_local,
                       tune_dt=args.tune_dt, tune_duration=args.tune_duration,
                       tune_seed=args.tune_seed))
