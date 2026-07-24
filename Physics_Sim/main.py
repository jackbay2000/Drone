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
from sim.timing                import ControlLoopTiming
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

# Altitudes below match Drone_Main/Waypoints.h's 0.4m cap -- the VL53L1X
# rangefinder's readings degrade above ~0.58m (see project notes / commit
# history, 2026-07-19), so these stay within the validated envelope. Was
# 1-2m, a leftover from before that cap was decided.
SCENARIOS = {
    'hover': {
        'description': 'Take off and hover at (0, 0, 0.4 m) for 15 s.',
        'duration_s': 15.0,
        'waypoints': [(0.0, 0.0, 0.4)],
    },
    'altitude_step': {
        'description': 'Climb to 0.2 m, hold, then step up to 0.4 m.',
        'duration_s': 20.0,
        'waypoints': [(0.0, 0.0, 0.2), (0.0, 0.0, 0.4)],
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
    timing    = ControlLoopTiming(engine)
    timing.reset(state)

    print(f"\n    Simulating {duration}s at dt={dt*1000:.1f}ms ({n_steps} steps)...")

    for i in range(n_steps):
        cmd          = timing.step(controller, state, dt)
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
    timing    = ControlLoopTiming(engine)
    timing.reset(state)

    print(f"\n    Simulating {duration:.0f}s at dt={dt*1000:.1f}ms ({n_steps} steps)...")
    print(f"  {'t(s)':>6}  {'WP':>2}  {'x':>7}  {'y':>7}  {'z':>6}"
          f"  {'roll':>6}  {'pitch':>6}  {'yaw':>6}")

    for i in range(n_steps):
        cmd          = timing.step(controller, state, dt)
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

    target_z_arr = np.array([wps[min(idx, len(wps) - 1)].z for idx in wp_log])
    print_metrics(t_arr, states, setpoint_z=target_z_arr)
    show_vehicle_model()
    plot_results(t_arr, states, commands,
                 title=f"Quadcopter Sim — waypoints  "
                       f"(mass={vehicle.mass_kg*1000:.0f}g, {len(wps)} WPs)",
                 waypoints=wps)


# ------------------------------------------------------------------
def _simulate_hover(gains: dict, vehicle, hover_alt: float = 1.0,
                    duration: float = 15.0, dt: float = 0.002):
    """
    Independent generalization check for gains found by tuning against a
    specific waypoint mission: replay the SAME winning gains on a plain,
    single-target hover, on a fresh engine/controller. A genuinely good set
    of PID gains shouldn't only work for the exact path/timing it was
    searched against -- if a plain hover looks much worse than the
    mission's own verification pass, that's a sign of overfitting to the
    tuning mission rather than gains that are actually solid.

    Note: hover_alt defaults to 1m, but the real mission (Waypoints.h) and
    the Monte Carlo tuning mission both deliberately stay at or below 0.4m,
    because the VL53L1X rangefinder's readings degrade above ~0.58m (see
    project notes / commit history). A worse result here doesn't
    automatically mean the gains are bad -- it may just mean 1m is outside
    the altitude range this vehicle's sensors have actually been validated
    for. Pass hover_alt=0.4 to cross-check within the validated envelope
    instead.

    Pure simulation, no printing/plotting -- caller decides when/how to
    report it (see _run_tune, which prints this alongside the mission's own
    metrics before either plot window pops up, so both results are visible
    together in the console instead of separated by a blocking plot).
    Returns (t_arr, states, commands).
    """
    motor      = MotorModel()
    engine     = PhysicsEngine(vehicle, motor)
    controller = DroneController(engine, gains=gains)
    controller.add_waypoint(0.0, 0.0, hover_alt, keep_heading=True)

    n_steps  = int(duration / dt)
    states   = np.zeros((n_steps + 1, 12))
    commands = np.zeros((n_steps + 1, 4))
    t_arr    = np.linspace(0, duration, n_steps + 1)

    state     = engine.reset()
    states[0] = state
    controller.reset()
    timing    = ControlLoopTiming(engine)
    timing.reset(state)

    for i in range(n_steps):
        cmd         = timing.step(controller, state, dt)
        commands[i] = cmd
        state       = engine.step(state, cmd, dt)
        states[i+1] = state
        if np.any(np.abs(state[6:8]) > np.radians(90)):
            print(f"\n    [!] Hover cross-check diverged at t={t_arr[i]:.2f}s.")
            states = states[:i+2]; commands = commands[:i+2]; t_arr = t_arr[:i+2]
            break
    commands[-1] = commands[-2]

    return t_arr, states, commands


# ------------------------------------------------------------------
def _run_tune(dt: float = 0.002, waypoints_file: str = None,
              tune_trials: int = 250, tune_local: int = 250,
              tune_dt: float = 0.004, tune_duration: float = 45.0,
              tune_seed: int = None, tune_crosscheck_alt: float = 1.0):
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
    wp_log   = np.zeros(n_steps + 1, dtype=int)

    state     = engine.reset()
    states[0] = state
    controller.reset()
    timing    = ControlLoopTiming(engine)
    timing.reset(state)

    for i in range(n_steps):
        cmd         = timing.step(controller, state, dt)
        commands[i] = cmd
        wp_log[i]   = controller.current_waypoint_index
        state       = engine.step(state, cmd, dt)
        states[i+1] = state
        if np.any(np.abs(state[6:8]) > np.radians(90)):
            states = states[:i+2]; commands = commands[:i+2]; t_arr = t_arr[:i+2]; wp_log = wp_log[:i+2]
            break
        if controller.mission_complete:
            states = states[:i+2]; commands = commands[:i+2]; t_arr = t_arr[:i+2]; wp_log = wp_log[:i+2]
            break
    commands[-1] = commands[-2]

    # Independent cross-check: same winning gains, unrelated simple scenario.
    # Simulated here (before any metrics are printed) so both results can be
    # reported together at the end, rather than the hover's metrics showing
    # up only after you've closed the mission's plot window.
    hover_result = None
    if tune_crosscheck_alt is not None:
        print(f"\n[tune] Cross-check: replaying the winning gains against a plain "
              f"hover at (0,0,{tune_crosscheck_alt:.2f}m), independent of the tuning mission...")
        if tune_crosscheck_alt > 0.65:
            print(f"    [!] {tune_crosscheck_alt:.2f}m is above the ~0.58m range where the "
                  f"rangefinder is known to degrade -- a worse result here may reflect sensor "
                  f"range, not gain quality. See Waypoints.h.")
        hover_result = _simulate_hover(best_gains, vehicle, hover_alt=tune_crosscheck_alt, dt=dt)

    target_z_arr = np.array([wps[min(idx, len(wps) - 1)].z for idx in wp_log])
    print("\n--- Mission verification (tuning mission, full resolution) ---")
    print_metrics(t_arr, states, setpoint_z=target_z_arr)

    if hover_result is not None:
        hover_t, hover_states, hover_commands = hover_result
        print("\n--- Hover cross-check (independent of the tuning mission) ---")
        print_metrics(hover_t, hover_states, setpoint_z=tune_crosscheck_alt)

    show_vehicle_model()
    plot_results(t_arr, states, commands,
                 title=f"Quadcopter Sim — tuned gains  (mass={vehicle.mass_kg*1000:.0f}g)",
                 waypoints=wps)
    if hover_result is not None:
        plot_results(hover_t, hover_states, hover_commands,
                     title=f"Cross-check — plain hover @ {tune_crosscheck_alt:.2f}m  "
                           f"(gains from mission tune)")


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
    parser.add_argument('--tune-duration', type=float, default=45.0,
                        help='Per-trial timeout in seconds (--scenario tune). Kept generous '
                             'since the cost function no longer rewards finishing faster.')
    parser.add_argument('--tune-seed', type=int, default=None,
                        help='RNG seed for reproducible tuning runs')
    parser.add_argument('--tune-crosscheck-alt', type=float, default=1.0,
                        help='After tuning, replay the winning gains against an independent '
                             'plain hover at this altitude (metres) to check they generalize '
                             'beyond the tuning mission. Pass a negative number to skip it. '
                             'Default 1.0 -- note this is above the ~0.58m range the rangefinder '
                             'is currently validated for (the tuning mission itself stays <=0.4m), '
                             'so a worse result here may reflect sensor range, not gain quality. '
                             'Pass 0.4 to cross-check within the validated envelope instead.')
    args = parser.parse_args()
    run(scenario_name=args.scenario, dt=args.dt, waypoints_file=args.waypoints,
        tune_args=dict(tune_trials=args.tune_trials, tune_local=args.tune_local,
                       tune_dt=args.tune_dt, tune_duration=args.tune_duration,
                       tune_seed=args.tune_seed,
                       tune_crosscheck_alt=(None if args.tune_crosscheck_alt < 0
                                            else args.tune_crosscheck_alt)))
