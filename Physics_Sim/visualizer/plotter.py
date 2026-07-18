"""
Post-simulation analysis plots.
Produces a 2x3 figure: altitude, roll, pitch, yaw, motor commands, 3D trajectory.
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401


def plot_results(t: np.ndarray, states: np.ndarray, commands: np.ndarray,
                 setpoints: dict = None, title: str = "Simulation Results",
                 waypoints=None):
    """
    Args:
        t:         time vector, shape (N,)
        states:    state history, shape (N, 12)
        commands:  motor command history, shape (N, 4), values in [0,1]
        setpoints: dict of {label: (time, value)} for reference lines
        title:     figure title
    """
    fig = plt.figure(figsize=(16, 9))
    fig.suptitle(title, fontsize=14)
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.4, wspace=0.35)

    ax_alt  = fig.add_subplot(gs[0, 0])
    ax_roll = fig.add_subplot(gs[0, 1])
    ax_pit  = fig.add_subplot(gs[0, 2])
    ax_yaw  = fig.add_subplot(gs[1, 0])
    ax_mot  = fig.add_subplot(gs[1, 1])
    ax_3d   = fig.add_subplot(gs[1, 2], projection='3d')

    deg = np.degrees

    # Altitude
    ax_alt.plot(t, states[:, 2], 'b')
    ax_alt.set_xlabel('t [s]'); ax_alt.set_ylabel('z [m]')
    ax_alt.set_title('Altitude')
    ax_alt.grid(True)

    # Roll
    ax_roll.plot(t, deg(states[:, 6]), 'r')
    ax_roll.set_xlabel('t [s]'); ax_roll.set_ylabel('φ [°]')
    ax_roll.set_title('Roll')
    ax_roll.grid(True)

    # Pitch
    ax_pit.plot(t, deg(states[:, 7]), 'g')
    ax_pit.set_xlabel('t [s]'); ax_pit.set_ylabel('θ [°]')
    ax_pit.set_title('Pitch')
    ax_pit.grid(True)

    # Yaw
    ax_yaw.plot(t, deg(states[:, 8]), 'm')
    ax_yaw.set_xlabel('t [s]'); ax_yaw.set_ylabel('ψ [°]')
    ax_yaw.set_title('Yaw')
    ax_yaw.grid(True)

    # Motor commands
    labels = ['FL', 'FR', 'RR', 'RL']
    colors = ['C0', 'C1', 'C2', 'C3']
    for i in range(4):
        ax_mot.plot(t, commands[:, i] * 100, color=colors[i], label=labels[i])
    ax_mot.set_xlabel('t [s]'); ax_mot.set_ylabel('Throttle [%]')
    ax_mot.set_title('Motor Commands')
    ax_mot.legend(fontsize=8); ax_mot.grid(True)
    ax_mot.set_ylim(0, 100)

    # 3D trajectory
    ax_3d.plot(states[:, 0], states[:, 1], states[:, 2], 'b', linewidth=0.8)
    ax_3d.scatter(*states[0, :3], color='g', s=40, label='start')
    ax_3d.scatter(*states[-1, :3], color='r', s=40, label='end')

    # Waypoint markers
    if waypoints:
        for i, wp in enumerate(waypoints):
            color = '#ff8800' if wp.keep_heading else '#aa00ff'
            ax_3d.scatter(wp.x, wp.y, wp.z, color=color, s=60, marker='^', zorder=5)
            ax_3d.text(wp.x, wp.y, wp.z + 0.05, f'WP{i}', fontsize=6, color=color)
        # Connect waypoints with dashed line
        wx = [wp.x for wp in waypoints]
        wy = [wp.y for wp in waypoints]
        wz = [wp.z for wp in waypoints]
        ax_3d.plot(wx, wy, wz, '--', color='gray', linewidth=0.7, alpha=0.6)
        # Legend entries
        ax_3d.scatter([], [], color='#ff8800', marker='^', label='WP keep_heading')
        ax_3d.scatter([], [], color='#aa00ff', marker='^', label='WP face-next')

    ax_3d.set_xlabel('x [m]'); ax_3d.set_ylabel('y [m]'); ax_3d.set_zlabel('z [m]')
    ax_3d.set_title('Trajectory')
    ax_3d.legend(fontsize=7)

    # Reference lines from setpoints dict
    if setpoints:
        for label, (sp_t, sp_v) in setpoints.items():
            for ax, v in zip([ax_alt, ax_roll, ax_pit, ax_yaw], sp_v):
                if v is not None:
                    ax.axhline(v, color='k', linestyle='--', linewidth=0.8, alpha=0.6)

    plt.show()


def plot_tuning_progress(history: list, best_gains: dict, title: str = "Monte Carlo Gain Search"):
    """
    Args:
        history: list of per-trial dicts (see tools/tune_gains.py::tune), each with
                 trial, phase, cost, diverged, best_cost, best_roll_rms_deg,
                 best_pitch_rms_deg, best_yaw_err_rms_deg, best_alt_rms_cm,
                 best_jerk_pct, best_progress.
        best_gains: winning gain dict, for the final bar chart.
        title: figure title.
    """
    trial      = np.array([r['trial'] for r in history])
    cost       = np.array([r['cost'] for r in history], dtype=float)
    best_cost  = np.array([r['best_cost'] for r in history], dtype=float)
    diverged   = np.array([r['diverged'] for r in history])
    is_local   = np.array([r['phase'] == 'local' for r in history])
    phase_boundary = trial[is_local][0] if is_local.any() else None

    fig = plt.figure(figsize=(15, 9))
    fig.suptitle(title, fontsize=14)
    gs = gridspec.GridSpec(2, 2, figure=fig, hspace=0.35, wspace=0.28)

    ax_cost = fig.add_subplot(gs[0, 0])
    ax_metr = fig.add_subplot(gs[0, 1])
    ax_jerk = fig.add_subplot(gs[1, 0])
    ax_gain = fig.add_subplot(gs[1, 1])

    # --- Cost convergence: raw per-trial cost (scatter) + best-so-far (line) ---
    cost_plot = np.clip(cost, 1e-2, None)   # log-safe
    ok = ~diverged
    ax_cost.scatter(trial[ok],  cost_plot[ok],  s=10, color='C0', alpha=0.5, label='trial cost')
    ax_cost.scatter(trial[~ok], cost_plot[~ok], s=10, color='C3', alpha=0.5, label='diverged')
    ax_cost.plot(trial, best_cost, color='k', linewidth=1.8, label='best so far')
    if phase_boundary is not None:
        ax_cost.axvline(phase_boundary, color='gray', linestyle='--', linewidth=1,
                        label='global → local')
    ax_cost.set_yscale('log')
    ax_cost.set_xlabel('trial'); ax_cost.set_ylabel('cost (log)')
    ax_cost.set_title('Cost convergence')
    ax_cost.legend(fontsize=8); ax_cost.grid(True, which='both', alpha=0.3)

    # --- Best-so-far tracking-error metrics over the search ---
    ax_metr.plot(trial, [r['best_xy_rms_cm']       for r in history], label='xy position RMS [cm]', linewidth=2)
    ax_metr.plot(trial, [r['best_alt_rms_cm']      for r in history], label='alt RMS [cm]', linewidth=2)
    ax_metr.plot(trial, [r['best_roll_rms_deg']    for r in history], label='roll RMS [deg]')
    ax_metr.plot(trial, [r['best_pitch_rms_deg']   for r in history], label='pitch RMS [deg]')
    ax_metr.plot(trial, [r['best_yaw_err_rms_deg'] for r in history], label='yaw err RMS [deg]')
    if phase_boundary is not None:
        ax_metr.axvline(phase_boundary, color='gray', linestyle='--', linewidth=1)
    ax_metr.set_xlabel('trial'); ax_metr.set_ylabel('error (best-so-far)')
    ax_metr.set_title('Best-so-far tracking error')
    ax_metr.legend(fontsize=8); ax_metr.grid(True, alpha=0.3)

    # --- Best-so-far jerk% and mission progress ---
    ax_jerk.plot(trial, [r['best_jerk_pct'] for r in history], color='C4', label='motor jerk [%/step]')
    ax_jerk.set_xlabel('trial'); ax_jerk.set_ylabel('jerk [%/step]', color='C4')
    ax_jerk.tick_params(axis='y', labelcolor='C4')
    if phase_boundary is not None:
        ax_jerk.axvline(phase_boundary, color='gray', linestyle='--', linewidth=1)
    ax_prog = ax_jerk.twinx()
    ax_prog.plot(trial, [r['best_progress'] * 100 for r in history], color='C2', label='mission progress [%]')
    ax_prog.set_ylabel('mission progress [%]', color='C2')
    ax_prog.tick_params(axis='y', labelcolor='C2')
    ax_prog.set_ylim(0, 105)
    ax_jerk.set_title('Best-so-far jerk & mission progress')
    ax_jerk.grid(True, alpha=0.3)

    # --- Final winning gains, grouped by axis ---
    axes_order = ['roll', 'pitch', 'yaw', 'x', 'y', 'z']
    terms = ['kp', 'ki', 'kd']
    x = np.arange(len(axes_order))
    width = 0.25
    for j, term in enumerate(terms):
        vals = [best_gains[f'{term}_{ax}'] for ax in axes_order]
        ax_gain.bar(x + (j - 1) * width, vals, width, label=term)
    ax_gain.set_xticks(x); ax_gain.set_xticklabels(axes_order)
    ax_gain.set_ylabel('gain value')
    ax_gain.set_title('Winning gains by axis')
    ax_gain.legend(fontsize=8); ax_gain.grid(True, axis='y', alpha=0.3)

    plt.show()


def print_metrics(t: np.ndarray, states: np.ndarray, setpoint_z: float = 1.0):
    """Print basic stability metrics to console."""
    z = states[:, 2]
    phi_deg   = np.degrees(states[:, 6])
    theta_deg = np.degrees(states[:, 7])

    ss_idx = int(len(t) * 0.8)  # last 20% of sim = steady state
    z_ss = np.mean(z[ss_idx:])
    z_err = abs(z_ss - setpoint_z)
    z_peak = np.max(np.abs(z - setpoint_z))

    print("\n--- Simulation Metrics ---")
    print(f"  Altitude steady-state error : {z_err*100:.1f} cm")
    print(f"  Altitude peak overshoot     : {z_peak*100:.1f} cm")
    print(f"  Roll  RMS (steady-state)    : {np.std(phi_deg[ss_idx:]):.2f}°")
    print(f"  Pitch RMS (steady-state)    : {np.std(theta_deg[ss_idx:]):.2f}°")

    # Settling time (within 5% of setpoint for altitude)
    band = 0.05 * setpoint_z
    settled = np.where(np.abs(z - setpoint_z) < band)[0]
    if len(settled) > 0:
        # First time it enters and stays
        settle_t = t[settled[0]]
        print(f"  Altitude settling time (5%) : {settle_t:.2f} s")
    else:
        print(f"  Altitude did not settle within simulation time.")
    print("--------------------------")
