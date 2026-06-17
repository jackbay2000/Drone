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
