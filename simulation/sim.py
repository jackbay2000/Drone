"""
Drone physics simulation + PID tuning UI.

Usage:
    cd simulation
    python sim.py
"""

import os, sys
import numpy as np
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Slider, Button
from mpl_toolkits.mplot3d import Axes3D   # noqa: F401

sys.path.insert(0, os.path.dirname(__file__))
from ctrl   import Controller, Gains, load_gains, save_gains, GAINS_KEYS
from physics import DronePhysics
from drone_config import build_drone, ARM_LENGTH_M, MOTOR_TAU

GAINS_FILE = os.path.join(os.path.dirname(__file__), "..", "Drone_Main", "gains.txt")

SIM_DT     = 0.001
SIM_PERIOD = 8.0
WP_RADIUS  = 0.15

print("Computing mass properties from STL files … ", end="", flush=True)
_mass, _com, _I, _Kt, _Kq = build_drone()
print(f"done.  Total mass: {_mass*1000:.0f} g")

drone = DronePhysics(_mass, _I, _Kt, _Kq, ARM_LENGTH_M, MOTOR_TAU)


def _run_sim(waypoints, gains, duration):
    """Run the sim with waypoint sequencing matching Drone_Main.ino."""
    ctrl = Controller(gains)
    drone.reset()

    wp_idx = 0
    landed = False

    N = int(duration / SIM_DT)
    ts = np.zeros(N)
    roll_h = np.zeros(N); pitch_h = np.zeros(N); yaw_h = np.zeros(N)
    x_h = np.zeros(N); y_h = np.zeros(N); z_h = np.zeros(N)
    m_h = np.zeros((N, 4))

    for i in range(N):
        t = i * SIM_DT
        ts[i] = t
        roll, pitch, yaw = drone.get_roll_pitch_yaw()
        px, py, pz = drone.get_pos_frame()
        roll_h[i] = np.degrees(roll)
        pitch_h[i] = np.degrees(pitch)
        yaw_h[i] = np.degrees(yaw)
        x_h[i], y_h[i], z_h[i] = px, py, pz

        if landed:
            u = np.zeros(4)
        else:
            wp = waypoints[wp_idx]
            ex, ey, ez = wp[0] - px, wp[1] - py, wp[2] - pz
            if np.sqrt(ex**2 + ey**2 + ez**2) < WP_RADIUS:
                if wp[2] == 0.0:
                    landed = True
                    u = np.zeros(4)
                    m_h[i] = u
                    drone.set_motor_commands(u)
                    drone.step(SIM_DT)
                    continue
                if wp_idx < len(waypoints) - 1:
                    wp_idx += 1
                    wp = waypoints[wp_idx]

            u = ctrl.update(t, SIM_DT, roll, pitch, yaw,
                            px, py, pz, wp[0], wp[1], wp[2])

        m_h[i] = u
        drone.set_motor_commands(u)
        drone.step(SIM_DT)

    return dict(t=ts, roll=roll_h, pitch=pitch_h, yaw=yaw_h,
                x=x_h, y=y_h, z=z_h, motors=m_h)


def _hover_test(gains):
    return _run_sim([(0, 0, 1.0)], gains, SIM_PERIOD)


def _step_test(gains, axis, step_val=0.3):
    waypoints = [(0, 0, 1.0)]
    if axis == "x":
        waypoints.append((step_val, 0, 1.0))
    elif axis == "y":
        waypoints.append((0, step_val, 1.0))
    elif axis == "z":
        waypoints.append((0, 0, 1.0 + step_val))
    return _run_sim(waypoints, gains, SIM_PERIOD)


def _waypoint_test(gains):
    waypoints = [
        (0, 0, 1.0),
        (2.0, 0, 1.0),
        (2.0, 1.5, 1.0),
        (0, 0, 1.0),
        (0, 0, 0.0),
    ]
    return _run_sim(waypoints, gains, SIM_PERIOD * 3)


class SimUI:
    SLIDER_SPECS = [
        ("kp_roll",  "kp_roll",  0.0, 10.0, 0.01),
        ("ki_roll",  "ki_roll",  0.0,  1.0, 0.001),
        ("kd_roll",  "kd_roll",  0.0,  2.0, 0.005),
        ("kp_pitch", "kp_pitch", 0.0, 10.0, 0.01),
        ("ki_pitch", "ki_pitch", 0.0,  1.0, 0.001),
        ("kd_pitch", "kd_pitch", 0.0,  2.0, 0.005),
        ("kp_yaw",   "kp_yaw",   0.0,  8.0, 0.01),
        ("ki_yaw",   "ki_yaw",   0.0,  0.5, 0.001),
        ("kd_yaw",   "kd_yaw",   0.0,  1.0, 0.005),
        ("kp_z",     "kp_z",     0.0,  2.0, 0.005),
        ("ki_z",     "ki_z",     0.0,  0.2, 0.001),
        ("kd_z",     "kd_z",     0.0,  2.0, 0.005),
        ("kp_x",     "kp_x",     0.0,  1.0, 0.005),
        ("ki_x",     "ki_x",     0.0,  0.1, 0.001),
        ("kd_x",     "kd_x",     0.0,  1.0, 0.005),
        ("kp_y",     "kp_y",     0.0,  1.0, 0.005),
        ("ki_y",     "ki_y",     0.0,  0.1, 0.001),
        ("kd_y",     "kd_y",     0.0,  1.0, 0.005),
    ]

    def __init__(self):
        self._gains  = load_gains(GAINS_FILE) if os.path.exists(GAINS_FILE) else Gains()
        self._result = None
        self._test   = "hover"
        self._build_figure()
        self._run_sim()
        self._refresh_plots()

    def _build_figure(self):
        self.fig = plt.figure(figsize=(18, 11), constrained_layout=False)
        self.fig.suptitle("Drone Simulation — PID Tuning", fontsize=13)

        gs_outer = gridspec.GridSpec(1, 2, figure=self.fig,
                                     left=0.04, right=0.99,
                                     top=0.94, bottom=0.04,
                                     wspace=0.32)

        gs_plots = gridspec.GridSpecFromSubplotSpec(
            3, 2, subplot_spec=gs_outer[0], hspace=0.45, wspace=0.35)
        self.ax_roll  = self.fig.add_subplot(gs_plots[0, 0])
        self.ax_pitch = self.fig.add_subplot(gs_plots[0, 1])
        self.ax_yaw   = self.fig.add_subplot(gs_plots[1, 0])
        self.ax_z     = self.fig.add_subplot(gs_plots[1, 1])
        self.ax_xy    = self.fig.add_subplot(gs_plots[2, 0])
        self.ax_mot   = self.fig.add_subplot(gs_plots[2, 1])

        for ax, title, ylabel in [
            (self.ax_roll,  "Roll",    "deg"),
            (self.ax_pitch, "Pitch",   "deg"),
            (self.ax_yaw,   "Yaw",     "deg"),
            (self.ax_z,     "Altitude","m"),
            (self.ax_xy,    "XY track","Y (m)"),
            (self.ax_mot,   "Motors",  "throttle"),
        ]:
            ax.set_title(title, fontsize=9)
            ax.set_ylabel(ylabel, fontsize=8)
            ax.set_xlabel("t (s)" if title != "XY track" else "X (m)", fontsize=8)
            ax.tick_params(labelsize=7)
            ax.grid(True, linewidth=0.5, alpha=0.5)

        gs_right = gridspec.GridSpecFromSubplotSpec(
            2, 1, subplot_spec=gs_outer[1],
            height_ratios=[2, 1], hspace=0.4)

        n_sliders = len(self.SLIDER_SPECS)
        slider_fig_left  = 0.545
        slider_fig_right = 0.99
        slider_top       = 0.90
        slider_bottom    = 0.52
        slider_h         = (slider_top - slider_bottom) / n_sliders
        self.sliders = {}

        for i, (attr, label, vmin, vmax, vstep) in enumerate(self.SLIDER_SPECS):
            y    = slider_top - (i + 1) * slider_h + 0.005
            ax_s = self.fig.add_axes([slider_fig_left, y,
                                      slider_fig_right - slider_fig_left - 0.01,
                                      slider_h * 0.55])
            val = getattr(self._gains, attr)
            s   = Slider(ax_s, label, vmin, vmax, valinit=val, valstep=vstep)
            s.label.set_fontsize(7)
            s.valtext.set_fontsize(7)
            s.on_changed(lambda v, a=attr: self._on_slider(a, v))
            self.sliders[attr] = s

        btn_y   = 0.48
        btn_h   = 0.04
        btn_w   = 0.07
        btn_gap = 0.005
        bx = slider_fig_left
        def _btn(label, cb):
            nonlocal bx
            ax = self.fig.add_axes([bx, btn_y, btn_w, btn_h])
            b  = Button(ax, label, color="0.85", hovercolor="0.7")
            b.label.set_fontsize(8)
            b.on_clicked(cb)
            bx += btn_w + btn_gap
            return b

        self.btn_hover  = _btn("Hover",   lambda _: self._set_test("hover"))
        self.btn_x      = _btn("X step",  lambda _: self._set_test("x"))
        self.btn_y      = _btn("Y step",  lambda _: self._set_test("y"))
        self.btn_z      = _btn("Z step",  lambda _: self._set_test("z"))
        self.btn_wp     = _btn("Waypts",  lambda _: self._set_test("wp"))
        self.btn_run    = _btn("Run",     lambda _: self._run_and_refresh())
        self.btn_export = _btn("Export",  lambda _: self._export_gains())
        self.btn_import = _btn("Import",  lambda _: self._import_gains())

        self.ax3d = self.fig.add_axes(
            [slider_fig_left, 0.05, 0.42, 0.38], projection="3d")
        self.ax3d.set_title("Final attitude", fontsize=9)
        self.ax3d.set_box_aspect([1, 1, 1])

    def _on_slider(self, attr, val):
        setattr(self._gains, attr, float(val))

    def _set_test(self, name):
        self._test = name
        self._run_and_refresh()

    def _run_and_refresh(self):
        self._run_sim()
        self._refresh_plots()

    STEP_VALS = {"x": 0.3, "y": 0.3, "z": 0.5}

    def _run_sim(self):
        g = self._gains
        if self._test == "hover":
            self._result = _hover_test(g)
        elif self._test == "wp":
            self._result = _waypoint_test(g)
        else:
            sv = self.STEP_VALS.get(self._test, 0.3)
            self._result = _step_test(g, self._test, step_val=sv)

    def _refresh_plots(self):
        r = self._result
        t = r["t"]

        def _plot(ax, ys, labels, colors, ylabel=None, hline=None):
            ax.cla()
            ax.grid(True, linewidth=0.5, alpha=0.5)
            if ylabel: ax.set_ylabel(ylabel, fontsize=8)
            ax.tick_params(labelsize=7)
            for y, lab, col in zip(ys, labels, colors):
                ax.plot(t, y, color=col, linewidth=0.9, label=lab)
            if hline is not None:
                ax.axhline(hline, color="k", linewidth=0.5, linestyle="--")
            if len(labels) > 1:
                ax.legend(fontsize=7, loc="upper right")

        _plot(self.ax_roll,  [r["roll"]],  ["roll"],  ["tab:blue"],   "deg", 0)
        _plot(self.ax_pitch, [r["pitch"]], ["pitch"], ["tab:orange"], "deg", 0)
        _plot(self.ax_yaw,   [r["yaw"]],   ["yaw"],   ["tab:green"],  "deg", 0)
        _plot(self.ax_z,     [r["z"]],     ["z"],     ["tab:red"],    "m",   1.0)

        self.ax_roll .set_title("Roll",     fontsize=9)
        self.ax_pitch.set_title("Pitch",    fontsize=9)
        self.ax_yaw  .set_title("Yaw",      fontsize=9)
        self.ax_z    .set_title("Altitude",  fontsize=9)
        for ax in [self.ax_roll, self.ax_pitch, self.ax_yaw, self.ax_z]:
            ax.set_xlabel("t (s)", fontsize=8)

        self.ax_xy.cla()
        self.ax_xy.grid(True, linewidth=0.5, alpha=0.5)
        self.ax_xy.plot(r["x"], r["y"], color="tab:purple", linewidth=0.8)
        self.ax_xy.plot(r["x"][0], r["y"][0], "go", markersize=5, label="start")
        self.ax_xy.plot(r["x"][-1], r["y"][-1], "rs", markersize=5, label="end")
        self.ax_xy.set_title("XY Track", fontsize=9)
        self.ax_xy.set_xlabel("X (m)", fontsize=8)
        self.ax_xy.set_ylabel("Y (m)", fontsize=8)
        self.ax_xy.set_aspect("equal", "datalim")
        self.ax_xy.legend(fontsize=7)
        self.ax_xy.tick_params(labelsize=7)

        colors_m = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
        self.ax_mot.cla()
        self.ax_mot.grid(True, linewidth=0.5, alpha=0.5)
        for j, col in enumerate(colors_m):
            self.ax_mot.plot(t, r["motors"][:, j], color=col,
                             linewidth=0.8, label=f"M{j+1}")
        self.ax_mot.axhline(0.5, color="k", linewidth=0.5, linestyle="--")
        self.ax_mot.set_ylim(0, 1.05)
        self.ax_mot.set_title("Motors", fontsize=9)
        self.ax_mot.set_xlabel("t (s)", fontsize=8)
        self.ax_mot.set_ylabel("throttle", fontsize=8)
        self.ax_mot.legend(fontsize=7, loc="upper right")
        self.ax_mot.tick_params(labelsize=7)

        self._draw_drone_3d(r["roll"][-1], r["pitch"][-1], r["yaw"][-1])
        self.fig.canvas.draw_idle()

    def _draw_drone_3d(self, roll_deg, pitch_deg, yaw_deg):
        ax = self.ax3d
        ax.cla()
        ax.set_title(f"Final: roll={roll_deg:.1f}° pitch={pitch_deg:.1f}° yaw={yaw_deg:.1f}°",
                     fontsize=8)

        roll  = np.radians(roll_deg)
        pitch = np.radians(pitch_deg)
        yaw   = np.radians(yaw_deg)

        cy, sy = np.cos(yaw),   np.sin(yaw)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cr, sr = np.cos(roll),  np.sin(roll)
        Ry = np.array([[ cr, 0, sr], [0,1,0], [-sr,0,cr]])
        Rx = np.array([[1,0,0], [0,cp,-sp], [0,sp,cp]])
        Rz = np.array([[cy,-sy,0],[sy,cy,0],[0,0,1]])
        R  = Rz @ Rx @ Ry

        arm = 0.12
        r   = arm / np.sqrt(2)
        colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
        local_pos = np.array([[-r,r,0],[r,r,0],[-r,-r,0],[r,-r,0]])
        for i, (lp, col) in enumerate(zip(local_pos, colors)):
            wp = R @ lp
            ax.plot([0, wp[0]], [0, wp[1]], [0, wp[2]], color=col, linewidth=2.5)
            ax.scatter(*wp, color=col, s=30)
            ax.text(wp[0]*1.1, wp[1]*1.1, wp[2]*1.1, f"M{i+1}", fontsize=7)

        bz = R @ np.array([0, 0, 0.08])
        ax.quiver(0,0,0, bz[0],bz[1],bz[2], color="black", linewidth=1.5)
        ax.quiver(0,0,0, 0,0,0.1, color="gray", linewidth=0.8, linestyle="--")

        ax.set_xlim(-0.15, 0.15); ax.set_ylim(-0.15, 0.15); ax.set_zlim(-0.05, 0.2)
        ax.set_xlabel("X", fontsize=7); ax.set_ylabel("Y", fontsize=7)
        ax.set_zlabel("Z", fontsize=7)
        ax.tick_params(labelsize=6)

    def _export_gains(self):
        save_gains(self._gains, GAINS_FILE)
        print(f"Gains saved to: {os.path.abspath(GAINS_FILE)}")
        self.fig.suptitle(f"Exported → {os.path.abspath(GAINS_FILE)}", fontsize=11)
        self.fig.canvas.draw_idle()

    def _import_gains(self):
        g = load_gains(GAINS_FILE)
        self._gains = g
        for attr, s in self.sliders.items():
            s.set_val(getattr(g, attr))
        self._run_and_refresh()

    def show(self):
        plt.show()


if __name__ == "__main__":
    ui = SimUI()
    ui.show()
