"""
Drone assembly configuration and inertia calculation.

Body frame convention (used throughout the simulation):
  X = drone right
  Y = drone forward (+Y is forward, matching the flight controller code)
  Z = drone up

Roll  φ : rotation about Y (forward),  positive = right bank
Pitch θ : rotation about X (right),    positive = nose UP
Yaw   ψ : rotation about Z (up),       positive = CCW from above

The physical IMU (MPU6050) likely reports:
  imu_roll  =  φ         (same sign)
  imu_pitch = -θ         (positive = nose DOWN in firmware convention)
  imu_yaw   = -ψ         (positive = CW = increasing heading)
These negations are applied inside ctrl.py.

Motor layout (X-frame, viewed from above):
         +Y (forward)
    M1(FL,CCW)  M2(FR,CW)
          |       |
         cross frame
          |       |
    M3(RL,CW)  M4(RR,CCW)
"""

import numpy as np
from stl_mass import (
    get_all_parts, shift_inertia, box_inertia,
    BREADBOARD_MASS_G, BREADBOARD_DIM,
)

# ── Configurable parameters ───────────────────────────────────────────────────

# Distance from drone centre to each motor (metres).
# Measure from the centre of the body to the motor bell.
ARM_LENGTH_M = 0.120          # 120 mm — adjust to match your physical frame

# Motor properties
MOTOR_MASS_G  = 26.0          # typical 2205 BLDC motor
ESC_MASS_G    = 10.0          # each ESC

# Additional electronics in the central body
TEENSY_MASS_G   = 15.0
SENSOR_MISC_G   = 20.0        # IMU, rangefinder, flow sensor, wiring

# Battery (3S 1300 mAh LiPo, mounted below centre of body)
BATTERY_MASS_G  = 110.0
BATTERY_DIM_MM  = np.array([70.0, 35.0, 20.0])  # length × width × height (mm)
# Position of battery CoM relative to drone origin (body frame, mm)
# Negative Z = below the main body, slightly behind centre
BATTERY_POS_MM  = np.array([0.0, -10.0, -30.0])

# Propellers (treated as point masses at motor positions; axial inertia ignored)
PROP_MASS_G = 5.0

# ── Derived motor positions (body frame, metres) ──────────────────────────────
_r = ARM_LENGTH_M / np.sqrt(2)   # component along each axis
MOTOR_POSITIONS_M = np.array([
    [-_r,  _r, 0],   # M1: Front-Left  (CCW)
    [ _r,  _r, 0],   # M2: Front-Right (CW)
    [-_r, -_r, 0],   # M3: Rear-Left   (CW)
    [ _r, -_r, 0],   # M4: Rear-Right  (CCW)
])

# Yaw sign for each motor: +1 if CCW prop (positive reaction CW on frame),
# expressed in the CCW-positive Z-up frame → CCW props give negative τ_z
# τ_z (CCW+) = k_q * (-F1 + F2 + F3 - F4)
MOTOR_YAW_SIGN = np.array([-1, +1, +1, -1])   # CCW=-1, CW=+1


# ── Motor thrust / torque model ───────────────────────────────────────────────

# Thrust coefficient: F_i = K_THRUST * u_i  (u_i ∈ [0,1])
# At hover (u = HOVER_THROTTLE = 0.5): 4 * K_THRUST * 0.5 = m * g
# → K_THRUST = m_total * g / 2.0   (filled in after mass calculation below)

HOVER_THROTTLE = 0.50    # from controller.h HOVER_THROTTLE

# Ratio of drag torque to thrust for one motor (k_q / k_t)
# Typical range: 0.008–0.015 for small hobby motors
YAW_TORQUE_RATIO = 0.012

# First-order motor lag time constant (seconds)
MOTOR_TAU = 0.05


# ── Assembly → total mass and inertia ─────────────────────────────────────────

def build_drone():
    """
    Returns
    -------
    total_mass   : float  (kg)
    drone_com    : (3,)   (m, body frame — will be near zero by design)
    I_total      : (3,3)  (kg·m², about drone CoM)
    K_THRUST     : float  (N per unit throttle per motor)
    K_TORQUE     : float  (N·m per unit throttle per motor, for yaw)
    """
    parts = get_all_parts()

    components = []   # list of (mass_g, com_mm_body, I_g_mm2_about_own_com)

    # ── circ-bay-base (bottom body) ────────────────────────────────────────
    m_base, c_base, I_base = parts["circ-bay-base"]
    # STL origin is at one corner. Centre of footprint ≈ (50.6, 61.1, 24.3) mm.
    # We define drone body origin at the STL's local (50.6, 61.1, 0) — i.e. the
    # geometric centre of the base plate's footprint at ground level.
    BASE_ORIGIN_MM = np.array([c_base[0], c_base[1], 0.0])
    c_base_body = c_base - BASE_ORIGIN_MM
    components.append((m_base, c_base_body, I_base))

    # breadboard inside circ-bay-base — centred at the same XY, 1/2 height up
    bb1_pos = np.array([0.0, 0.0, c_base[2]])  # approx at body CoM height
    I_bb1   = box_inertia(BREADBOARD_MASS_G, BREADBOARD_DIM)
    components.append((BREADBOARD_MASS_G, bb1_pos, I_bb1))

    # ── top-bb-holder (top plate, sits on top of body) ─────────────────────
    m_top, c_top, I_top = parts["top-bb-holder"]
    # Mounted on top: Z offset = circ-bay-base height + half of top plate
    TOP_Z_OFFSET = 83.8 + 6.0   # body height + half plate thickness (mm)
    c_top_local  = np.array([c_top[0] - c_base[0],
                              c_top[1] - c_base[1],
                              TOP_Z_OFFSET + c_top[2]])
    components.append((m_top, c_top_local, I_top))

    # breadboard in top-bb-holder
    bb2_pos = np.array([0.0, 0.0, TOP_Z_OFFSET + c_top[2]])
    I_bb2   = box_inertia(BREADBOARD_MASS_G, BREADBOARD_DIM)
    components.append((BREADBOARD_MASS_G, bb2_pos, I_bb2))

    # ── prop-arms (4 off, rotated to ±45°) ────────────────────────────────
    m_arm, c_arm_local, I_arm_stl = parts["prop-arm"]
    # STL arm runs along its local X axis.  We need to rotate each arm so its
    # X-axis points toward the corresponding motor position in body frame.
    ARM_ANGLES_DEG = [135, 45, 225, 315]   # angle from +X axis to arm direction
    ARM_Z_MM = 40.0   # height of arm attachment point (mm, roughly mid-body)

    for angle_deg in ARM_ANGLES_DEG:
        theta = np.radians(angle_deg)
        # Rotation matrix about Z (arm rotated in XY plane)
        Rz = np.array([
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta),  np.cos(theta), 0],
            [0,              0,             1],
        ])
        # Arm CoM in body frame: rotate the STL-frame CoM
        c_arm_stl = np.array([c_arm_local[0], c_arm_local[1] - 16.5, 0.0])
        c_arm_body = Rz @ c_arm_stl
        c_arm_body[2] = ARM_Z_MM + c_arm_local[2]

        # Rotate inertia tensor: I_body = R @ I_stl @ R.T
        I_arm_body = Rz @ I_arm_stl @ Rz.T

        components.append((m_arm, c_arm_body, I_arm_body))

    # ── Motors (point masses at motor positions) ───────────────────────────
    for pos_m in MOTOR_POSITIONS_M:
        pos_mm = pos_m * 1000.0
        pos_mm[2] = ARM_Z_MM + 40.0   # motors are at top of arm
        components.append((MOTOR_MASS_G + PROP_MASS_G, pos_mm, np.zeros((3, 3))))

    # ── ESCs (point masses at 60% arm length) ─────────────────────────────
    for pos_m in MOTOR_POSITIONS_M:
        esc_pos_mm = pos_m * 0.6 * 1000.0
        esc_pos_mm[2] = ARM_Z_MM
        components.append((ESC_MASS_G, esc_pos_mm, np.zeros((3, 3))))

    # ── Electronics (in body) ─────────────────────────────────────────────
    components.append((TEENSY_MASS_G,  np.zeros(3), np.zeros((3, 3))))
    components.append((SENSOR_MISC_G,  np.zeros(3), np.zeros((3, 3))))

    # ── Battery ───────────────────────────────────────────────────────────
    I_bat = box_inertia(BATTERY_MASS_G, BATTERY_DIM_MM)
    components.append((BATTERY_MASS_G, BATTERY_POS_MM, I_bat))

    # ── Compute total CoM ─────────────────────────────────────────────────
    masses = np.array([c[0] for c in components])
    coms   = np.array([c[1] for c in components])
    total_mass_g = np.sum(masses)
    drone_com_mm = np.einsum("i,ij->j", masses, coms) / total_mass_g

    # ── Sum inertia tensors about drone CoM ───────────────────────────────
    I_total_g_mm2 = np.zeros((3, 3))
    for (mass_g, com_mm, I_own) in components:
        r_mm = com_mm - drone_com_mm    # vector from drone CoM to part CoM
        I_at_drone_com = shift_inertia(I_own, mass_g, r_mm)
        I_total_g_mm2 += I_at_drone_com

    # Convert to SI
    total_mass_kg = total_mass_g * 1e-3
    drone_com_m   = drone_com_mm * 1e-3
    I_total_kg_m2 = I_total_g_mm2 * 1e-9   # g·mm² → kg·m²

    g = 9.81
    K_THRUST = total_mass_kg * g / (4.0 * HOVER_THROTTLE)   # N per unit throttle
    K_TORQUE = K_THRUST * YAW_TORQUE_RATIO * ARM_LENGTH_M   # N·m per unit throttle

    return total_mass_kg, drone_com_m, I_total_kg_m2, K_THRUST, K_TORQUE


if __name__ == "__main__":
    m, com, I, Kt, Kq = build_drone()
    print(f"Total mass : {m*1000:.0f} g")
    print(f"Drone CoM  : ({com[0]*1000:.1f}, {com[1]*1000:.1f}, {com[2]*1000:.1f}) mm (body frame)")
    print(f"K_thrust   : {Kt:.4f} N per unit throttle per motor")
    print(f"K_torque   : {Kq:.6f} N·m per unit throttle per motor")
    print(f"Ixx = {I[0,0]*1e6:.4f}  Iyy = {I[1,1]*1e6:.4f}  Izz = {I[2,2]*1e6:.4f}  (x1e-6 kg.m2)")
