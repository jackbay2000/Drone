"""
STL mass-property extraction using the signed-tetrahedral-decomposition method.
Every triangle + the origin forms a tetrahedron; signed volumes sum to the true
solid volume regardless of mesh orientation.
"""

import struct
import numpy as np

# ── Material constants ────────────────────────────────────────────────────────
PLA_DENSITY_G_PER_MM3 = 0.00124          # solid PLA
INFILL_FRACTION       = 0.30             # effective density factor (walls + infill)
PART_DENSITY          = PLA_DENSITY_G_PER_MM3 * INFILL_FRACTION  # g/mm³

# Mini breadboard (half-size): 85 × 55 × 10 mm, ~17 g
BREADBOARD_MASS_G = 17.0
BREADBOARD_DIM    = np.array([85.0, 55.0, 10.0])   # mm

# ── STL I/O ──────────────────────────────────────────────────────────────────

def load_stl(path: str) -> np.ndarray:
    """Return (N, 3, 3) float64 array of triangle vertices (mm)."""
    with open(path, "rb") as f:
        f.read(80)
        n = struct.unpack("<I", f.read(4))[0]
        tris = np.zeros((n, 3, 3), dtype=np.float64)
        for i in range(n):
            f.read(12)
            for j in range(3):
                tris[i, j] = struct.unpack("<3f", f.read(12))
            f.read(2)
    return tris


# ── Core computation ─────────────────────────────────────────────────────────

def mass_properties(tris: np.ndarray, density: float = PART_DENSITY):
    """
    Compute mass, center-of-mass, and inertia tensor for a solid mesh.

    Parameters
    ----------
    tris    : (N, 3, 3) vertex array in mm
    density : g/mm³

    Returns
    -------
    mass    : float   (g)
    com     : (3,)    (mm, in the STL's local coordinate frame)
    inertia : (3, 3)  (g·mm², about the part's own CoM)
    """
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]

    # Signed volume of each tetrahedron (apex at world origin)
    cross = np.cross(v1, v2)                          # (N, 3)
    sv    = np.einsum("ni,ni->n", v0, cross) / 6.0   # (N,)

    V = np.sum(sv)
    if V < 0:
        V, sv = -V, -sv

    # Center of mass (weighted centroid of tetrahedra)
    inv4V = 1.0 / (4.0 * V)
    com   = np.array([
        np.sum(sv * (v0[:, k] + v1[:, k] + v2[:, k])) * inv4V
        for k in range(3)
    ])

    mass = V * density

    # Inertia tensor about com via Mirtich (1996) tetrahedral formula
    p0 = v0 - com
    p1 = v1 - com
    p2 = v2 - com
    x1, y1, z1 = p0[:, 0], p0[:, 1], p0[:, 2]
    x2, y2, z2 = p1[:, 0], p1[:, 1], p1[:, 2]
    x3, y3, z3 = p2[:, 0], p2[:, 1], p2[:, 2]

    f_d = density / 10.0
    f_od = density / 20.0

    def _sum(a, b, c):
        return np.sum(sv * (a*a + b*b + c*c + a*b + a*c + b*c))

    Ixx = f_d  * (_sum(y1, y2, y3) + _sum(z1, z2, z3))
    Iyy = f_d  * (_sum(x1, x2, x3) + _sum(z1, z2, z3))
    Izz = f_d  * (_sum(x1, x2, x3) + _sum(y1, y2, y3))

    Ixy = f_od * np.sum(sv * (2*x1*y1 + x1*y2 + x1*y3 +
                               x2*y1 + 2*x2*y2 + x2*y3 +
                               x3*y1 + x3*y2 + 2*x3*y3))
    Ixz = f_od * np.sum(sv * (2*x1*z1 + x1*z2 + x1*z3 +
                               x2*z1 + 2*x2*z2 + x2*z3 +
                               x3*z1 + x3*z2 + 2*x3*z3))
    Iyz = f_od * np.sum(sv * (2*y1*z1 + y1*z2 + y1*z3 +
                               y2*z1 + 2*y2*z2 + y2*z3 +
                               y3*z1 + y3*z2 + 2*y3*z3))

    I = np.array([
        [ Ixx, -Ixy, -Ixz],
        [-Ixy,  Iyy, -Iyz],
        [-Ixz, -Iyz,  Izz],
    ])
    return mass, com, I


# ── Parallel-axis theorem helper ─────────────────────────────────────────────

def shift_inertia(I_com: np.ndarray, mass: float, r: np.ndarray) -> np.ndarray:
    """
    Shift inertia tensor from a part's own CoM to a new reference point.
    r : (3,) vector from part CoM to new reference point (mm).
    Returns I about the new point (g·mm²).
    """
    return I_com + mass * (np.dot(r, r) * np.eye(3) - np.outer(r, r))


def box_inertia(mass: float, dims: np.ndarray) -> np.ndarray:
    """Solid-box inertia tensor about the box centroid (g·mm²)."""
    dx, dy, dz = dims
    Ixx = mass * (dy**2 + dz**2) / 12.0
    Iyy = mass * (dx**2 + dz**2) / 12.0
    Izz = mass * (dx**2 + dy**2) / 12.0
    return np.diag([Ixx, Iyy, Izz])


# ── Pre-computed part library ─────────────────────────────────────────────────
# Each entry: (mass_g, com_local_mm, inertia_about_com_g_mm2)
# com_local is in the STL file's own coordinate frame.

import os as _os

_STL_DIR = _os.path.join(_os.path.dirname(__file__), "..")

def _load_part(name):
    path = _os.path.join(_STL_DIR, f"{name}.STL")
    tris = load_stl(path)
    return mass_properties(tris)


def get_all_parts():
    """
    Return dict of {part_name: (mass, com_local, inertia_about_com)}.
    Loads from disk on first call.
    """
    parts = {}
    for name in ("circ-bay-base", "prop-arm", "top-bb-holder"):
        m, c, I = _load_part(name)
        parts[name] = (m, c, I)
    return parts


if __name__ == "__main__":
    for name, (m, c, I) in get_all_parts().items():
        print(f"{name}:")
        print(f"  mass = {m:.1f} g")
        print(f"  CoM  = ({c[0]:.1f}, {c[1]:.1f}, {c[2]:.1f}) mm")
        print(f"  Ixx={I[0,0]:.0f}  Iyy={I[1,1]:.0f}  Izz={I[2,2]:.0f}  g*mm^2")
