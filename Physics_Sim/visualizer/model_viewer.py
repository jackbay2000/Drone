"""
3D vehicle assembly viewer.

Renders the component layout from component_list.json in a matplotlib 3D window:
  - Components with a .3mf or .stl file are drawn as translucent meshes at
    the position and orientation specified in the component list.
  - Point-mass components (no geometry file) are drawn as colored dots with a
    label showing name, mass (g), and center-of-mass position.

Call show_vehicle_model() before plt.show() — it creates the figure but does
not block, so it appears alongside any other open figures.
"""

import json
import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D          # noqa: F401
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

try:
    import trimesh
    _TRIMESH = True
except ImportError:
    _TRIMESH = False

_COMPONENTS_PATH = os.path.join(
    os.path.dirname(__file__), '..', 'inputs', 'components', 'component_list.json')
_MODELS_DIR = os.path.join(
    os.path.dirname(__file__), '..', '..', 'Models')

_DOT_COLORS = [
    '#e6194b', '#3cb44b', '#4363d8', '#f58231', '#911eb4',
    '#42d4f4', '#f032e6', '#bfef45', '#469990', '#dcbeff',
]

_MAX_TRIANGLES = 15_000   # downsample meshes above this for rendering speed


def show_vehicle_model(components_path: str = None, models_dir: str = None,
                       verbose: bool = True):
    """
    Create a 3D matplotlib figure showing the assembled vehicle.

    Components with a file_3mf or stl_file entry are loaded and rendered as
    translucent meshes. All other components are rendered as a colored dot
    with a text label (name / mass / CoM position).

    Does NOT call plt.show() — the caller is responsible.
    Returns the Figure, or None if the component list is missing.
    """
    components_path = components_path or _COMPONENTS_PATH
    models_dir      = models_dir      or _MODELS_DIR

    if not os.path.exists(components_path):
        if verbose:
            print(f"[model_viewer] {components_path} not found — skipping model view.")
        return None

    with open(components_path) as f:
        components = json.load(f).get('components', [])
    if not components:
        return None

    fig = plt.figure('Vehicle Assembly', figsize=(10, 8))
    ax  = fig.add_subplot(111, projection='3d')
    ax.set_title('Vehicle Assembly — Component Layout', fontsize=13)

    all_points  = []
    dot_color_i = 0

    for comp in components:
        name     = comp.get('name', 'unnamed')
        pos      = np.array(comp.get('position_m', [0.0, 0.0, 0.0]), dtype=float)
        tmf_file = comp.get('file_3mf')
        stl_file = comp.get('stl_file')
        mass_g   = comp.get('mass_g')

        mesh = None
        if _TRIMESH:
            for fname in filter(None, [tmf_file, stl_file]):
                mesh = _load_mesh(
                    os.path.join(models_dir, fname),
                    comp.get('rotation_deg'),
                    pos,
                    verbose,
                )
                if mesh is not None:
                    break

        if mesh is not None:
            _draw_mesh(ax, mesh)
            all_points.append(mesh.vertices)
        else:
            color = _DOT_COLORS[dot_color_i % len(_DOT_COLORS)]
            dot_color_i += 1

            ax.scatter(*pos, color=color, s=90, zorder=5, depthshade=False)

            mass_str = f'{float(mass_g):.1f} g' if mass_g is not None else '? g'
            label = (f'{name}\n'
                     f'{mass_str}\n'
                     f'CoM  [{pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}] m')
            ax.text(pos[0], pos[1], pos[2], '  ' + label,
                    color=color, fontsize=7, zorder=6, va='bottom')
            all_points.append(pos.reshape(1, 3))

    if not _TRIMESH and any(c.get('file_3mf') or c.get('stl_file') for c in components):
        ax.set_title(
            'Vehicle Assembly  (install trimesh to render mesh components)', fontsize=11)

    if all_points:
        _equal_axes(ax, np.vstack(all_points))

    ax.set_xlabel('x  forward [m]')
    ax.set_ylabel('y  left [m]')
    ax.set_zlabel('z  up [m]')
    fig.tight_layout()
    return fig


# ---------------------------------------------------------------------------

def _load_mesh(path: str, rotation_deg, position_m: np.ndarray, verbose: bool):
    """Load a .3mf or .stl file, apply rotation and position, return Trimesh in metres."""
    if not os.path.exists(path):
        return None
    try:
        loaded = trimesh.load(path)
        if isinstance(loaded, trimesh.Scene):
            parts = list(loaded.geometry.values())
            if not parts:
                return None
            mesh = trimesh.util.concatenate(parts)
        elif isinstance(loaded, trimesh.Trimesh):
            mesh = loaded
        else:
            return None

        if rotation_deg and any(r != 0 for r in rotation_deg):
            rx, ry, rz = [np.radians(r) for r in rotation_deg]
            T = trimesh.transformations.euler_matrix(rx, ry, rz, axes='sxyz')
            mesh.apply_transform(T)

        mesh.apply_scale(1e-3)                # mm → m
        mesh.apply_translation(position_m)    # place mesh origin at body-frame position
        return mesh
    except Exception as e:
        if verbose:
            print(f"[model_viewer] Could not load {os.path.basename(path)}: {e}")
        return None


def _draw_mesh(ax, mesh):
    tris = mesh.vertices[mesh.faces]   # (F, 3, 3)
    if len(tris) > _MAX_TRIANGLES:
        step = max(1, len(tris) // _MAX_TRIANGLES)
        tris = tris[::step]
    poly = Poly3DCollection(tris, alpha=0.40, linewidth=0,
                            facecolor='#5b8ecc', edgecolor='none')
    ax.add_collection3d(poly)


def _equal_axes(ax, points: np.ndarray):
    lo = points.min(axis=0)
    hi = points.max(axis=0)
    c  = (lo + hi) / 2.0
    r  = max((hi - lo).max() / 2.0, 0.05)
    ax.set_xlim(c[0] - r, c[0] + r)
    ax.set_ylim(c[1] - r, c[1] + r)
    ax.set_zlim(c[2] - r, c[2] + r)
