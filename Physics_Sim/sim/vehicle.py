"""
Assembles vehicle physical parameters from available inputs.

Priority order:
  1. 3MF files (geometry + print settings)  -- via inputs/components/component_list.json
  2. STL files + manual density
  3. config/vehicle_defaults.json

The motor layout assumes an X-configuration quadcopter.
Body frame convention: +x=forward, +y=left, +z=up (right-handed).

Motor indices and positions (d = arm_length / sqrt(2)):
  0: front-left   (+d, +d, 0)  CCW  yaw_sign=+1
  1: front-right  (+d, -d, 0)  CW   yaw_sign=-1
  2: rear-right   (-d, -d, 0)  CCW  yaw_sign=+1
  3: rear-left    (-d, +d, 0)  CW   yaw_sign=-1
"""

import numpy as np
import json
import os

_DEFAULTS_PATH = os.path.join(os.path.dirname(__file__), '..', 'config', 'vehicle_defaults.json')
_COMPONENTS_PATH = os.path.join(os.path.dirname(__file__), '..', 'inputs', 'components', 'component_list.json')
_STL_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'Models')
_3MF_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'Models')  # adjacent Models/ folder; accepts both


class VehicleParams:
    def __init__(self):
        self.mass_kg: float = 0.0
        self.inertia: np.ndarray = np.zeros((3, 3))
        self.arm_length_m: float = 0.0
        self.center_of_mass_m: np.ndarray = np.zeros(3)
        self.motor_positions: np.ndarray = np.zeros((4, 3))
        self.motor_yaw_signs: np.ndarray = np.array([1, -1, 1, -1], dtype=float)
        self.drag_linear: float = 0.1
        self.drag_rotational: float = 0.02
        self.source: str = "unknown"

    def mixer_matrix(self, torque_ratio: float) -> np.ndarray:
        """
        Returns 4x4 matrix A such that [F, τx, τy, τz]^T = A @ [T0,T1,T2,T3]^T
        """
        pos = self.motor_positions
        A = np.array([
            [1,       1,       1,       1      ],
            [pos[0,1], pos[1,1], pos[2,1], pos[3,1]],
            [-pos[0,0],-pos[1,0],-pos[2,0],-pos[3,0]],
            [torque_ratio * self.motor_yaw_signs[0],
             torque_ratio * self.motor_yaw_signs[1],
             torque_ratio * self.motor_yaw_signs[2],
             torque_ratio * self.motor_yaw_signs[3]],
        ])
        return A


def load_vehicle(verbose=True) -> VehicleParams:
    """Load vehicle parameters, preferring measured data over defaults."""
    params = VehicleParams()

    components_loaded = _try_load_components(params, verbose)
    if not components_loaded:
        _load_defaults(params, verbose)

    if not _try_real_motor_positions(params, verbose):
        _build_motor_positions(params)
    return params


def _try_real_motor_positions(params: VehicleParams, verbose: bool) -> bool:
    """
    Use the actual per-motor (x, y) positions from component_list.json when
    all four are present, instead of _build_motor_positions()'s idealized
    symmetric square. Diagnosed 2026-07-24: this vehicle's real frame is NOT
    symmetric (front-back motor offset 115.1mm vs left-right 103.1mm, ~12%
    apart) -- _build_motor_positions collapses both to a single averaged
    Euclidean distance, silently making roll and pitch torque arms identical
    in the sim when they aren't on the real vehicle. Z is intentionally
    ignored here (set to 0), same simplification _build_motor_positions
    already made -- the mixer_matrix derivation assumes coplanar motors.
    """
    if not os.path.exists(_COMPONENTS_PATH):
        return False
    with open(_COMPONENTS_PATH) as f:
        components = json.load(f).get('components', [])

    by_name = {c['name']: c for c in components}
    order = ['motor_fl', 'motor_fr', 'motor_rr', 'motor_rl']  # sim order
    if not all(name in by_name and by_name[name].get('position_m') for name in order):
        return False

    params.motor_positions = np.array([
        [by_name[name]['position_m'][0], by_name[name]['position_m'][1], 0.0]
        for name in order
    ])
    if verbose:
        print(f"[vehicle]   Using real per-motor positions from component_list.json "
              f"(front-back arm {abs(params.motor_positions[0,0])*1000:.1f}mm, "
              f"left-right arm {abs(params.motor_positions[0,1])*1000:.1f}mm)")
    return True


def _build_motor_positions(params: VehicleParams):
    d = params.arm_length_m / np.sqrt(2)
    params.motor_positions = np.array([
        [ d,  d, 0.0],   # 0: front-left  CCW
        [ d, -d, 0.0],   # 1: front-right CW
        [-d, -d, 0.0],   # 2: rear-right  CCW
        [-d,  d, 0.0],   # 3: rear-left   CW
    ])


def _load_defaults(params: VehicleParams, verbose: bool):
    with open(_DEFAULTS_PATH) as f:
        cfg = json.load(f)

    params.mass_kg = cfg['mass_kg']
    params.arm_length_m = cfg['arm_length_m']
    I = cfg['inertia_kgm2']
    params.inertia = np.diag([I['Ixx'], I['Iyy'], I['Izz']])
    params.center_of_mass_m = np.array(cfg['center_of_mass_m'])
    params.drag_linear = cfg['drag_coefficients']['linear']
    params.drag_rotational = cfg['drag_coefficients']['rotational']
    params.source = "defaults"
    if verbose:
        print(f"[vehicle] Using default parameters (mass={params.mass_kg*1000:.0f}g). "
              f"Drop STL files and fill component_list.json to improve accuracy.")


def _try_load_components(params: VehicleParams, verbose: bool) -> bool:
    if not os.path.exists(_COMPONENTS_PATH):
        return False

    with open(_COMPONENTS_PATH) as f:
        cfg = json.load(f)

    components = cfg.get('components', [])
    if not components:
        return False

    total_mass = 0.0
    weighted_pos = np.zeros(3)

    mesh_inertias = []  # collect (mass_kg, center_m, inertia_3x3) from 3mf parts

    for comp in components:
        mass_kg = comp.get('mass_g', 0) / 1000.0 if comp.get('mass_g') is not None else 0.0
        pos = np.array(comp['position_m']) if comp.get('position_m') is not None else np.zeros(3)
        loaded_geometry = None

        # Prefer .3mf over .stl (richer info)
        tmf_file = comp.get('file_3mf')
        stl_file = comp.get('stl_file')

        if tmf_file:
            from tools.parse_3mf import load_3mf
            tmf_path = os.path.join(_3MF_DIR, tmf_file)
            result = load_3mf(
                tmf_path,
                material=comp.get('material', 'PLA'),
                infill_override=comp.get('infill_percent'),
                wall_factor=comp.get('wall_factor', 0.25),
                rotation_deg=comp.get('rotation_deg', [0, 0, 0]),
                position_m=comp.get('position_m', [0, 0, 0]),
                verbose=verbose,
            )
            if result is not None:
                loaded_geometry = result  # (mass_kg, center_m, inertia_3x3)

        if loaded_geometry is None and stl_file:
            stl_path = os.path.join(_STL_DIR, stl_file)
            stl_result = _try_load_stl(stl_path, comp.get('material_density_kg_m3', 1250), verbose)
            if stl_result is not None:
                mass_kg, pos = stl_result

        if loaded_geometry is not None:
            lm, lc, li = loaded_geometry
            mass_kg = lm
            pos = lc
            mesh_inertias.append((lm, lc, li))

        total_mass += mass_kg
        weighted_pos += mass_kg * pos

    if total_mass <= 0:
        return False

    params.mass_kg = total_mass
    params.center_of_mass_m = weighted_pos / total_mass
    params.source = "component_list"

    # Inertia: use mesh tensors where available, point masses elsewhere
    I = np.zeros((3, 3))
    mesh_names = {id(r): True for r in mesh_inertias}

    # Add mesh inertias (parallel-axis theorem to shift to composite CoM)
    for lm, lc, li in mesh_inertias:
        r = lc - params.center_of_mass_m
        I += li + lm * (np.dot(r, r) * np.eye(3) - np.outer(r, r))

    # Add point-mass inertias for components without geometry
    loaded_geo_idx = set(range(len(mesh_inertias)))
    geo_i = 0
    for comp in components:
        if comp.get('file_3mf') and geo_i < len(mesh_inertias):
            geo_i += 1
            continue  # already added via mesh
        m = (comp.get('mass_g') or 0) / 1000.0
        if m <= 0 or comp.get('position_m') is None:
            continue
        r = np.array(comp['position_m']) - params.center_of_mass_m
        I += m * (np.dot(r, r) * np.eye(3) - np.outer(r, r))
    # Guard against degenerate inertia (all point masses at origin)
    if np.allclose(I, 0):
        with open(_DEFAULTS_PATH) as f:
            d = json.load(f)['inertia_kgm2']
        I = np.diag([d['Ixx'], d['Iyy'], d['Izz']])
    params.inertia = I

    # Arm length: distance from CoM to average motor position (approximation)
    motor_comps = [c for c in components if c['name'].startswith('motor')]
    if motor_comps:
        avg_dist = np.mean([np.linalg.norm(np.array(c['position_m']) - params.center_of_mass_m)
                            for c in motor_comps])
        params.arm_length_m = avg_dist
    else:
        with open(_DEFAULTS_PATH) as f:
            params.arm_length_m = json.load(f)['arm_length_m']

    # Fall back to default drag coefficients
    with open(_DEFAULTS_PATH) as f:
        d = json.load(f)['drag_coefficients']
    params.drag_linear = d['linear']
    params.drag_rotational = d['rotational']

    if verbose:
        print(f"[vehicle] Loaded from component_list.json  mass={params.mass_kg*1000:.1f}g  "
              f"CoM={params.center_of_mass_m}  arm={params.arm_length_m*1000:.1f}mm")
    return True


def _try_load_stl(path: str, density: float, verbose: bool):
    """Returns (mass_kg, center_m) from STL or None if unavailable."""
    if not os.path.exists(path):
        return None
    try:
        import trimesh
        mesh = trimesh.load(path)
        mesh.density = density
        mass_kg = float(mesh.mass) / 1000.0  # trimesh returns grams
        center = np.array(mesh.center_mass)
        if verbose:
            print(f"[vehicle]   STL {os.path.basename(path)}: mass={mass_kg*1000:.1f}g  CoM={center}")
        return mass_kg, center
    except ImportError:
        if verbose:
            print(f"[vehicle]   trimesh not installed; skipping {os.path.basename(path)}. "
                  f"Run: pip install trimesh")
        return None
    except Exception as e:
        if verbose:
            print(f"[vehicle]   Failed to load {os.path.basename(path)}: {e}")
        return None
