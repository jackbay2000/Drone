"""
Extracts geometry and print settings from .3mf files.

A .3mf file is a ZIP archive. Different slicers store print settings in
different locations inside it:

  PrusaSlicer / SuperSlicer
    Metadata/Slic3r_PE.config          INI-like  fill_density = 15%

  BambuStudio / OrcaSlicer
    Metadata/project_settings.config   JSON      "fill_density": "15%"
    Metadata/model_settings.config     JSON      per-object overrides

  Cura
    3D/3dmodel.model                   XML       cura:fillDensity="15"
    (embedded as namespace attributes in the geometry file)

  Generic fallback
    Any metadata/*.config or *.ini file searched for fill_density patterns.

Effective density accounts for the fact that printed parts are not solid:
  effective_density = material_density × (wall_factor + (1-wall_factor) × infill/100)

wall_factor (~0.25) represents the solid perimeter shells that are always
present regardless of infill. Adjust per-component in component_list.json if
your parts have unusually thick or thin walls.
"""

import zipfile
import json
import re
import os

try:
    import trimesh
    _TRIMESH_AVAILABLE = True
except ImportError:
    _TRIMESH_AVAILABLE = False

import numpy as np

# Bulk material densities in kg/m³
MATERIAL_DENSITIES = {
    'PLA':      1240,
    'PETG':     1270,
    'ABS':      1040,
    'ASA':      1070,
    'TPU':      1210,
    'NYLON':    1150,
    'PA':       1150,
    'PC':       1200,
    'PETG-CF':  1400,
    'PLA-CF':   1300,
    'ABS-CF':   1200,
}

# Fraction of part volume that is solid shell regardless of infill.
# Typical value for 2-3 perimeter drone frames at 0.4mm line width.
DEFAULT_WALL_FACTOR = 0.25


def load_3mf(filepath: str, material: str = 'PLA',
             infill_override: float = None,
             wall_factor: float = DEFAULT_WALL_FACTOR,
             rotation_deg: list = None,
             position_m: list = None,
             verbose: bool = True):
    """
    Load a .3mf file and return (mass_kg, center_of_mass_m, inertia_3x3).
    Returns None if loading fails.

    Args:
        filepath:         Path to the .3mf file.
        material:         Fallback material if not detected in file.
        infill_override:  If set (0-100), use this instead of reading from file.
        wall_factor:      Fraction of part volume that is always solid shell (default 0.25).
        rotation_deg:     [rx, ry, rz] Euler angles (degrees, XYZ extrinsic) to rotate the
                          mesh from its CAD export orientation into the body frame.
                          Example: [0, 0, 90] rotates 90° about Z.
        position_m:       [x, y, z] position of the mesh ORIGIN in the body frame (meters).
                          The true CoM is auto-computed as position_m + R @ mesh_center.
                          Leave as [0,0,0] if the mesh origin is already at the body frame origin.
        verbose:          Print progress messages.
    """
    if not os.path.exists(filepath):
        if verbose:
            print(f"[3mf] File not found: {filepath}")
        return None

    if not _TRIMESH_AVAILABLE:
        if verbose:
            print("[3mf] trimesh not installed. Run: pip install trimesh\n"
                  "      Cannot load .3mf geometry.")
        return None

    # --- 1. Load geometry via trimesh ---------------------------------
    try:
        loaded = trimesh.load(filepath)
        if isinstance(loaded, trimesh.Scene):
            parts = list(loaded.geometry.values())
            if not parts:
                if verbose:
                    print(f"[3mf] No geometry found in {os.path.basename(filepath)}")
                return None
            mesh = trimesh.util.concatenate(parts)
        else:
            mesh = loaded
    except Exception as e:
        if verbose:
            print(f"[3mf] Geometry load failed for {os.path.basename(filepath)}: {e}")
        return None

    # --- 1b. Apply rotation to align mesh with body frame --------------
    if rotation_deg and any(r != 0 for r in rotation_deg):
        rx, ry, rz = [np.radians(r) for r in rotation_deg]
        T = trimesh.transformations.euler_matrix(rx, ry, rz, axes='sxyz')
        mesh.apply_transform(T)
        if verbose:
            print(f"[3mf]   Applied rotation {rotation_deg}°")

    # --- 2. Parse print settings from the ZIP -------------------------
    detected_infill  = None
    detected_material = None

    try:
        with zipfile.ZipFile(filepath, 'r') as zf:
            names = zf.namelist()
            detected_infill, detected_material = _parse_settings(zf, names, verbose)
    except Exception as e:
        if verbose:
            print(f"[3mf] Could not read settings from {os.path.basename(filepath)}: {e}")

    # --- 3. Resolve infill and material --------------------------------
    infill_pct = infill_override if infill_override is not None else detected_infill
    if infill_pct is None:
        infill_pct = 15.0  # conservative default
        if verbose:
            print(f"[3mf]   Infill not found in file; assuming {infill_pct:.0f}%. "
                  f"Set 'infill_percent' in component_list.json to override.")

    mat = (detected_material or material).upper()
    mat_density = MATERIAL_DENSITIES.get(mat, MATERIAL_DENSITIES['PLA'])
    if verbose and mat not in MATERIAL_DENSITIES:
        print(f"[3mf]   Unknown material '{mat}', using PLA density. "
              f"Add to MATERIAL_DENSITIES in tools/parse_3mf.py if needed.")

    # --- 4. Effective density ------------------------------------------
    # solid shells always present + infill fills the rest
    effective_fill = wall_factor + (1.0 - wall_factor) * (infill_pct / 100.0)
    effective_density = mat_density * effective_fill

    if verbose:
        print(f"[3mf]   {os.path.basename(filepath)}: material={mat}  "
              f"infill={infill_pct:.0f}%  "
              f"effective_density={effective_density:.0f} kg/m³  "
              f"(bulk={mat_density} kg/m³, wall_factor={wall_factor})")

    # --- 5. Mass properties -------------------------------------------
    mesh.density = effective_density
    # trimesh uses SI units if the model is in mm — convert volume to m³
    # trimesh reports volume in the model's native units (usually mm³ from CAD)
    volume_m3 = mesh.volume * 1e-9  # mm³ → m³
    mass_kg   = effective_density * volume_m3
    center_m  = mesh.center_mass * 1e-3  # mm → m (relative to mesh origin)

    # Apply position offset: mesh origin placed at position_m in the body frame
    pos_offset = np.array(position_m) if position_m else np.zeros(3)
    center_m = center_m + pos_offset

    # Inertia tensor: trimesh gives it in model units (mm²·kg equivalent)
    # moment_inertia is in [density units × length^4], need kg·m²
    # Use parallel-axis-free inertia about CoM
    inertia_kgm2 = mesh.moment_inertia * effective_density * 1e-12  # mm^4 * kg/mm^3 → kg·m^2...
    # Actually trimesh.moment_inertia for a unit-density mesh is in mm^5 (volume × length²)
    # With density in kg/m³ and volume in mm³ we need to be careful.
    # Safest: compute manually from mesh primitives.
    inertia_kgm2 = _inertia_tensor(mesh, effective_density)

    if verbose:
        print(f"[3mf]   mass={mass_kg*1000:.1f}g  CoM={center_m}  "
              f"Ixx={inertia_kgm2[0,0]*1e6:.2f}×10⁻⁶ kg·m²")

    return mass_kg, center_m, inertia_kgm2


def _inertia_tensor(mesh: 'trimesh.Trimesh', density_kg_m3: float) -> np.ndarray:
    """Compute inertia tensor in kg·m² from a trimesh object (coords in mm)."""
    # trimesh.moment_inertia is computed assuming density=1 in mesh units.
    # Mesh is in mm, density is in kg/m³.
    # Conversion: 1 mm = 1e-3 m, so 1 mm^5 = 1e-15 m^5
    # Inertia = density [kg/m³] × moment [m^5] = density × mesh_moment [mm^5] × 1e-15
    raw = mesh.moment_inertia  # shape (3,3), units: mm^5 (when density=1 kg/mm³)
    # mesh.moment_inertia with density=1 → units are mm^5
    # To get kg·m²: multiply by (density_kg_m3 / 1e9) × 1e-6
    #   density_kg_m3 / 1e9 → kg/mm³
    #   × 1e-6 → mm^5 × kg/mm^3 × (1e-3 m/mm)^2 = kg·mm^2 × 1e-6 → kg·m²?
    # Let's do it cleanly:
    #   I [kg·m²] = density [kg/m³] × moment_volume [m^5]
    #   moment_volume [m^5] = raw [mm^5] × (1e-3)^5 = raw × 1e-15
    #   I = density × raw × 1e-15
    return density_kg_m3 * raw * 1e-15


# ------------------------------------------------------------------
def _parse_settings(zf: zipfile.ZipFile, names: list, verbose: bool):
    """Try each slicer's config location. Returns (infill_pct, material_str)."""

    # ---- PrusaSlicer / SuperSlicer (INI-like config) -----------------
    prusa_files = [n for n in names if re.search(
        r'(slic3r_pe|prusaslicer|superslicer|prusa_slicer).*\.(config|ini)$', n, re.I)]
    for fname in prusa_files:
        with zf.open(fname) as f:
            text = f.read().decode('utf-8', errors='ignore')
        infill, mat = _parse_ini_style(text, verbose, source='PrusaSlicer')
        if infill is not None:
            return infill, mat

    # ---- BambuStudio / OrcaSlicer (JSON config) ----------------------
    bambu_files = [n for n in names if re.search(
        r'(project_settings|model_settings|bambu).*\.config$', n, re.I)]
    for fname in bambu_files:
        with zf.open(fname) as f:
            text = f.read().decode('utf-8', errors='ignore')
        infill, mat = _parse_json_style(text, verbose, source='BambuStudio/OrcaSlicer')
        if infill is not None:
            return infill, mat

    # ---- Cura (settings embedded in 3dmodel.model XML) ---------------
    model_files = [n for n in names if n.endswith('3dmodel.model')]
    for fname in model_files:
        with zf.open(fname) as f:
            text = f.read().decode('utf-8', errors='ignore')
        infill, mat = _parse_cura_xml(text, verbose)
        if infill is not None:
            return infill, mat

    # ---- Generic fallback: search all metadata text files ------------
    meta_files = [n for n in names if
                  re.search(r'metadata|config|settings|slic', n, re.I)
                  and not n.endswith(('.png', '.gcode', '.pdf'))]
    for fname in meta_files:
        if fname in (prusa_files + bambu_files + model_files):
            continue
        try:
            with zf.open(fname) as f:
                text = f.read().decode('utf-8', errors='ignore')
            infill = _extract_infill_generic(text)
            if infill is not None:
                if verbose:
                    print(f"[3mf]   Found infill={infill:.0f}% in {fname} (generic parse)")
                return infill, None
        except Exception:
            pass

    return None, None


def _parse_ini_style(text: str, verbose: bool, source: str):
    """Parse PrusaSlicer INI-like config."""
    infill = _extract_infill_generic(text)
    mat = None
    m = re.search(r'filament_type\s*=\s*([A-Za-z0-9_-]+)', text)
    if m:
        mat = m.group(1).strip()
    if infill is not None and verbose:
        print(f"[3mf]   {source}: fill_density={infill:.0f}%  material={mat}")
    return infill, mat


def _parse_json_style(text: str, verbose: bool, source: str):
    """Parse BambuStudio/OrcaSlicer JSON config."""
    # Config may be JSON or INI-like depending on version
    infill = None
    mat = None
    try:
        data = json.loads(text)
        # BambuStudio uses "sparse_infill_density" or "fill_density"
        for key in ('sparse_infill_density', 'fill_density', 'infill_density'):
            val = data.get(key)
            if val is not None:
                infill = float(str(val).strip().rstrip('%'))
                break
        for key in ('filament_type', 'material_type', 'filament'):
            val = data.get(key)
            if val is not None:
                mat = str(val).strip().strip('"')
                break
    except (json.JSONDecodeError, ValueError):
        # Try INI-style anyway
        infill = _extract_infill_generic(text)
        m = re.search(r'filament_type\s*[=:]\s*"?([A-Za-z0-9_-]+)"?', text)
        if m:
            mat = m.group(1)

    if infill is not None and verbose:
        print(f"[3mf]   {source}: fill_density={infill:.0f}%  material={mat}")
    return infill, mat


def _parse_cura_xml(text: str, verbose: bool):
    """Extract infill from Cura's namespace attributes in 3dmodel.model."""
    infill = None
    mat = None
    m = re.search(r'(?:cura:)?fill[_\-]?[Dd]ensity["\s]*[=:]\s*"?([\d.]+)', text)
    if m:
        infill = float(m.group(1))
    m2 = re.search(r'(?:cura:)?material["\s]*[=:]\s*"?([A-Za-z0-9_-]+)', text)
    if m2:
        mat = m2.group(1)
    if infill is not None and verbose:
        print(f"[3mf]   Cura: fill_density={infill:.0f}%  material={mat}")
    return infill, mat


def _extract_infill_generic(text: str):
    """Regex search for any fill/infill density value in arbitrary text."""
    patterns = [
        r'(?:fill_density|sparse_infill_density|infill_density|infillDensity)\s*[=:]\s*"?([\d.]+)%?"?',
        r'"infill"\s*:\s*"?([\d.]+)%?"?',
    ]
    for pat in patterns:
        m = re.search(pat, text, re.I)
        if m:
            return float(m.group(1))
    return None
