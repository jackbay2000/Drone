"""
Extracts PID gains from Arduino .ino / .cpp / .h files in Drone_Main/.

Two extraction strategies run in order:

  1. Direct name match — looks for variable names that exactly match the
     Drone_Main Gains struct fields (kp_roll, ki_z, …) and maps them
     straight to the DroneController gain dict using the same names.

  2. Fuzzy keyword match — fallback for gain keys that were not already
     resolved by the direct pass (e.g. kp_roll_rate → kp_roll).

Extraction handles:
  float kp_roll = 1.5;              (single declaration)
  float kp_roll = 3.5f;             (f-suffix literal)
  float kp_roll = 3.5f, ki_roll = 0.04f, kd_roll = 0.15f;  (comma-chain)
  #define KP_ROLL 1.5
  pid.setKp(1.5);  /  pid.Kp = 1.5;
"""

import re
import os
import glob

_CONTROLLER_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'Drone_Main')

# Direct name → DroneController gain key (identity: Drone_Main names are used as-is).
_DIRECT_MAP = {
    # Altitude
    'kp_z':     'kp_z',
    'ki_z':     'ki_z',
    'kd_z':     'kd_z',
    # Roll attitude
    'kp_roll':  'kp_roll',
    'ki_roll':  'ki_roll',
    'kd_roll':  'kd_roll',
    # Pitch attitude
    'kp_pitch': 'kp_pitch',
    'ki_pitch': 'ki_pitch',
    'kd_pitch': 'kd_pitch',
    # Yaw
    'kp_yaw':   'kp_yaw',
    'ki_yaw':   'ki_yaw',
    'kd_yaw':   'kd_yaw',
    # X / Y position
    'kp_x':     'kp_x',
    'ki_x':     'ki_x',
    'kd_x':     'kd_x',
    'kp_y':     'kp_y',
    'ki_y':     'ki_y',
    'kd_y':     'kd_y',
}

# Fuzzy fallback: (axis_keywords, term_keywords) → DroneController gain key.
# 'z' is intentionally absent from yaw axis keywords to prevent kp_z from
# false-matching yaw. 'x' is absent from roll to avoid kp_x (position) match.
_GAIN_MAP = [
    ({'alt', 'altitude', 'z', 'height', 'hover'}, {'p', 'kp'},       'kp_alt'),
    ({'alt', 'altitude', 'z', 'height', 'hover'}, {'i', 'ki'},       'ki_alt'),
    ({'alt', 'altitude', 'z', 'height', 'hover'}, {'d', 'kd'},       'kd_alt'),
    ({'roll', 'phi'},    {'rate', 'p', 'kp'},   'kp_rr'),
    ({'roll', 'phi'},    {'rate', 'i', 'ki'},   'ki_rr'),
    ({'roll', 'phi'},    {'rate', 'd', 'kd'},   'kd_rr'),
    ({'pitch', 'theta'}, {'rate', 'p', 'kp'},   'kp_pr'),
    ({'pitch', 'theta'}, {'rate', 'i', 'ki'},   'ki_pr'),
    ({'pitch', 'theta'}, {'rate', 'd', 'kd'},   'kd_pr'),
    ({'yaw', 'psi'},     {'rate', 'p', 'kp'},   'kp_yr'),
    ({'yaw', 'psi'},     {'rate', 'i', 'ki'},   'ki_yr'),
    ({'yaw', 'psi'},     {'rate', 'd', 'kd'},   'kd_yr'),
    ({'roll', 'phi'},    {'angle', 'outer', 'p', 'kp'}, 'kp_roll'),
    ({'pitch', 'theta'}, {'angle', 'outer', 'p', 'kp'}, 'kp_pitch'),
    ({'yaw', 'psi'},     {'angle', 'outer', 'p', 'kp'}, 'kp_yaw'),
]

# Regex patterns that extract (name, value) pairs.
#
# Pattern 1 — first variable in a typed declaration, with optional f/F suffix
#             and either comma or semicolon as terminator:
#               float kp_roll = 3.5f,   →  (kp_roll, 3.5)
#               double Kp = 1.5;        →  (Kp, 1.5)
#
# Pattern 2 — continuation variables in a comma-separated declaration.
#             Uses a lookahead so the trailing delimiter is NOT consumed,
#             allowing the next continuation variable to be found on the
#             same findall pass:
#               ,  ki_roll  = 0.04f,   →  (ki_roll, 0.04)
#               ,  kd_roll  = 0.15f;   →  (kd_roll, 0.15)
#
# Pattern 3 — preprocessor define:  #define KP_ROLL 1.5
#
# Pattern 4 — method / member assignment:  pid.setKp(1.5)  /  pid.Kp = 1.5
_PATTERNS = [
    re.compile(
        r'(?:float|double|int|const\s+\w+)\s+(\w+)\s*=\s*([\d.eE+-]+)[fF]?\s*[,;]',
        re.I),
    re.compile(
        r',\s*(\w+)\s*=\s*([\d.eE+-]+)[fF]?\s*(?=[,;])',
        re.I),
    re.compile(r'#define\s+(\w+)\s+([\d.eE+-]+)', re.I),
    re.compile(r'\.(?:set)?([KkPpIiDd]{1,2}[_\w]*)\s*[=(]\s*([\d.eE+-]+)', re.I),
]


def parse_gains(verbose=True) -> dict:
    """
    Scan all .ino, .cpp, and .h files in Drone_Main/ and return a dict
    of {gain_key: value} suitable for DroneController(gains=...).
    """
    files = (glob.glob(os.path.join(_CONTROLLER_DIR, '**', '*.ino'), recursive=True) +
             glob.glob(os.path.join(_CONTROLLER_DIR, '**', '*.cpp'), recursive=True) +
             glob.glob(os.path.join(_CONTROLLER_DIR, '**', '*.h'),   recursive=True))

    if not files:
        if verbose:
            print(f"[arduino_parser] No .ino/.cpp/.h files found in {_CONTROLLER_DIR}. "
                  "Using default PID gains.")
        return {}

    raw: dict[str, float] = {}
    for fpath in files:
        with open(fpath, encoding='utf-8', errors='ignore') as f:
            text = f.read()
        text = re.sub(r'//.*', '', text)   # strip line comments before matching
        for pattern in _PATTERNS:
            for name, val_str in pattern.findall(text):
                try:
                    raw[name.lower()] = float(val_str)
                except ValueError:
                    pass

    if verbose and raw:
        print(f"[arduino_parser] Found {len(raw)} numeric constants across {len(files)} file(s).")

    gains = {}

    # --- 1. Direct name match (Drone_Main Gains struct) -------------------
    for src_name, gain_key in _DIRECT_MAP.items():
        if src_name in raw:
            gains[gain_key] = raw[src_name]
            if verbose:
                print(f"[arduino_parser]   {gain_key} ← {src_name} = {raw[src_name]}")

    # --- 2. Fuzzy keyword match for anything not already resolved ---------
    already_mapped = set(gains)
    for axis_kw, term_kw, gain_key in _GAIN_MAP:
        if gain_key in already_mapped:
            continue
        best_name = _best_match(raw, axis_kw, term_kw)
        if best_name is not None:
            gains[gain_key] = raw[best_name]
            if verbose:
                print(f"[arduino_parser]   {gain_key} ← {best_name} = {raw[best_name]}  (fuzzy)")

    if not gains and verbose:
        print("[arduino_parser] Could not map any variables to PID gains. "
              "Check that your controller files use recognizable variable names.")

    return gains


def _best_match(raw: dict, axis_kw: set, term_kw: set) -> str | None:
    """
    Return the first raw variable name that contains at least one axis
    keyword and at least one term keyword as substrings.
    """
    for name in raw:
        if any(kw in name for kw in axis_kw) and any(kw in name for kw in term_kw):
            return name
    return None
