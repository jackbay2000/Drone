"""
Loads the compiled Drone_Main controller as a shared library and exposes it
as a DroneController class with the same interface as the Python fallback.

Auto-build: on every import, source mtimes are compared against the DLL.  If
any Drone_Main .cpp/.h or cpp_bridge file is newer than the DLL, a rebuild
is triggered automatically using whatever C++ compiler is on PATH
(cl.exe / MSVC, or g++ / MinGW).

If no compiler is found or the build fails, an ImportError is raised so that
pid_controller.py can fall back to its pure-Python translation.
"""

import ctypes
import os
import platform
import subprocess
import sys
import glob
import shutil

import numpy as np
from controller.base_controller import BaseController

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
_THIS_DIR   = os.path.dirname(os.path.abspath(__file__))
_SIM_DIR    = os.path.dirname(_THIS_DIR)
_BRIDGE_DIR = os.path.join(_SIM_DIR, 'cpp_bridge')
_DM_DIR     = os.path.join(os.path.dirname(_SIM_DIR), 'Drone_Main')

_DLL_NAME = 'sim_bridge.dll' if os.name == 'nt' else 'sim_bridge.so'
_DLL_PATH = os.path.join(_BRIDGE_DIR, _DLL_NAME)

# Drone_Main source files that are compiled into the bridge
_DM_SOURCES = [
    'PID.cpp',
    'AttitudeController.cpp',
    'YawController.cpp',
    'PositionPID.cpp',
    'controller.cpp',
]
# Their headers, plus main.h (pulled in transitively by controller.h). Staged
# into cpp_bridge/ alongside the stub headers before every build -- see the
# comment in build() for why this has to be a real copy, not just another -I
# path: quoted #includes ("IMU.h") resolve relative to the including file's
# OWN directory first, before any -I search path is even considered, so
# main.h (which lives in Drone_Main/) would always find the real, hardware-
# dependent Drone_Main/IMU.h instead of cpp_bridge/IMU.h's stub no matter
# what order -I flags are given in.
_DM_HEADERS = [
    'PID.h', 'AttitudeController.h', 'YawController.h', 'PositionPID.h',
    'controller.h', 'main.h',
]


# ---------------------------------------------------------------------------
# Build helpers
# ---------------------------------------------------------------------------

def _source_mtime() -> float:
    """Return the most recent mtime across all bridge and Drone_Main sources."""
    patterns = [
        os.path.join(_BRIDGE_DIR, '*.cpp'),
        os.path.join(_BRIDGE_DIR, '*.h'),
        os.path.join(_DM_DIR, '*.cpp'),
        os.path.join(_DM_DIR, '*.h'),
    ]
    files = []
    for p in patterns:
        files.extend(glob.glob(p))
    return max((os.path.getmtime(f) for f in files), default=0.0)


def _dll_mtime() -> float:
    return os.path.getmtime(_DLL_PATH) if os.path.exists(_DLL_PATH) else 0.0


def _find_compiler():
    import shutil
    if shutil.which('cl'):
        return 'msvc'
    if shutil.which('g++'):
        return 'gcc'
    if shutil.which('g++.exe'):
        return 'gcc'
    return None


def build(force: bool = False, verbose: bool = True) -> bool:
    """
    Compile the bridge DLL.  Returns True on success.
    Raises RuntimeError with the compiler output if compilation fails.
    """
    compiler = _find_compiler()
    if compiler is None:
        raise RuntimeError(
            "[drone_bridge] No C++ compiler found on PATH.\n"
            "  Install one of:\n"
            "    - Visual Studio Build Tools (adds cl.exe)\n"
            "    - MSYS2 / MinGW-w64          (adds g++)\n"
            "  Then re-run the simulation."
        )

    # Stage the pure-logic Drone_Main files directly into cpp_bridge/,
    # alongside the stub headers, with copy2 (preserves source mtime, so
    # _source_mtime() staleness checks still work correctly against the
    # ORIGINAL Drone_Main files). This makes every quoted #include inside
    # main.h/controller.h resolve to the stubs by construction, since they
    # now physically share a directory with them -- see _DM_HEADERS above.
    # These are build artifacts, not a second source of truth: never edit
    # a *.cpp/*.h file inside cpp_bridge/ that also exists in Drone_Main/,
    # edit Drone_Main/ and rebuild.
    for fname in _DM_SOURCES + _DM_HEADERS:
        shutil.copy2(os.path.join(_DM_DIR, fname), os.path.join(_BRIDGE_DIR, fname))

    sources = [os.path.join(_BRIDGE_DIR, s) for s in _DM_SOURCES]
    sources.append(os.path.join(_BRIDGE_DIR, 'sim_bridge.cpp'))

    if compiler == 'msvc':
        # Intermediate .obj files go into cpp_bridge to keep things tidy
        obj_dir = _BRIDGE_DIR
        cmd = [
            'cl', '/nologo', '/EHsc', '/O2', '/std:c++17',
            f'/I{_BRIDGE_DIR}',
            f'/Fo{obj_dir}{os.sep}',
            '/LD', f'/Fe:{_DLL_PATH}',
            *sources,
        ]
    else:
        cmd = [
            'g++', '-shared', '-O2', '-std=c++17',
            # Statically link the MinGW runtime (libgcc/libstdc++/libwinpthread)
            # so the DLL has no external dependency on the compiler's own bin/
            # directory being on PATH at load time -- otherwise ctypes.CDLL()
            # fails with a misleading "could not find module" (it's actually a
            # missing *dependency*, not a missing sim_bridge.dll itself).
            '-static-libgcc', '-static-libstdc++', '-static',
            f'-I{_BRIDGE_DIR}',
            *sources,
            '-o', _DLL_PATH,
        ]
        if platform.system() != 'Windows':
            cmd.insert(1, '-fPIC')

    if verbose:
        print(f"[drone_bridge] Building {_DLL_NAME} with {compiler}...")

    result = subprocess.run(cmd, capture_output=True, text=True,
                            cwd=_BRIDGE_DIR)

    if result.returncode != 0:
        raise RuntimeError(
            f"[drone_bridge] Compilation failed (exit {result.returncode}):\n"
            + result.stdout + result.stderr
        )

    if verbose:
        print(f"[drone_bridge] Build succeeded -> {_DLL_PATH}")
    return True


def _ensure_dll(verbose: bool = True):
    """Build the DLL if it is missing or any source file is newer than it."""
    if _source_mtime() > _dll_mtime():
        build(verbose=verbose)


# ---------------------------------------------------------------------------
# Library loader
# ---------------------------------------------------------------------------
_lib_cache = None


def _get_lib(verbose: bool = True) -> ctypes.CDLL:
    global _lib_cache
    if _lib_cache is not None:
        return _lib_cache

    _ensure_dll(verbose=verbose)

    if os.name == 'nt' and hasattr(os, 'add_dll_directory'):
        os.add_dll_directory(_BRIDGE_DIR)

    lib = ctypes.CDLL(_DLL_PATH)

    # --- function signatures ------------------------------------------------
    lib.sim_init.restype  = None
    lib.sim_init.argtypes = []

    lib.sim_set_gains.restype  = None
    lib.sim_set_gains.argtypes = [ctypes.c_float] * 18

    lib.sim_add_waypoint.restype  = None
    lib.sim_add_waypoint.argtypes = [ctypes.c_float, ctypes.c_float, ctypes.c_float, ctypes.c_int]

    lib.sim_clear_waypoints.restype  = None
    lib.sim_clear_waypoints.argtypes = []

    lib.sim_mission_complete.restype  = ctypes.c_int
    lib.sim_mission_complete.argtypes = []

    lib.sim_current_waypoint_index.restype  = ctypes.c_int
    lib.sim_current_waypoint_index.argtypes = []

    lib.sim_set_state.restype  = None
    lib.sim_set_state.argtypes = [ctypes.c_float] * 6 + [ctypes.c_int]

    lib.sim_update.restype  = None
    lib.sim_update.argtypes = [ctypes.c_ulong,
                               ctypes.POINTER(ctypes.c_float)]

    _lib_cache = lib
    return lib


# ---------------------------------------------------------------------------
# DroneController — calls the real Drone_Main C++ control loop each step
# ---------------------------------------------------------------------------

class DroneController(BaseController):
    """
    Wraps the compiled Drone_Main Controller via ctypes.

    The sim loop calls update(state, dt); state is split into sensor fields,
    the clock is advanced, and ESC outputs are read back as [0,1] commands.

    add_waypoint / clear_waypoints / mission_complete are implemented in
    cpp_bridge/sim_bridge.cpp, mirroring Drone_Main.ino's own waypoint
    sequencing loop (Controller itself only flies toward a single target).
    """

    def __init__(self, engine, gains: dict = None):
        self._engine    = engine
        self._lib       = _get_lib()
        self._gains     = gains or {}
        self._waypoints: list[tuple[float, float, float, bool]] = []
        self._motor_buf = (ctypes.c_float * 4)()
        self._altitude_stale = False
        self._setup_dll()

    # ------------------------------------------------------------------
    def _setup_dll(self):
        """(Re-)initialise the DLL with stored gains and waypoints."""
        self._lib.sim_init()
        g = self._gains
        self._lib.sim_set_gains(
            g.get('kp_roll',  3.5),   g.get('ki_roll',  0.04),  g.get('kd_roll',  0.15),
            g.get('kp_pitch', 3.5),   g.get('ki_pitch', 0.04),  g.get('kd_pitch', 0.15),
            g.get('kp_yaw',   2.0),   g.get('ki_yaw',   0.02),  g.get('kd_yaw',   0.08),
            g.get('kp_x',     0.25),  g.get('ki_x',     0.005), g.get('kd_x',     0.15),
            g.get('kp_y',     0.25),  g.get('ki_y',     0.005), g.get('kd_y',     0.15),
            g.get('kp_z',     0.40),  g.get('ki_z',     0.02),  g.get('kd_z',     0.20),
        )
        for wx, wy, wz, wh in self._waypoints:
            self._lib.sim_add_waypoint(
                ctypes.c_float(wx), ctypes.c_float(wy), ctypes.c_float(wz),
                ctypes.c_int(1 if wh else 0))

    # ------------------------------------------------------------------
    def add_waypoint(self, x: float, y: float, z: float, keep_heading: bool = True):
        self._waypoints.append((float(x), float(y), float(z), bool(keep_heading)))
        self._lib.sim_add_waypoint(
            ctypes.c_float(x), ctypes.c_float(y), ctypes.c_float(z),
            ctypes.c_int(1 if keep_heading else 0))

    def clear_waypoints(self):
        self._waypoints.clear()
        self._lib.sim_clear_waypoints()

    @property
    def mission_complete(self) -> bool:
        return bool(self._lib.sim_mission_complete())

    @property
    def current_waypoint_index(self) -> int:
        return int(self._lib.sim_current_waypoint_index())

    def set_altitude_stale(self, stale: bool):
        """Mirrors Position::altitudeStale() -- called by ControlLoopTiming
        each tick with the Python-side estimator's own staleness read, since
        this DLL's stub Position class has no real sensors of its own to
        judge that from (see cpp_bridge/Position.h)."""
        self._altitude_stale = bool(stale)

    # ------------------------------------------------------------------
    def update(self, state: np.ndarray, dt: float) -> np.ndarray:
        phi, theta, psi = float(state[6]), float(state[7]), float(state[8])
        x,   y,   z    = float(state[0]), float(state[1]), float(state[2])

        self._lib.sim_set_state(
            ctypes.c_float(phi),   ctypes.c_float(theta), ctypes.c_float(psi),
            ctypes.c_float(x),     ctypes.c_float(y),     ctypes.c_float(z),
            ctypes.c_int(1 if self._altitude_stale else 0),
        )

        dt_us = ctypes.c_ulong(max(1, int(dt * 1e6)))
        self._lib.sim_update(dt_us, self._motor_buf)

        return np.array(list(self._motor_buf), dtype=np.float64)

    def reset(self):
        self._setup_dll()
