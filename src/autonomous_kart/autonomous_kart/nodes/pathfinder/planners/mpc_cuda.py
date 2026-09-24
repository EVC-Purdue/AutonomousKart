# MPCPlanner with _solve on the GPU.
from __future__ import annotations

import ctypes
import os

import numpy as np

from autonomous_kart import paths
from autonomous_kart.nodes.pathfinder.planners.mpc import MPCPlanner

# Compile-time limits for cuda that would get overwritten.
NMAX = 64   # max horizon_steps
NK = 17     # steer-map knot count
MS = 42     # max proj_back + proj_fwd

# Kernel cost are literals. If updating in repo must be updated here.
_BAKED = {
    "w_d": 8.85, "w_heading": 2.97, "w_speed": 30.69, "w_delta": 0.05,
    "w_drate": 0.12, "w_accel": 2.0, "w_edge": 1456.58, "w_progress": 6.23,
    "w_term_h": 11.35, "w_a_lat": 0.0836, "a_lat_max": 5.3,
    "edge_inner": 0.273315,
}
_D = ctypes.POINTER(ctypes.c_double)


def lib_path() -> str:
    return os.environ.get(
        "MPC_CUDA_LIB", os.path.join(paths.ws_root(), "build", "mpc_cuda",
                                     "libmpc_cuda.so"))


def in_container() -> bool:
    return os.path.exists("/.dockerenv") or os.path.exists("/run/.containerenv")


def availability() -> tuple[bool, str]:
    """(usable, reason). Reason is for the log either way."""
    if in_container():
        return False, "container"
    so = lib_path()
    if not os.path.exists(so):
        return False, "no libmpc_cuda.so (run scripts/build_mpc_cuda.sh)"
    try:
        lib = ctypes.CDLL(so)
        lib.mpc_cuda_available.restype = ctypes.c_int
        if not lib.mpc_cuda_available():
            return False, "no usable CUDA device"
    except OSError as e:
        return False, f"cannot load libmpc_cuda.so ({e})"
    return True, "cuda fp32"


def describe_solver() -> str:
    ok, why = availability()
    return "cuda (fp32)" if ok else f"numpy ({why})"


def select_mpc_class():
    """CudaMPCPlanner when the GPU path is usable, else the numpy MPCPlanner."""
    ok, _ = availability()
    return CudaMPCPlanner if ok else MPCPlanner


def _d(a):
    """Read-only arg; data_as keeps the temporary alive for the call."""
    return np.ascontiguousarray(a, dtype=np.float64).ctypes.data_as(_D)


def _out(a):
    """Output buffer. Must already be contiguous float64 so the pointer is the
    real storage and not a copy nobody reads back."""
    assert a.dtype == np.float64 and a.flags["C_CONTIGUOUS"]
    return a.ctypes.data_as(_D)


class CudaMPCPlanner(MPCPlanner):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        if not self._map_cmd.size:
            raise RuntimeError("the kernel always applies the steer map; "
                               "this planner has none configured")
        if self._map_cmd.size != NK:
            raise RuntimeError(
                f"kernel was built for {NK} steer-map knots, yaml has "
                f"{self._map_cmd.size}; rebuild with NK={self._map_cmd.size}")
        if self.N > NMAX:
            raise RuntimeError(f"horizon_steps {self.N} exceeds kernel NMAX {NMAX}")
        if self.proj_back + self.proj_fwd > MS:
            raise RuntimeError(
                f"proj_back+proj_fwd {self.proj_back + self.proj_fwd} exceeds "
                f"kernel MS {MS}")
        self._check_baked()

        lib = ctypes.CDLL(lib_path())
        lib.mpc_cuda_available.restype = ctypes.c_int
        lib.mpc_cuda_init.restype = ctypes.c_void_p
        lib.mpc_cuda_init.argtypes = ([ctypes.c_int, ctypes.c_int, _D, _D,
                                       ctypes.c_int] + [ctypes.c_double] * 9)
        lib.mpc_cuda_solve.restype = ctypes.c_int
        lib.mpc_cuda_solve.argtypes = (
            [ctypes.c_void_p] + [ctypes.c_double] * 7 + [_D] * 5 +
            [ctypes.c_int, _D, _D, ctypes.c_ulonglong, ctypes.c_int, _D, _D, _D])
        lib.mpc_cuda_free.argtypes = [ctypes.c_void_p]
        if not lib.mpc_cuda_available():
            raise RuntimeError("no usable CUDA device")
        self._lib = lib
        self._ctx = lib.mpc_cuda_init(
            int(self.K), int(self.N), _d(self._map_cmd), _d(self._map_wheel),
            int(self._map_cmd.size), float(self.dt), float(self.wheelbase),
            float(self.steer_tau_s), float(self.steer_rate_max),
            float(self.steer_max), float(self.steer_sigma),
            float(self.accel_sigma), float(self.a_min), float(self.a_max))
        if not self._ctx:
            raise RuntimeError(f"mpc_cuda_init failed (K={self.K}, N={self.N})")
        self._u_buf = np.zeros(2 * NMAX, dtype=np.float64)
        self._best = ctypes.c_double()
        self._cmd0 = ctypes.c_double()
        self._tick = 0
        # Kernel returns no elite trajectory, so margin_min reads absent, not full.
        self._no_traj = np.full(self.N, np.nan)
        self._n_elite = max(1, int(round(self.mppi_elite_frac * self.K)))

    def _check_baked(self):
        got = {
            "w_d": self.w_d, "w_heading": self.w_heading,
            "w_speed": self.w_speed, "w_delta": self.w_delta,
            "w_drate": self.w_drate, "w_accel": self.w_accel,
            "w_edge": self.w_edge, "w_progress": self.w_progress,
            "w_term_h": self.w_term_h, "w_a_lat": self.w_a_lat,
            "a_lat_max": self.a_lat_max, "edge_inner": self.edge_inner,
        }
        bad = {k: (v, _BAKED[k]) for k, v in got.items()
               if abs(v - _BAKED[k]) > 1e-6 * max(1.0, abs(_BAKED[k]))}
        if bad:
            raise RuntimeError(
                "cost weights differ from the literals in the kernel, so it "
                "would score a different problem: "
                + ", ".join(f"{k} planner={v:.6g} kernel={w:.6g}"
                            for k, (v, w) in bad.items()))

    def _solve(self, x0, y0, yaw0, v0, j_now, v_target, v_cap):
        lo = max(0, j_now - self.proj_back)
        hi = min(self.line_n, j_now + self.proj_fwd)
        self._tick += 1
        rc = self._lib.mpc_cuda_solve(
            self._ctx, float(x0), float(y0), float(yaw0), float(v0),
            float(self.delta_prev), float(v_cap), float(v_target),
            _d(self.l_x[lo:hi]), _d(self.l_y[lo:hi]), _d(self.l_psi[lo:hi]),
            _d(self.l_s[lo:hi]), _d(self.l_vx[lo:hi]), int(hi - lo),
            _d(self.u_mean[0]), _d(self.u_mean[1]),
            ctypes.c_ulonglong(self._tick * 0x9E3779B97F4A7C15 & 0xFFFFFFFFFFFFFFFF),
            int(self._n_elite), _out(self._u_buf), ctypes.byref(self._best),
            ctypes.byref(self._cmd0))
        if rc != 0:
            raise RuntimeError(f"mpc_cuda_solve rc={rc}")
        u = np.stack([self._u_buf[:self.N], self._u_buf[NMAX:NMAX + self.N]])
        best_cost = float(self._best.value)
        self._cmd_out = float(self._cmd0.value)
        traj = {"d": self._no_traj, "s": self._no_traj}
        return (u, best_cost, traj,
                best_cost < self.feasibility_threshold, (0.0,) * 12)

    def __del__(self):
        try:
            self._lib.mpc_cuda_free(self._ctx)
        except Exception:
            pass
