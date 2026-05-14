"""
Shared test fixtures.

The production nodes declare their parameters via
`automatically_declare_parameters_from_overrides=True` and their
`__init__` methods take no args, so we can't pass `parameter_overrides`
at construction. Instead we seed the rclpy context with global
`--ros-args -p key:=value` overrides before creating nodes — this is
the same mechanism launch files use via `SetParameter`.
"""
import os
import sys
import textwrap
from contextlib import contextmanager

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
_PKG_ROOT = os.path.abspath(os.path.join(_HERE, ".."))
if _PKG_ROOT not in sys.path:
    sys.path.insert(0, _PKG_ROOT)


# s_m, x_m, y_m, psi_rad, kappa_radpm, vx_mps, ax_mps2
_TINY_LINE_CSV = textwrap.dedent(
    """\
    0.0,0.0,0.0,0.0,0.0,10.0,0.0
    1.0,1.0,0.0,0.0,0.0,10.0,0.0
    2.0,2.0,0.0,0.0,0.0,10.0,0.0
    3.0,3.0,0.0,0.0,0.0,10.0,0.0
    4.0,4.0,0.0,0.0,0.0,10.0,0.0
    5.0,5.0,0.0,0.0,0.0,10.0,0.0
    6.0,6.0,0.0,0.0,0.0,10.0,0.0
    7.0,7.0,0.0,0.0,0.0,10.0,0.0
    8.0,8.0,0.0,0.0,0.0,10.0,0.0
    9.0,9.0,0.0,0.0,0.0,10.0,0.0
    """
)


@pytest.fixture
def tiny_racing_line(tmp_path):
    path = tmp_path / "line.csv"
    path.write_text(_TINY_LINE_CSV)
    return str(path)


def _format_param_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, (int, float)):
        return repr(value)
    # Strings — wrap in double quotes so special chars in paths survive.
    return '"' + str(value).replace('"', '\\"') + '"'


def _build_ros_args(params):
    args = ["--ros-args"]
    for k, v in params.items():
        args += ["-p", f"{k}:={_format_param_value(v)}"]
    return args


@pytest.fixture
def ros_ctx():
    """
    Returns a context-manager factory: `with ros_ctx(params) as rclpy: ...`.
    Initializes rclpy with the given parameter overrides applied globally
    to every node constructed inside the block, and always shuts down.
    """
    rclpy = pytest.importorskip("rclpy")
    import rclpy.executors  # noqa: F401  (ensures submodule is importable)

    @contextmanager
    def _ctx(params=None):
        args = _build_ros_args(params or {})
        rclpy.init(args=args)
        try:
            yield rclpy
        finally:
            if rclpy.ok():
                rclpy.shutdown()

    return _ctx


def _spin_until(executor, predicate, timeout=3.0):
    import time

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        if predicate():
            return True
    return False


@pytest.fixture
def spin_helper():
    return _spin_until
