"""Workspace-relative path resolution.

The devcontainer mounts the repo at /ws; a native install has it wherever it
was cloned. Params name paths relative to the workspace root and resolve them
here, so one set of yamls is correct in both places. Absolute paths pass
through untouched, so a bag that recorded `/ws/data/...` still replays.
"""
import os
from functools import lru_cache

_MARKERS = ("src", "data")


@lru_cache(maxsize=1)
def ws_root() -> str:
    env = os.environ.get("KART_WS")
    if env and os.path.isdir(env):
        return env
    # Works from the source tree and from the colcon install tree alike.
    d = os.path.dirname(os.path.abspath(__file__))
    while True:
        if all(os.path.isdir(os.path.join(d, m)) for m in _MARKERS):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    return "/ws" if os.path.isdir("/ws") else os.getcwd()


def resolve(path: str) -> str:
    if not path or os.path.isabs(path):
        return path
    return os.path.join(ws_root(), path)
