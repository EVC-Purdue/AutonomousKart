"""GIF render of a SimResult on the racing-line track. matplotlib-based."""
from __future__ import annotations

import os
import sys
from typing import Optional, Sequence

import numpy as np


HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
if REPO not in sys.path:
    sys.path.insert(0, REPO)


def load_line_xy(path: str) -> tuple[np.ndarray, np.ndarray]:
    """Load (lx, ly) from a racing-line CSV (cols x at idx 1, y at idx 2)."""
    lx, ly = [], []
    with open(path) as f:
        for row in f:
            parts = row.strip().split(",")
            if len(parts) < 3:
                continue
            try:
                lx.append(float(parts[1]))
                ly.append(float(parts[2]))
            except ValueError:
                continue
    return np.asarray(lx, dtype=np.float64), np.asarray(ly, dtype=np.float64)


def _tangent_normal(lx: np.ndarray, ly: np.ndarray):
    dx = np.empty_like(lx)
    dy = np.empty_like(ly)
    dx[1:-1] = lx[2:] - lx[:-2]
    dy[1:-1] = ly[2:] - ly[:-2]
    dx[0] = lx[1] - lx[0]
    dy[0] = ly[1] - ly[0]
    dx[-1] = lx[-1] - lx[-2]
    dy[-1] = ly[-1] - ly[-2]
    L = np.hypot(dx, dy)
    L = np.where(L < 1e-9, 1.0, L)
    tx, ty = dx / L, dy / L
    return -ty, tx  # left-hand normal


def render_gif(
    xs: np.ndarray,
    ys: np.ndarray,
    yaws: np.ndarray,
    vs: np.ndarray,
    ds: np.ndarray,
    times: np.ndarray,
    line_path: str,
    out_path: str,
    *,
    mpc_params: Optional[dict] = None,
    track_half_width: float = 2.0,
    kart_length_m: float = 1.05,
    kart_width_m: float = 0.5,
    fps: int = 30,
    max_frames: int = 300,
    title: str = "Kart sim",
) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.patches as mpatches
    from matplotlib.animation import FuncAnimation, PillowWriter

    lx, ly = load_line_xy(line_path)
    nx, ny = _tangent_normal(lx, ly)
    left_x = lx + nx * track_half_width
    left_y = ly + ny * track_half_width
    right_x = lx - nx * track_half_width
    right_y = ly - ny * track_half_width

    # Decimate frames
    N = len(xs)
    stride = max(1, N // max_frames)
    idxs = np.arange(0, N, stride)[:max_frames]
    n_frames = len(idxs)

    # Fixed view limits
    all_x = np.concatenate([xs, lx])
    all_y = np.concatenate([ys, ly])
    margin = 5.0
    xlim = (float(all_x.min()) - margin, float(all_x.max()) + margin)
    ylim = (float(all_y.min()) - margin, float(all_y.max()) + margin)

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.set_aspect("equal")
    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_title(title, fontsize=9)
    ax.axis("off")

    # Static background
    ax.plot(left_x, left_y, color="#444", lw=1.0)
    ax.plot(right_x, right_y, color="#444", lw=1.0)
    ax.plot(lx, ly, color="#999", lw=0.5, ls="--")

    trail_line, = ax.plot([], [], color="#0a7", alpha=0.5, lw=1.5)

    # Kart rectangle (initially off-screen)
    kart_rect = mpatches.Rectangle(
        (-kart_length_m / 2, -kart_width_m / 2),
        kart_length_m,
        kart_width_m,
        color="#0a7",
        zorder=5,
    )
    ax.add_patch(kart_rect)

    info_text = ax.text(
        0.01, 0.99, "",
        transform=ax.transAxes,
        va="top", ha="left",
        fontsize=7,
        family="monospace",
        color="#111",
    )

    # Build MPC param snippet if provided
    param_keys = ["target_speed_pct", "w_d", "w_heading", "w_progress", "w_boundary", "w_terminal_d"]
    param_text_str = ""
    if mpc_params is not None:
        lines_p = []
        for k in param_keys:
            if k in mpc_params:
                lines_p.append(f"{k}={mpc_params[k]:.3g}")
        param_text_str = "\n".join(lines_p[:6])
    param_text = ax.text(
        0.01, 0.01, param_text_str,
        transform=ax.transAxes,
        va="bottom", ha="left",
        fontsize=6,
        family="monospace",
        color="#555",
    )

    trail_len = 30

    def _update(frame_idx):
        k = int(idxs[frame_idx])
        # Trail
        t_start = max(0, frame_idx - trail_len)
        trail_idxs = idxs[t_start : frame_idx + 1]
        trail_line.set_data(xs[trail_idxs], ys[trail_idxs])

        # Kart rectangle: translate + rotate
        import matplotlib.transforms as mtransforms
        cx, cy = float(xs[k]), float(ys[k])
        yaw_deg = float(np.degrees(yaws[k]))
        t = (
            mtransforms.Affine2D()
            .rotate_deg(yaw_deg)
            .translate(cx, cy)
            + ax.transData
        )
        kart_rect.set_transform(t)

        # Text overlay
        info_text.set_text(
            f"t={float(times[k]):.1f}s  v={float(vs[k]):.2f}m/s  d={float(ds[k]):+.2f}m"
        )
        return trail_line, kart_rect, info_text, param_text

    anim = FuncAnimation(fig, _update, frames=n_frames, interval=1000 // fps, blit=True)

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    writer = PillowWriter(fps=fps)
    anim.save(out_path, writer=writer)
    plt.close(fig)
    print(f"wrote {out_path} ({n_frames} frames @ {fps} fps)")


if __name__ == "__main__":
    import argparse
    import json

    ap = argparse.ArgumentParser(description="Render a trajectory.npz as a GIF.")
    ap.add_argument("--trajectory", required=True)
    ap.add_argument("--line", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--params", default=None)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--max-frames", type=int, default=300)
    ap.add_argument("--title", default="Kart sim")
    args = ap.parse_args()

    data = np.load(args.trajectory)
    mpc_params = None
    if args.params:
        with open(args.params) as f:
            mpc_params = json.load(f)

    render_gif(
        xs=data["xs"],
        ys=data["ys"],
        yaws=data["yaws"],
        vs=data["vs"],
        ds=data["ds"],
        times=data["times"],
        line_path=args.line,
        out_path=args.out,
        mpc_params=mpc_params,
        fps=args.fps,
        max_frames=args.max_frames,
        title=args.title,
    )
