"""Open-loop multi-step rollout RMSE on a holdout bag, comparing three sims."""
from __future__ import annotations

from typing import Dict, List

import numpy as np


def rollout_rmse(
    sim,
    bag: dict,
    anchor_indices: List[int],
    horizons_ticks: List[int],
    dt: float,
) -> Dict[str, Dict[int, float]]:
    """For each anchor, snap sim to bag state, feed bag commands, compare
    against bag /odom at each horizon. Returns
    {"dx": {h: rmse}, "dy": ..., "dyaw": ..., "dv": ...}."""
    sq = {m: {h: [] for h in horizons_ticks} for m in ("dx", "dy", "dyaw", "dv")}
    H = max(horizons_ticks)
    for a in anchor_indices:
        if a + H >= bag["x"].size:
            continue
        sim.reset(float(bag["x"][a]), float(bag["y"][a]), float(bag["yaw"][a]))
        sim.v = float(bag["v"][a])
        for h in range(1, H + 1):
            i = a + h - 1
            x, y, yaw, v = sim.step(float(bag["cmd_throttle"][i]),
                                    float(bag["cmd_steer"][i]), dt)
            if h in sq["dx"]:
                sq["dx"][h].append((x - bag["x"][a + h]) ** 2)
                sq["dy"][h].append((y - bag["y"][a + h]) ** 2)
                sq["dyaw"][h].append(_angle_diff(yaw, bag["yaw"][a + h]) ** 2)
                sq["dv"][h].append((v - bag["v"][a + h]) ** 2)
    return {m: {h: float(np.sqrt(np.mean(vs))) for h, vs in by_h.items() if vs}
            for m, by_h in sq.items()}


def _angle_diff(a: float, b: float) -> float:
    d = a - b
    return float(np.arctan2(np.sin(d), np.cos(d)))


def _bag_to_dict(npz_path: str) -> dict:
    from sim.load_bags import AlignedBag
    bag = AlignedBag.from_npz(npz_path)
    return {
        "t": bag.t,
        "x": bag.odom_x, "y": bag.odom_y, "yaw": bag.odom_yaw, "v": bag.v,
        "cmd_throttle": bag.cmd_throttle, "cmd_steer": bag.cmd_steer,
        "autonomous": bag.autonomous,
    }


def _build_bicycle_sim():
    """sim_bicycle.BicycleModel wrapped as a DataSim-shaped step()."""
    import os
    import sys
    REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    sys.path.insert(0, os.path.join(REPO, "src", "autonomous_kart"))
    from autonomous_kart.nodes.localization.sim_bicycle import BicycleModel
    return BicycleModel(wheelbase_m=1.05, v_max_mps=12.0, steer_max_deg=60.0)


def _build_id_sim(params_path, noise_path):
    from sim.data_sim import DataSim
    return DataSim(params_path=params_path, noise_path=noise_path, noise_mode="none")


def _build_full_sim(params_path, noise_path, mlp_path, norm_path, clamp_path):
    from sim.data_sim import DataSim
    return DataSim(params_path=params_path, noise_path=noise_path,
                   mlp_path=mlp_path, norm_path=norm_path, clamp_path=clamp_path,
                   noise_mode="none")


def plot_validation(json_path: str, out_png: str) -> None:
    import json
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(json_path) as f:
        payload = json.load(f)
    horizons = payload["horizons_ticks"]
    dt = payload["dt"]
    results = payload["results"]
    horizons_s = [h * dt for h in horizons]
    metrics = [("dx", "x error (m)"), ("dy", "y error (m)"),
               ("dyaw", "yaw error (rad)"), ("dv", "v error (m/s)")]
    colors = {"sim_bicycle": "#888", "id_only": "#37c", "full": "#c33"}

    fig, axes = plt.subplots(1, len(metrics), figsize=(15, 4))
    for ax, (key, label) in zip(axes, metrics):
        for sim_name in ("sim_bicycle", "id_only", "full"):
            ys = [results[sim_name][key][str(h)] for h in horizons]
            ax.plot(horizons_s, ys, marker="o", color=colors[sim_name], label=sim_name)
        ax.set_xlabel("horizon (s)")
        ax.set_ylabel(label)
        ax.grid(alpha=0.3)
    axes[0].legend(loc="upper left", frameon=False)
    fig.tight_layout()
    fig.savefig(out_png, dpi=140)
    plt.close(fig)
    print(f"wrote {out_png}")


def build_pdf_report(
    json_path: str,
    out_pdf: str,
    params_path: str,
    bag: dict,
    sims: dict,
    anchors: list,
    horizons_ticks: list,
    dt: float,
) -> None:
    import json
    import os
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_pdf import PdfPages

    with open(json_path) as f:
        payload = json.load(f)
    results = payload["results"]
    horizons_s = [h * dt for h in horizons_ticks]
    metrics = [("dx", "x err (m)"), ("dy", "y err (m)"),
               ("dyaw", "yaw err (rad)"), ("dv", "v err (m/s)")]
    colors = {"sim_bicycle": "#888", "id_only": "#37c", "full": "#c33"}

    with open(params_path) as f:
        params_payload = json.load(f)

    with PdfPages(out_pdf) as pdf:
        # Page 1: title + verdict + identified params
        fig = plt.figure(figsize=(11, 8.5))
        fig.text(0.5, 0.92, "Data-Driven Kart Sim — Validation Report",
                 ha="center", fontsize=16, weight="bold")
        fig.text(0.5, 0.88, os.path.basename(json_path), ha="center", fontsize=9, color="#666")

        all_strict = all(
            results["full"][m][str(h)] < min(results["sim_bicycle"][m][str(h)],
                                              results["id_only"][m][str(h)])
            for m, _ in metrics for h in horizons_ticks
        )
        verdict = "PASS — full sim strictly beats both baselines on every horizon and metric."
        if not all_strict:
            verdict = "PARTIAL — full sim wins on most but not all horizons/metrics. See tables."
        fig.text(0.5, 0.80, verdict, ha="center", fontsize=11)

        param_lines = [f"{k}: {v:.4f}" for k, v in params_payload["params"].items()]
        fig.text(0.1, 0.55, "Identified bicycle parameters:", fontsize=11, weight="bold")
        for i, line in enumerate(param_lines):
            fig.text(0.1, 0.50 - 0.025 * i, line, family="monospace", fontsize=9)

        fig.text(0.55, 0.55, "Setup", fontsize=11, weight="bold")
        fig.text(0.55, 0.50, f"Holdout bag: {payload.get('holdout_tag', '231452')}", fontsize=9)
        fig.text(0.55, 0.475, f"dt: {dt:.4f}s  ({1.0/dt:.1f} Hz grid)", fontsize=9)
        fig.text(0.55, 0.45, f"Anchors used: {len(anchors)}", fontsize=9)
        fig.text(0.55, 0.425, f"Horizons (s): {horizons_s}", fontsize=9)
        pdf.savefig(fig)
        plt.close(fig)

        # Page 2: per-metric curves
        fig, axes = plt.subplots(2, 2, figsize=(11, 8.5))
        for ax, (key, label) in zip(axes.ravel(), metrics):
            for sim_name in ("sim_bicycle", "id_only", "full"):
                ys = [results[sim_name][key][str(h)] for h in horizons_ticks]
                ax.plot(horizons_s, ys, marker="o", color=colors[sim_name], label=sim_name)
            ax.set_xlabel("horizon (s)")
            ax.set_ylabel(label)
            ax.grid(alpha=0.3)
        axes[0, 0].legend(loc="upper left", frameon=False)
        fig.suptitle("RMSE vs rollout horizon — three-way comparison")
        fig.tight_layout(rect=(0, 0, 1, 0.96))
        pdf.savefig(fig)
        plt.close(fig)

        # Page 3: RMSE table
        fig = plt.figure(figsize=(11, 8.5))
        fig.text(0.5, 0.95, "RMSE table by horizon", ha="center", fontsize=14, weight="bold")
        col_labels = [f"{h * dt:.2f}s" for h in horizons_ticks]
        row_labels = []
        cells = []
        for sim_name in ("sim_bicycle", "id_only", "full"):
            for key, label in metrics:
                row_labels.append(f"{sim_name} — {key}")
                cells.append([f"{results[sim_name][key][str(h)]:.3g}" for h in horizons_ticks])
        ax = fig.add_axes((0.1, 0.1, 0.8, 0.8))
        ax.axis("off")
        table = ax.table(cellText=cells, rowLabels=row_labels, colLabels=col_labels,
                         loc="center", cellLoc="right")
        table.auto_set_font_size(False)
        table.set_fontsize(8)
        pdf.savefig(fig)
        plt.close(fig)

        # Page 4: trajectory overlays
        sample_anchors = anchors[:: max(1, len(anchors) // 5)][:5]
        H_show = horizons_ticks[-1]
        fig, axes = plt.subplots(2, 3, figsize=(13, 8.5))
        for ax, a in zip(axes.ravel(), sample_anchors):
            ax.plot(bag["x"][a:a + H_show + 1], bag["y"][a:a + H_show + 1],
                    color="#333", lw=1.5, label="bag /odom")
            for sim_name, sim in sims.items():
                sim.reset(float(bag["x"][a]), float(bag["y"][a]), float(bag["yaw"][a]))
                sim.v = float(bag["v"][a])
                xs, ys = [sim.x], [sim.y]
                for h in range(H_show):
                    x, y, _, _ = sim.step(float(bag["cmd_throttle"][a + h]),
                                          float(bag["cmd_steer"][a + h]), dt)
                    xs.append(x); ys.append(y)
                ax.plot(xs, ys, color=colors[sim_name], lw=1.0, label=sim_name)
            ax.set_aspect("equal", adjustable="datalim")
            ax.set_title(f"anchor t={bag['t'][a]:.1f}s")
            ax.grid(alpha=0.3)
        axes.ravel()[0].legend(loc="upper left", frameon=False, fontsize=8)
        if len(sample_anchors) < 6:
            axes.ravel()[-1].axis("off")
        fig.suptitle("Trajectory overlays over the longest horizon")
        fig.tight_layout(rect=(0, 0, 1, 0.96))
        pdf.savefig(fig)
        plt.close(fig)

    print(f"wrote {out_pdf}")


def main():
    import argparse
    import json
    import os
    ap = argparse.ArgumentParser()
    ap.add_argument("--holdout", required=True)
    ap.add_argument("--params", required=True)
    ap.add_argument("--noise", required=True)
    ap.add_argument("--mlp", required=True)
    ap.add_argument("--norm", required=True)
    ap.add_argument("--clamp", required=True)
    ap.add_argument("--out-json", required=True)
    ap.add_argument("--out-png", default="")
    ap.add_argument("--out-pdf", default="")
    ap.add_argument("--anchor-stride", type=int, default=30)
    args = ap.parse_args()

    bag = _bag_to_dict(args.holdout)
    dt = float(np.median(np.diff(bag["t"])))
    horizons_ticks = [int(round(h / dt)) for h in (0.1, 0.5, 1.0, 2.0)]
    anchors = [i for i in range(0, bag["x"].size - max(horizons_ticks) - 1, args.anchor_stride)
               if bag["autonomous"][i]]
    print(f"{len(anchors)} anchors, horizons (ticks) = {horizons_ticks}")

    sims = {
        "sim_bicycle": _build_bicycle_sim(),
        "id_only": _build_id_sim(args.params, args.noise),
        "full": _build_full_sim(args.params, args.noise, args.mlp, args.norm, args.clamp),
    }
    results = {name: rollout_rmse(sim, bag, anchors, horizons_ticks, dt)
               for name, sim in sims.items()}
    os.makedirs(os.path.dirname(args.out_json) or ".", exist_ok=True)
    with open(args.out_json, "w") as f:
        json.dump({"horizons_ticks": horizons_ticks, "dt": dt,
                   "holdout_tag": os.path.basename(args.holdout),
                   "results": results}, f, indent=2)
    print(f"wrote {args.out_json}")
    if args.out_png:
        plot_validation(args.out_json, args.out_png)
    if args.out_pdf:
        build_pdf_report(args.out_json, args.out_pdf,
                         args.params, bag, sims, anchors, horizons_ticks, dt)
    for name, res in results.items():
        print(f"  {name}: dy@1s = {res['dy'].get(horizons_ticks[2], float('nan')):.3f} m")


if __name__ == "__main__":
    import os
    import sys
    REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if REPO not in sys.path:
        sys.path.insert(0, REPO)
    main()
