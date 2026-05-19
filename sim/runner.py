"""Cluster-friendly runner for the offline kart simulator.

Subcommands:
  one               — run a single MPC param set; optionally render a GIF
  sweep             — Optuna sweep over MPC params, parallel across trials
  render            — render a saved trajectory.npz as a GIF
  compare-residuals — compare multiple residual-learner modes side-by-side
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from typing import Optional

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
if REPO not in sys.path:
    sys.path.insert(0, REPO)

import numpy as np


def load_line(path: str):
    line = []
    with open(path) as f:
        for row in f:
            parts = row.strip().split(",")
            if len(parts) < 6:
                continue
            try:
                line.append(tuple(float(x) for x in parts))
            except ValueError:
                continue
    return line


# ---------------------------------------------------------------------------
# Sweep objective helpers (top-level so they can be serialized by joblib/loky)
# ---------------------------------------------------------------------------
def _sample_params(trial, defaults: dict) -> dict:
    """Optuna trial -> mpc params dict. Mirrors optimize_mpc_sim.sample_params."""
    p = dict(defaults)
    p["horizon_steps"] = trial.suggest_int("horizon_steps", 10, 40)
    p["num_samples"] = trial.suggest_int("num_samples", 32, 128, step=8)
    p["steer_sigma_deg"] = trial.suggest_float("steer_sigma_deg", 2.0, 12.0)
    p["accel_sigma_mps2"] = trial.suggest_float("accel_sigma_mps2", 0.3, 2.5)
    p["proj_window_fwd"] = trial.suggest_int("proj_window_fwd", 30, 120)
    p["target_speed_pct"] = trial.suggest_float("target_speed_pct", 0.4, 1.0)
    p["w_d"] = trial.suggest_float("w_d", 5.0, 500.0, log=True)
    p["w_heading"] = trial.suggest_float("w_heading", 1.0, 200.0, log=True)
    p["w_speed"] = trial.suggest_float("w_speed", 0.5, 50.0, log=True)
    p["w_delta_rate"] = trial.suggest_float("w_delta_rate", 5.0, 300.0, log=True)
    p["w_boundary"] = trial.suggest_float("w_boundary", 100.0, 10000.0, log=True)
    p["w_progress"] = trial.suggest_float("w_progress", 0.1, 50.0, log=True)
    p["w_terminal_d"] = trial.suggest_float("w_terminal_d", 20.0, 600.0, log=True)
    p["w_terminal_heading"] = trial.suggest_float("w_terminal_heading", 10.0, 400.0, log=True)
    p["w_a_lat"] = trial.suggest_float("w_a_lat", 5.0, 300.0, log=True)
    p["w_edge"] = trial.suggest_float("w_edge", 10.0, 2000.0, log=True)
    p["edge_inner_frac"] = trial.suggest_float("edge_inner_frac", 0.2, 0.95)
    return p


def _sample_params_from_spec(spec: dict):
    """Return a _sample_params-compatible function built from a JSON spec dict.

    The spec schema::

        {
          "fixed":  {str: any},           # optional — applied to DEFAULT_MPC first
          "search": [                      # required
            {"name": str, "dist": str, "lo": float, "hi": float, ...}
          ]
        }

    Supported dist values: uniform, log_uniform, int, int_log, categorical.
    Categorical entries carry a "choices" list instead of lo/hi.
    Int entries may carry an optional "step" field.
    """
    fixed = spec.get("fixed", {})
    search = spec["search"]

    def _sampler(trial, defaults: dict) -> dict:
        p = dict(defaults)
        p.update(fixed)
        for entry in search:
            name = entry["name"]
            dist = entry["dist"]
            if dist == "uniform":
                p[name] = trial.suggest_float(name, entry["lo"], entry["hi"])
            elif dist == "log_uniform":
                p[name] = trial.suggest_float(name, entry["lo"], entry["hi"], log=True)
            elif dist == "int":
                step = entry.get("step", 1)
                p[name] = trial.suggest_int(name, int(entry["lo"]), int(entry["hi"]), step=step)
            elif dist == "int_log":
                p[name] = trial.suggest_int(name, int(entry["lo"]), int(entry["hi"]), log=True)
            elif dist == "categorical":
                p[name] = trial.suggest_categorical(name, entry["choices"])
            else:
                raise ValueError(f"unknown dist {dist!r} for param {name!r}")
        return p

    return _sampler


def _sweep_objective(
    trial,
    line,
    defaults: dict,
    sim_backend: str,
    datasim_model_dir: str,
    noise_mode: str,
    n_laps: int,
    seeds_per_trial: int,
    safety_margin: float,
    sample_fn=None,
):
    """Top-level Optuna objective — defined at module level for n_jobs > 1."""
    from sim.closed_loop import simulate

    _fn = sample_fn if sample_fn is not None else _sample_params
    params = _fn(trial, defaults)
    results = []
    for seed in range(seeds_per_trial):
        r = simulate(
            line, params,
            sim_backend=sim_backend,
            datasim_model_dir=datasim_model_dir,
            noise_mode=noise_mode,
            rng_seed=seed,
            n_laps=n_laps,
            safety_margin=safety_margin,
        )
        results.append(r)

    all_complete = all(r.completed_laps >= n_laps and not r.aborted for r in results)
    worst_max_d = max(r.max_abs_d for r in results)
    worst_laps = min(r.completed_laps for r in results)
    lap_times = [r.avg_lap_time_s for r in results if r.completed_laps >= n_laps]
    mean_lap_time = float(np.mean(lap_times)) if lap_times else float("inf")
    worst_lap_time = float(np.max(lap_times)) if lap_times else float("inf")
    mean_speed = float(np.mean([r.avg_speed for r in results]))

    track_half = params.get("track_half_width_m", 2.0)
    kart_half = params.get("kart_half_width_m", 0.5025)
    limit = track_half - kart_half - safety_margin
    safe_all = all_complete and worst_max_d <= limit

    trial.set_user_attr("laps_min", worst_laps)
    trial.set_user_attr("mean_lap_time_s", mean_lap_time)
    trial.set_user_attr("worst_lap_time_s", worst_lap_time)
    trial.set_user_attr("worst_max_abs_d", worst_max_d)
    trial.set_user_attr("mean_speed", mean_speed)
    trial.set_user_attr("safe", safe_all)
    trial.set_user_attr("all_complete", all_complete)

    if not all_complete:
        shortfall = sum(max(0, n_laps - r.completed_laps) for r in results)
        return 1.0e6 + 1.0e4 * shortfall
    if not safe_all:
        over = max(0.0, worst_max_d - limit)
        return 1.0e5 + 1.0e3 * over + mean_lap_time
    return mean_lap_time


# ---------------------------------------------------------------------------
# Subcommand: one
# ---------------------------------------------------------------------------
def _add_one_parser(sub):
    p = sub.add_parser("one", help="run a single MPC param set")
    p.add_argument("--line", default="data/racing_line/line6.csv")
    p.add_argument("--sim", choices=["bicycle", "datasim"], default="datasim")
    p.add_argument("--datasim-model-dir", default="sim/model")
    p.add_argument("--noise-mode", choices=["none", "gaussian", "bootstrap"], default="none")
    p.add_argument("--params-json", default=None, help="JSON overlay onto DEFAULT_MPC")
    p.add_argument("--laps", type=int, default=2)
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--torch-threads", type=int, default=0)
    p.add_argument("--out-dir", default=None)
    p.add_argument("--render", action="store_true")
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--max-frames", type=int, default=300)


def cmd_one(args):
    from sim.closed_loop import DEFAULT_MPC, simulate

    if args.torch_threads > 0:
        import torch
        torch.set_num_threads(args.torch_threads)

    out_dir = args.out_dir or os.path.join("runs", f"one-{int(time.time())}")
    os.makedirs(out_dir, exist_ok=True)

    line_path = args.line if os.path.isabs(args.line) else os.path.join(REPO, args.line)
    line = load_line(line_path)
    if len(line) < 10:
        raise SystemExit(f"racing line at {line_path} too short / not found")

    mpc_params = dict(DEFAULT_MPC)
    if args.params_json:
        with open(args.params_json) as f:
            overlay = json.load(f)
        mpc_params.update(overlay)

    result = simulate(
        line, mpc_params,
        sim_backend=args.sim,
        datasim_model_dir=args.datasim_model_dir,
        noise_mode=args.noise_mode,
        rng_seed=args.seed,
        n_laps=args.laps,
    )

    # Save params
    params_out = dict(mpc_params)
    params_out["_meta"] = {
        "sim": args.sim,
        "noise_mode": args.noise_mode,
        "seed": args.seed,
        "laps": args.laps,
        "line": args.line,
    }
    with open(os.path.join(out_dir, "params.json"), "w") as f:
        json.dump(params_out, f, indent=2)

    # Save metrics
    metrics = {
        "completed_laps": result.completed_laps,
        "total_time_s": result.total_time_s,
        "avg_lap_time_s": result.avg_lap_time_s,
        "max_abs_d": result.max_abs_d,
        "avg_speed": result.avg_speed,
        "safe": result.safe,
        "aborted": result.aborted,
    }
    with open(os.path.join(out_dir, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)

    # Save trajectory
    np.savez(
        os.path.join(out_dir, "trajectory.npz"),
        xs=result.xs, ys=result.ys, yaws=result.yaws,
        vs=result.vs, ds=result.ds, times=result.times,
    )

    if args.render:
        from sim.render import render_gif
        render_gif(
            result.xs, result.ys, result.yaws, result.vs, result.ds, result.times,
            line_path=line_path,
            out_path=os.path.join(out_dir, "out.gif"),
            mpc_params=mpc_params,
            fps=args.fps,
            max_frames=args.max_frames,
            title=f"{args.sim} sim — {result.completed_laps} laps, {result.avg_lap_time_s:.1f}s/lap",
        )

    print(
        f"one: laps={result.completed_laps} avg_lap={result.avg_lap_time_s:.2f}s "
        f"max|d|={result.max_abs_d:.3f}m avg_v={result.avg_speed:.2f}m/s "
        f"safe={result.safe} aborted={result.aborted} -> {out_dir}"
    )


# ---------------------------------------------------------------------------
# Subcommand: sweep
# ---------------------------------------------------------------------------
def _add_sweep_parser(sub):
    p = sub.add_parser("sweep", help="Optuna sweep over MPC params")
    p.add_argument("--line", default="data/racing_line/line6.csv")
    p.add_argument("--sim", choices=["bicycle", "datasim"], default="datasim")
    p.add_argument("--datasim-model-dir", default="sim/model")
    p.add_argument("--noise-mode", choices=["none", "gaussian", "bootstrap"], default="none")
    p.add_argument("--laps", type=int, default=2)
    p.add_argument("--torch-threads", type=int, default=0)
    p.add_argument("--trials", type=int, default=120)
    p.add_argument("--n-jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    p.add_argument("--startup", type=int, default=20)
    p.add_argument("--seeds-per-trial", type=int, default=3)
    p.add_argument("--safety-margin", type=float, default=0.2)
    p.add_argument("--log", choices=["summary", "full"], default="summary")
    p.add_argument("--render-best", action="store_true")
    p.add_argument("--out-dir", default=None)
    p.add_argument("--resume", action="store_true")
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--sweep-spec", default=None, metavar="PATH",
                   help="JSON search-space spec; when set replaces the hardcoded _sample_params")


def cmd_sweep(args):
    import optuna
    import functools
    from sim.closed_loop import DEFAULT_MPC

    optuna.logging.set_verbosity(optuna.logging.WARNING)

    if args.torch_threads > 0:
        import torch
        torch.set_num_threads(args.torch_threads)

    out_dir = args.out_dir or os.path.join("sweeps", f"sweep-{int(time.time())}")
    os.makedirs(out_dir, exist_ok=True)

    line_path = args.line if os.path.isabs(args.line) else os.path.join(REPO, args.line)
    line = load_line(line_path)
    if len(line) < 10:
        raise SystemExit(f"racing line at {line_path} too short / not found")

    sampler = optuna.samplers.TPESampler(seed=args.seed, n_startup_trials=args.startup)
    study = optuna.create_study(direction="minimize", sampler=sampler)

    # Resume: load prior trials from trials.json
    trials_path = os.path.join(out_dir, "trials.json")
    if args.resume and os.path.exists(trials_path):
        with open(trials_path) as f:
            prior = json.load(f)
        for t in prior:
            study.add_trial(optuna.trial.create_trial(
                params=t.get("params", {}),
                distributions={},
                value=t.get("value", float("inf")),
            ))

    # Pick sample function: spec-driven or hardcoded default
    if args.sweep_spec:
        with open(args.sweep_spec) as _sf:
            _spec = json.load(_sf)
        sample_fn = _sample_params_from_spec(_spec)
    else:
        sample_fn = None  # _sweep_objective falls back to _sample_params

    # Bind with functools.partial over a module-level function (required for n_jobs > 1)
    objective = functools.partial(
        _sweep_objective,
        line=line,
        defaults=dict(DEFAULT_MPC),
        sim_backend=args.sim,
        datasim_model_dir=args.datasim_model_dir,
        noise_mode=args.noise_mode,
        n_laps=args.laps,
        seeds_per_trial=args.seeds_per_trial,
        safety_margin=args.safety_margin,
        sample_fn=sample_fn,
    )

    jsonl_path = os.path.join(out_dir, "trials.jsonl")
    log_mode = args.log

    def _per_trial_cb(study, trial):
        attrs = trial.user_attrs
        safe = attrs.get("safe", False)
        laps_min = attrs.get("laps_min", 0)
        lap_t = attrs.get("mean_lap_time_s", float("inf"))
        max_d = attrs.get("worst_max_abs_d", float("inf"))
        all_complete = attrs.get("all_complete", False)
        tag = "OK    " if safe else ("UNSAFE" if all_complete else "FAIL  ")

        # Write JSONL line
        if log_mode == "summary":
            rec = {
                "number": trial.number,
                "value": trial.value,
                "mean_lap_time_s": lap_t,
                "worst_max_abs_d": max_d,
                "safe": safe,
                "all_complete": all_complete,
            }
        else:
            rec = {
                "number": trial.number,
                "value": trial.value,
                "params": trial.params,
                **attrs,
            }
        with open(jsonl_path, "a") as jf:
            jf.write(json.dumps(rec) + "\n")

        # Progress line (<=100 chars)
        msg = (
            f"  trial {trial.number:3d} [{tag}] laps_min={laps_min} "
            f"lap_t={lap_t:.2f}s  max|d|={max_d:.3f}m  obj={trial.value:.3g}"
        )
        print(msg[:100])

    print(
        f"Optuna sweep: {args.trials} trials, {args.laps}-lap eval, "
        f"{args.seeds_per_trial} seed(s), {args.n_jobs} job(s), "
        f"safety={args.safety_margin}m -> {out_dir}"
    )
    t0 = time.time()
    study.optimize(objective, n_trials=args.trials, n_jobs=args.n_jobs,
                   callbacks=[_per_trial_cb])
    print(f"Sweep done in {time.time() - t0:.1f}s")

    # Save all trials
    all_trials = sorted([
        {
            "number": t.number,
            "value": t.value,
            "params": t.params,
            **t.user_attrs,
        }
        for t in study.trials
    ], key=lambda x: x["number"])
    with open(trials_path, "w") as f:
        json.dump(all_trials, f, indent=2)

    # Find best safe trial
    safe_trials = [t for t in study.trials if t.user_attrs.get("safe")]
    best_json_path = os.path.join(out_dir, "best.json")
    if safe_trials:
        best_t = min(safe_trials, key=lambda t: t.user_attrs.get("mean_lap_time_s", float("inf")))
        with open(best_json_path, "w") as f:
            json.dump({
                "trial_number": best_t.number,
                "params": best_t.params,
                "metrics": dict(best_t.user_attrs),
            }, f, indent=2)

    # Write summary.txt
    summary_path = os.path.join(out_dir, "summary.txt")
    with open(summary_path, "w") as f:
        f.write(f"Sweep: {len(all_trials)} trials total, {len(safe_trials)} safe\n\n")
        if safe_trials:
            top10 = sorted(safe_trials, key=lambda t: t.user_attrs.get("mean_lap_time_s", float("inf")))[:10]
            f.write("Top-10 safe trials by mean lap time:\n")
            for t in top10:
                attrs = t.user_attrs
                f.write(
                    f"  #{t.number:3d}  lap_t={attrs.get('mean_lap_time_s', float('inf')):.3f}s"
                    f"  max|d|={attrs.get('worst_max_abs_d', float('inf')):.3f}m"
                    f"  speed={attrs.get('mean_speed', 0):.2f}m/s\n"
                )
            best_lap = top10[0].user_attrs.get("mean_lap_time_s", float("inf"))
            f.write(f"\n{len(safe_trials)} safe / {len(all_trials)} trials, best lap = {best_lap:.3f}s\n")
        else:
            f.write("No safe trials found.\n")

    if args.render_best and safe_trials:
        from sim.closed_loop import DEFAULT_MPC, simulate
        from sim.render import render_gif

        best_params = dict(DEFAULT_MPC)
        best_params.update(best_t.params)
        result = simulate(
            line, best_params,
            sim_backend=args.sim,
            datasim_model_dir=args.datasim_model_dir,
            noise_mode=args.noise_mode,
            rng_seed=0,
            n_laps=args.laps,
            max_steps=200000,
        )
        np.savez(
            os.path.join(out_dir, "best_trajectory.npz"),
            xs=result.xs, ys=result.ys, yaws=result.yaws,
            vs=result.vs, ds=result.ds, times=result.times,
        )
        render_gif(
            result.xs, result.ys, result.yaws, result.vs, result.ds, result.times,
            line_path=line_path,
            out_path=os.path.join(out_dir, "best.gif"),
            mpc_params=best_params,
            title=f"Best trial #{best_t.number} — {result.completed_laps} laps",
        )

    n_safe = len(safe_trials)
    best_lap_str = (
        f"{min(t.user_attrs.get('mean_lap_time_s', float('inf')) for t in safe_trials):.2f}s"
        if safe_trials else "N/A"
    )
    print(
        f"sweep: {len(all_trials)} trials, {n_safe} safe, "
        f"best_lap={best_lap_str} -> {out_dir}"
    )


# ---------------------------------------------------------------------------
# Subcommand: render
# ---------------------------------------------------------------------------
def _add_render_parser(sub):
    p = sub.add_parser("render", help="render a saved trajectory.npz as a GIF")
    p.add_argument("--trajectory", required=True)
    p.add_argument("--line", required=True)
    p.add_argument("--params", default=None)
    p.add_argument("--out", required=True)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--max-frames", type=int, default=300)
    p.add_argument("--title", default="Kart sim")


def cmd_render(args):
    from sim.render import render_gif

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
        line_path=args.line if os.path.isabs(args.line) else os.path.join(REPO, args.line),
        out_path=args.out,
        mpc_params=mpc_params,
        fps=args.fps,
        max_frames=args.max_frames,
        title=args.title,
    )


# ---------------------------------------------------------------------------
# Subcommand: compare-residuals
# ---------------------------------------------------------------------------

# Mode -> param overrides applied on top of the user's --params-json overlay.
_RESIDUAL_MODE_OVERRIDES = {
    "off":    {"residual.mode": "off"},
    "rls":    {"residual.mode": "apply", "mpc.residual.use_gbm_on_select": False,
               "mpc.residual.gbm_enabled": False},
    "gbm":    {"residual.mode": "apply", "mpc.residual.use_gbm_on_select": True,
               "mpc.residual.gbm_enabled": True},
    "both":   {"residual.mode": "apply", "mpc.residual.gbm_enabled": True},
    "shadow": {"residual.mode": "shadow"},
}

_ALL_MODES = list(_RESIDUAL_MODE_OVERRIDES.keys())


def _run_one_compare(mode: str, seed: int, args_dict: dict) -> dict:
    """Top-level picklable job for joblib.Parallel."""
    import traceback as _tb
    from sim.closed_loop import DEFAULT_MPC, simulate

    out_dir = args_dict["out_dir"]
    run_dir = os.path.join(out_dir, f"{mode}_seed{seed}")
    os.makedirs(run_dir, exist_ok=True)

    mpc_params = dict(DEFAULT_MPC)
    if args_dict.get("params_json"):
        with open(args_dict["params_json"]) as f:
            mpc_params.update(json.load(f))
    mpc_params.update(_RESIDUAL_MODE_OVERRIDES.get(mode, {}))

    line_path = args_dict["line_path"]
    line = load_line(line_path)

    try:
        result = simulate(
            line, mpc_params,
            sim_backend=args_dict["sim"],
            datasim_model_dir=args_dict["datasim_model_dir"],
            noise_mode=args_dict["noise_mode"],
            rng_seed=seed,
            n_laps=args_dict["laps"],
        )
        aborted = result.aborted
        abort_step = None
        abort_xy = None
        abort_t = None
        max_d_step = int(np.argmax(np.abs(result.ds))) if result.ds.size else None
        max_d_value = float(result.max_abs_d)
        completed_laps = result.completed_laps
        lap_time = result.avg_lap_time_s
        avg_speed = result.avg_speed
        total_time = result.total_time_s
    except Exception:
        aborted = True
        abort_step = -1
        abort_xy = None
        abort_t = None
        max_d_step = None
        max_d_value = float("inf")
        completed_laps = 0
        lap_time = float("inf")
        avg_speed = 0.0
        total_time = 0.0
        result = None
        print(f"  [{mode} seed{seed}] EXCEPTION:\n{_tb.format_exc()}", flush=True)

    if result is not None:
        np.savez(
            os.path.join(run_dir, "trajectory.npz"),
            xs=result.xs, ys=result.ys, yaws=result.yaws,
            vs=result.vs, ds=result.ds, times=result.times,
        )
        metrics = {
            "completed_laps": result.completed_laps,
            "total_time_s": result.total_time_s,
            "avg_lap_time_s": result.avg_lap_time_s,
            "max_abs_d": result.max_abs_d,
            "avg_speed": result.avg_speed,
            "safe": result.safe,
            "aborted": result.aborted,
        }
    else:
        metrics = {
            "completed_laps": 0,
            "total_time_s": 0.0,
            "avg_lap_time_s": float("inf"),
            "max_abs_d": float("inf"),
            "avg_speed": 0.0,
            "safe": False,
            "aborted": True,
        }
    with open(os.path.join(run_dir, "metrics.json"), "w") as f:
        json.dump(metrics, f, indent=2)

    saved_params = dict(mpc_params)
    saved_params["_meta"] = {"mode": mode, "seed": seed}
    with open(os.path.join(run_dir, "params.json"), "w") as f:
        json.dump(saved_params, f, indent=2)

    if result is not None and aborted and result.ds.size:
        abort_step = len(result.ds) - 1
        abort_xy = [float(result.xs[-1]), float(result.ys[-1])]
        abort_t = float(result.times[-1])

    return {
        "mode": mode,
        "seed": seed,
        "run_dir": run_dir,
        "aborted": aborted,
        "abort_step": abort_step,
        "abort_xy": abort_xy,
        "abort_t": abort_t,
        "max_d_step": max_d_step,
        "max_d_value": max_d_value,
        "completed_laps": completed_laps,
        "lap_time": lap_time,
        "avg_speed": avg_speed,
        "total_time": total_time,
        "result": result,  # kept in-process for render; not serialised by joblib
    }


def _add_compare_residuals_parser(sub):
    p = sub.add_parser("compare-residuals",
                       help="compare multiple residual-learner modes side-by-side")
    p.add_argument("--line", default="data/racing_line/line6.csv")
    p.add_argument("--sim", choices=["bicycle", "datasim"], default="datasim")
    p.add_argument("--datasim-model-dir", default="sim/model")
    p.add_argument("--noise-mode", choices=["none", "gaussian", "bootstrap"], default="none")
    p.add_argument("--params-json", default=None, metavar="PATH",
                   help="JSON overlay onto DEFAULT_MPC applied to every mode")
    p.add_argument("--modes", nargs="+", default=["off", "rls", "gbm", "both", "shadow"],
                   metavar="MODE")
    p.add_argument("--laps", type=int, default=5)
    p.add_argument("--seeds", type=int, default=2, metavar="N")
    p.add_argument("--n-jobs", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    p.add_argument("--torch-threads", type=int, default=0)
    p.add_argument("--out-dir", default=None)
    p.add_argument("--render", action="store_true",
                   help="write comparison.png with 4 trajectories overlaid")


def cmd_compare_residuals(args):
    from joblib import Parallel, delayed

    if args.torch_threads > 0:
        import torch
        torch.set_num_threads(args.torch_threads)

    ts = time.strftime("%Y%m%d-%H%M%S")
    out_dir = args.out_dir or os.path.join("runs", f"residual-compare-{ts}")
    os.makedirs(out_dir, exist_ok=True)

    line_path = args.line if os.path.isabs(args.line) else os.path.join(REPO, args.line)
    line = load_line(line_path)
    if len(line) < 10:
        raise SystemExit(f"racing line at {line_path} too short / not found")

    # Validate modes
    for m in args.modes:
        if m not in _RESIDUAL_MODE_OVERRIDES:
            raise SystemExit(f"unknown mode {m!r}; valid: {_ALL_MODES}")

    args_dict = {
        "out_dir": out_dir,
        "line_path": line_path,
        "sim": args.sim,
        "datasim_model_dir": args.datasim_model_dir,
        "noise_mode": args.noise_mode,
        "params_json": args.params_json,
        "laps": args.laps,
    }

    jobs = [(mode, seed) for mode in args.modes for seed in range(args.seeds)]
    print(
        f"compare-residuals: {len(jobs)} runs "
        f"({len(args.modes)} modes × {args.seeds} seed(s)), "
        f"n_jobs={args.n_jobs} -> {out_dir}"
    )

    raw_results = Parallel(n_jobs=args.n_jobs)(
        delayed(_run_one_compare)(mode, seed, args_dict)
        for mode, seed in jobs
    )

    # ------------------------------------------------------------------
    # Aggregate per-mode stats
    # ------------------------------------------------------------------
    from collections import defaultdict
    mode_results = defaultdict(list)
    for r in raw_results:
        mode_results[r["mode"]].append(r)

    track_half = 2.0
    kart_half = 0.5025
    safety_margin = 0.2
    limit = track_half - kart_half - safety_margin

    summary = {}
    failure_points = []

    for mode in args.modes:
        rs = mode_results[mode]
        laps_vals = [r["completed_laps"] for r in rs]
        lap_times = [r["lap_time"] for r in rs if r["lap_time"] != float("inf")]
        max_ds = [r["max_d_value"] for r in rs if r["max_d_value"] != float("inf")]
        aborted_count = sum(1 for r in rs if r["aborted"])
        summary[mode] = {
            "laps_mean": float(np.mean(laps_vals)) if laps_vals else 0.0,
            "laps_min": int(min(laps_vals)) if laps_vals else 0,
            "lap_time_mean": float(np.mean(lap_times)) if lap_times else float("inf"),
            "lap_time_std": float(np.std(lap_times)) if len(lap_times) > 1 else 0.0,
            "max_abs_d_max": float(max(max_ds)) if max_ds else float("inf"),
            "aborted_count": aborted_count,
            "n_runs": len(rs),
        }
        for r in rs:
            is_failure = r["aborted"] or r["max_d_value"] > limit
            if is_failure:
                failure_points.append({
                    "mode": r["mode"],
                    "seed": r["seed"],
                    "abort_step": r["abort_step"],
                    "abort_xy": r["abort_xy"],
                    "abort_t": r["abort_t"],
                    "max_d_step": r["max_d_step"],
                    "max_d_value": r["max_d_value"],
                })

    with open(os.path.join(out_dir, "summary.json"), "w") as f:
        json.dump(summary, f, indent=2)
    with open(os.path.join(out_dir, "failure_points.json"), "w") as f:
        json.dump(failure_points, f, indent=2)

    # Summary table
    col_w = 10
    header = f"{'mode':<12} {'laps_mean':>{col_w}} {'laps_min':>{col_w}} {'lap_t_mean':>{col_w}} {'lap_t_std':>{col_w}} {'max|d|_max':>{col_w}} {'aborted':>{col_w}}"
    rows = [header, "-" * len(header)]
    for mode in args.modes:
        s = summary[mode]
        lm = f"{s['laps_mean']:.2f}" if s['laps_mean'] != float("inf") else "inf"
        li = str(s['laps_min'])
        lt = f"{s['lap_time_mean']:.3f}" if s['lap_time_mean'] != float("inf") else "inf"
        ls = f"{s['lap_time_std']:.3f}"
        md = f"{s['max_abs_d_max']:.3f}" if s['max_abs_d_max'] != float("inf") else "inf"
        ab = str(s['aborted_count'])
        rows.append(f"{mode:<12} {lm:>{col_w}} {li:>{col_w}} {lt:>{col_w}} {ls:>{col_w}} {md:>{col_w}} {ab:>{col_w}}")

    summary_txt = "\n".join(rows) + "\n"
    with open(os.path.join(out_dir, "summary.txt"), "w") as f:
        f.write(summary_txt)

    # ------------------------------------------------------------------
    # Optional comparison plot
    # ------------------------------------------------------------------
    if args.render:
        _render_comparison(raw_results, args.modes, line_path, out_dir)

    print(summary_txt)
    print(f"wrote {out_dir}")


def _render_comparison(raw_results, modes, line_path, out_dir):
    """Write comparison.png: up to 4 trajectories overlaid on track."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from sim.render import load_line_xy, _tangent_normal

    lx, ly = load_line_xy(line_path)
    nx, ny = _tangent_normal(lx, ly)
    track_half = 2.0
    left_x = lx + nx * track_half
    left_y = ly + ny * track_half
    right_x = lx - nx * track_half
    right_y = ly - ny * track_half

    colors = ["#e63", "#36e", "#3a3", "#c3c", "#a80"]
    fig, ax = plt.subplots(figsize=(8, 7))
    ax.set_aspect("equal")
    ax.axis("off")
    ax.plot(left_x, left_y, color="#555", lw=0.8)
    ax.plot(right_x, right_y, color="#555", lw=0.8)
    ax.plot(lx, ly, color="#bbb", lw=0.5, ls="--")

    # One trajectory per mode (first seed that has data)
    plotted = []
    for i, mode in enumerate(modes[:4]):
        col = colors[i % len(colors)]
        for r in raw_results:
            if r["mode"] == mode and r.get("result") is not None and r["result"].xs.size:
                xs = r["result"].xs
                ys = r["result"].ys
                ax.plot(xs, ys, color=col, lw=1.0, alpha=0.8, label=mode)
                plotted.append(mode)
                break

    if plotted:
        ax.legend(fontsize=7, loc="upper right")
    ax.set_title("Residual mode comparison (first seed each)", fontsize=8)
    out_path = os.path.join(out_dir, "comparison.png")
    fig.savefig(out_path, dpi=120, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {out_path}")


# ---------------------------------------------------------------------------
# Main dispatch
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    sub = ap.add_subparsers(dest="cmd", required=True)
    _add_one_parser(sub)
    _add_sweep_parser(sub)
    _add_render_parser(sub)
    _add_compare_residuals_parser(sub)
    args = ap.parse_args()
    {
        "one": cmd_one,
        "sweep": cmd_sweep,
        "render": cmd_render,
        "compare-residuals": cmd_compare_residuals,
    }[args.cmd](args)


if __name__ == "__main__":
    main()
