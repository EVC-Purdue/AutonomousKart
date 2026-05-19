#!/bin/bash -l
#SBATCH --job-name=sim-sweep
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=64
#SBATCH --time=12:00:00
#SBATCH -A lilly-rob1
#SBATCH -p smallgpu
#SBATCH --gres=gpu:1
#SBATCH -o $SCRATCH/logs/sim-sweep_%j.out
#SBATCH -e $SCRATCH/logs/sim-sweep_%j.err

# Run a 400-trial Bayesian (Optuna TPE) parameter sweep over MPC weights.
set -euo pipefail

# Repo root resolution. SLURM spools the script to /var/spool so "$0" doesn't
# point at the original path — use $SLURM_SUBMIT_DIR (set to the dir where you
# ran `sbatch`). Submit from the repo root, or set REPO_ROOT explicitly.
REPO_ROOT="${REPO_ROOT:-${SLURM_SUBMIT_DIR:-$PWD}}"
if [[ ! -f "${REPO_ROOT}/sim/runner.py" ]]; then
    echo "ERROR: sim/runner.py not found under REPO_ROOT=${REPO_ROOT}" >&2
    echo "  Run 'sbatch sim/slurm/sweep_params.sbatch' from the repo root, or set REPO_ROOT explicitly." >&2
    exit 1
fi

# Set SIM_PY_ENV_ACTIVATE=/path/to/venv/bin/activate in your .bashrc or before
# calling sbatch, e.g.:   SIM_PY_ENV_ACTIVATE=~/venvs/kart/bin/activate sbatch ...
: "${SIM_PY_ENV_ACTIVATE:=}"
if [[ -n "$SIM_PY_ENV_ACTIVATE" ]]; then
    # shellcheck source=/dev/null
    source "$SIM_PY_ENV_ACTIVATE"
fi

OUT_DIR="${SCRATCH}/sim_out/sweep-${SLURM_JOB_ID:-local}-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT_DIR"
mkdir -p "${SCRATCH}/logs"

# === SWEEP PARAMS — edit me ===
# Write the search-space spec to a temp JSON file.
# Add/remove entries from "search", or change the "fixed" block below.
SPEC_JSON="$(mktemp --suffix=.json)"
cat > "$SPEC_JSON" <<'SPEC_EOF'
{
  "fixed": {
    "residual.mode": "shadow"
  },
  "search": [
    {"name": "w_d",              "dist": "log_uniform", "lo": 5.0,  "hi": 500.0},
    {"name": "w_heading",        "dist": "log_uniform", "lo": 1.0,  "hi": 200.0},
    {"name": "w_progress",       "dist": "log_uniform", "lo": 0.1,  "hi": 50.0},
    {"name": "w_terminal_d",     "dist": "log_uniform", "lo": 20.0, "hi": 600.0},
    {"name": "w_a_lat",          "dist": "log_uniform", "lo": 5.0,  "hi": 300.0},
    {"name": "target_speed_pct", "dist": "uniform",     "lo": 0.4,  "hi": 1.0},
    {"name": "horizon_steps",    "dist": "int",         "lo": 10,   "hi": 40},
    {"name": "edge_inner_frac",  "dist": "uniform",     "lo": 0.2,  "hi": 0.95}
  ]
}
SPEC_EOF
# === end sweep params ===

echo "Job ${SLURM_JOB_ID:-local}: sweep -> $OUT_DIR"

python3 "${REPO_ROOT}/sim/runner.py" sweep \
    --sim datasim \
    --trials 400 \
    --n-jobs "${SLURM_CPUS_PER_TASK:-64}" \
    --startup 30 \
    --laps 2 \
    --seeds-per-trial 3 \
    --torch-threads 1 \
    --sweep-spec "$SPEC_JSON" \
    --out-dir "$OUT_DIR" \
    --render-best \
    --log summary

rm -f "$SPEC_JSON"
echo "Done. Results in $OUT_DIR"
