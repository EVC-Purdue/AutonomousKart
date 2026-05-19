#!/bin/bash -l
#SBATCH --job-name=sim-one-20laps
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=64
#SBATCH --time=12:00:00
#SBATCH -A lilly-rob1
#SBATCH -p smallgpu
#SBATCH --gres=gpu:1
#SBATCH -o $SCRATCH/logs/sim-one-20laps_%j.out
#SBATCH -e $SCRATCH/logs/sim-one-20laps_%j.err

# Run a single 20-lap closed-loop MPC simulation with residuals applied and render a GIF.
set -euo pipefail

# Repo root: directory containing this script's parent (sim/sbatch/../.. = repo root).
# Override via:  REPO_ROOT=/your/path sbatch 01_run_one.sbatch
REPO_ROOT="${REPO_ROOT:-$(cd "$(dirname "$0")/../.." && pwd)}"

# Python environment activation.
# Set SIM_PY_ENV_ACTIVATE=/path/to/venv/bin/activate in your .bashrc or before
# calling sbatch, e.g.:   SIM_PY_ENV_ACTIVATE=~/venvs/kart/bin/activate sbatch ...
: "${SIM_PY_ENV_ACTIVATE:=}"
if [[ -n "$SIM_PY_ENV_ACTIVATE" ]]; then
    # shellcheck source=/dev/null
    source "$SIM_PY_ENV_ACTIVATE"
fi

OUT_DIR="${SCRATCH}/sim_out/one-${SLURM_JOB_ID:-local}-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT_DIR"
mkdir -p "${SCRATCH}/logs"

# Write MPC params overlay to a temp file — edit these values as needed.
PARAMS_JSON="$(mktemp --suffix=.json)"
cat > "$PARAMS_JSON" <<'PARAMS_EOF'
{
  "residual.mode": "apply"
}
PARAMS_EOF

echo "Job ${SLURM_JOB_ID:-local}: one/20-lap run -> $OUT_DIR"

python3 "${REPO_ROOT}/sim/runner.py" one \
    --sim datasim \
    --laps 20 \
    --seed 0 \
    --torch-threads "${SLURM_CPUS_PER_TASK:-64}" \
    --params-json "$PARAMS_JSON" \
    --out-dir "$OUT_DIR" \
    --render \
    --fps 30 \
    --max-frames 300

rm -f "$PARAMS_JSON"
echo "Done. Results in $OUT_DIR"
