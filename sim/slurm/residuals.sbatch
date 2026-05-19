#!/bin/bash -l
#SBATCH --job-name=sim-residual-compare
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=64
#SBATCH --time=12:00:00
#SBATCH -A lilly-rob1
#SBATCH -p smallgpu
#SBATCH --gres=gpu:1
#SBATCH -o $SCRATCH/logs/sim-residual-compare_%j.out
#SBATCH -e $SCRATCH/logs/sim-residual-compare_%j.err

# Compare residual-learner modes (off, rls, gbm, both) with 4 seeds each.
set -euo pipefail

REPO_ROOT="${REPO_ROOT:-$(cd "$(dirname "$0")/../.." && pwd)}"

# Set SIM_PY_ENV_ACTIVATE=/path/to/venv/bin/activate in your .bashrc or before
# calling sbatch, e.g.:   SIM_PY_ENV_ACTIVATE=~/venvs/kart/bin/activate sbatch ...
: "${SIM_PY_ENV_ACTIVATE:=}"
if [[ -n "$SIM_PY_ENV_ACTIVATE" ]]; then
    # shellcheck source=/dev/null
    source "$SIM_PY_ENV_ACTIVATE"
fi

OUT_DIR="${SCRATCH}/sim_out/residual-compare-${SLURM_JOB_ID:-local}-$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT_DIR"
mkdir -p "${SCRATCH}/logs"

echo "Job ${SLURM_JOB_ID:-local}: residual-compare -> $OUT_DIR"

python3 "${REPO_ROOT}/sim/runner.py" compare-residuals \
    --sim datasim \
    --laps 5 \
    --seeds 4 \
    --modes off rls gbm both \
    --n-jobs "${SLURM_CPUS_PER_TASK:-64}" \
    --torch-threads 1 \
    --out-dir "$OUT_DIR" \
    --render

echo "Done. Results in $OUT_DIR"
