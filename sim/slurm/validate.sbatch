#!/bin/bash -l
#SBATCH --job-name=sim-validate
#SBATCH --nodes=1
#SBATCH --ntasks=1
#SBATCH --cpus-per-task=64
#SBATCH --time=12:00:00
#SBATCH -A lilly-rob1
#SBATCH -p smallgpu
#SBATCH --gres=gpu:1
#SBATCH -o $SCRATCH/logs/sim-validate_%j.out
#SBATCH -e $SCRATCH/logs/sim-validate_%j.err

# Recreate the open-loop multi-step rollout validation that produced the 5x /
# 4.4x improvement of the data sim over sim_bicycle.py.  Holdout bag is the
# smallest of the three (231452), never seen during MLP training.
set -euo pipefail

# Repo root resolution. SLURM spools the script to /var/spool so "$0" doesn't
# point at the original path — use $SLURM_SUBMIT_DIR (set to the dir where you
# ran `sbatch`). Submit from the repo root, or set REPO_ROOT explicitly.
REPO_ROOT="${REPO_ROOT:-${SLURM_SUBMIT_DIR:-$PWD}}"
if [[ ! -f "${REPO_ROOT}/sim/runner.py" ]]; then
    echo "ERROR: sim/runner.py not found under REPO_ROOT=${REPO_ROOT}" >&2
    echo "  Run 'sbatch sim/slurm/validate.sbatch' from the repo root, or set REPO_ROOT explicitly." >&2
    exit 1
fi

# Optional python env activation.
: "${SIM_PY_ENV_ACTIVATE:=}"
if [[ -n "${SIM_PY_ENV_ACTIVATE}" ]]; then
    # shellcheck source=/dev/null
    source "${SIM_PY_ENV_ACTIVATE}"
fi

cd "${REPO_ROOT}"

# =============================================================================
# Paths.  Bags live under bags/; aligned npz cache goes under docs/superpowers
# (gitignored).  The validation outputs go under $SCRATCH so they survive past
# the job.
# =============================================================================
CACHE_DIR="${REPO_ROOT}/docs/superpowers/data_sim/cache"
MODEL_DIR="${REPO_ROOT}/sim/model"
OUT_DIR="${OUT_DIR:-${SCRATCH}/sim_out/validate-${SLURM_JOB_ID:-local}-$(date +%Y%m%d-%H%M%S)}"
export OUT_DIR
mkdir -p "${OUT_DIR}" "${CACHE_DIR}"

echo "Job ${SLURM_JOB_ID:-local}: validate -> ${OUT_DIR}"

# =============================================================================
# Step 1: ensure the three aligned bag npz files exist.  Each bag is loaded
# once and cached; subsequent runs of this script are no-ops on this step.
# =============================================================================
ensure_bag () {
    local tag="$1"      # e.g. 231452
    local bag_dir="${REPO_ROOT}/bags/mpc_run_20260517_${tag}"
    local out_npz="${CACHE_DIR}/${tag}.npz"
    if [[ -f "${out_npz}" ]]; then
        echo "  cache hit: ${out_npz}"
        return
    fi
    if [[ ! -d "${bag_dir}" ]]; then
        echo "ERROR: bag dir not found: ${bag_dir}" >&2
        echo "  Upload the bag to \$REPO_ROOT/bags/ or set CACHE_DIR to a pre-loaded location." >&2
        exit 1
    fi
    echo "  loading ${bag_dir} -> ${out_npz}"
    python3 "${REPO_ROOT}/sim/load_bags.py" "${bag_dir}" --out "${out_npz}"
}

echo "Step 1: ensure aligned bag cache"
ensure_bag 215024     # training bag
ensure_bag 232625     # training bag
ensure_bag 231452     # holdout

# =============================================================================
# Step 2: run the validation harness.  Compares three sims on the holdout:
#   - sim_bicycle (the existing toy model in src/autonomous_kart/.../sim_bicycle.py)
#   - id_only     (identified bicycle, no MLP residual)
#   - full        (identified bicycle + MLP residual + Mahalanobis clamp)
# Output: validation.json (raw RMSE per horizon), validation.png (overview
# curves), validation_report.pdf (4-page report with verdict).
# Expected on the canonical model + holdout (231452):
#   dy@1s:  sim_bicycle = 1.558 m
#           id_only     = 0.321 m   (4.85x better)
#           full        = 0.358 m   (4.36x better)
# =============================================================================
echo "Step 2: run validation harness (open-loop multi-step rollout RMSE)"
python3 "${REPO_ROOT}/sim/validate.py" \
    --holdout "${CACHE_DIR}/231452.npz" \
    --params  "${MODEL_DIR}/bicycle_params.json" \
    --noise   "${MODEL_DIR}/noise.json" \
    --mlp     "${MODEL_DIR}/mlp.pt" \
    --norm    "${MODEL_DIR}/norm.json" \
    --clamp   "${MODEL_DIR}/clamp.json" \
    --out-json "${OUT_DIR}/validation.json" \
    --out-png  "${OUT_DIR}/validation.png" \
    --out-pdf  "${OUT_DIR}/validation_report.pdf"

# =============================================================================
# Step 3: distil the headline numbers from validation.json so the SLURM .out
# log contains the answer without needing to inspect the PDF.
# =============================================================================
echo ""
echo "Step 3: headline summary"
python3 - <<'PY'
import json, os
path = os.path.join(os.environ["OUT_DIR"], "validation.json")
data = json.load(open(path))
horizons = data["horizons_ticks"]
dt = data["dt"]
results = data["results"]
print(f"  holdout: {data.get('holdout_tag', '?')}")
print(f"  dt={dt:.4f}s  horizons (s) = {[round(h*dt, 2) for h in horizons]}")
print()
print(f"  {'sim':<14}{'dy@0.1s':>10}{'dy@0.5s':>10}{'dy@1.0s':>10}{'dy@2.0s':>10}")
for name in ("sim_bicycle", "id_only", "full"):
    dy = results[name]["dy"]
    row = [dy[str(h)] for h in horizons]
    print(f"  {name:<14}" + "".join(f"{v:10.3f}" for v in row))
print()
baseline_1s = results["sim_bicycle"]["dy"][str(horizons[2])]
id_1s = results["id_only"]["dy"][str(horizons[2])]
full_1s = results["full"]["dy"][str(horizons[2])]
print(f"  speed-up at 1s horizon:")
print(f"    id_only / sim_bicycle = {baseline_1s / id_1s:.2f}x")
print(f"    full    / sim_bicycle = {baseline_1s / full_1s:.2f}x")
PY

echo ""
echo "Done. Outputs in ${OUT_DIR}"
echo "  - validation.json          (raw RMSE)"
echo "  - validation.png           (overview curves)"
echo "  - validation_report.pdf    (4-page report)"
