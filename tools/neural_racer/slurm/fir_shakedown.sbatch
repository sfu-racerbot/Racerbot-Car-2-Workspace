#!/bin/bash
# The first job to run on FIR: does everything work on an H100, at tiny scale?
#
#   tools/neural_racer/slurm/fir_run.sh shakedown
#
#   1. the test suite (ROS-free parts) inside the job's environment
#   2. teacher -> distill -> league -> benchmark on the GPU, a few updates each
#   3. export the league snapshot and run the car's golden check on it
# Prints PASS/FAIL per step and exits non-zero if anything failed. Results:
# $SCRATCH/neural_racer/runs/shakedown-<jobid>/.
#SBATCH --job-name=nr-shakedown
#SBATCH --gpus-per-node=h100:1
#SBATCH --cpus-per-task=12
#SBATCH --mem=64G
#SBATCH --time=01:00:00
#SBATCH --output=%x-%j.out

set -uo pipefail
NR_REPO="${NR_REPO:-${SLURM_SUBMIT_DIR:?submit from the workspace root}}"
R="${SCRATCH:?}/neural_racer/runs/shakedown-${SLURM_JOB_ID:-local}"
mkdir -p "${R}"
set -e
# shellcheck source=fir_env.sh
source "${NR_REPO}/tools/neural_racer/slurm/fir_env.sh"
set +e
cd "${NR_REPO}"
export PYTHONPATH="${NR_REPO}/tools/neural_racer:${PYTHONPATH:-}"
export OMP_NUM_THREADS=4
NR_DEVICE="${NR_DEVICE:-cuda}"     # cpu only for a local dress rehearsal
M=neural_racer_train
results=()

run() {   # run <label> <command...>
    local label="$1"; shift
    echo; echo "== ${label}"; local t0=$SECONDS
    if "$@"; then results+=("PASS  ${label}  ($((SECONDS - t0)) s)")
    else results+=("FAIL  ${label}  ($((SECONDS - t0)) s)"); fi
}

nvidia-smi --query-gpu=name,memory.total --format=csv,noheader
run "tests: training" python -m pytest tools/neural_racer/tests/ -q -p no:cacheprovider
run "tests: neural_racer (ROS-free)" python -m pytest src/neural_racer/test/ -q -p no:cacheprovider \
    --ignore=src/neural_racer/test/test_neural_racer_node.py
run "tests: imu_bridge status (ROS-free)" python -m pytest src/imu_bridge/test/test_imu_health.py -q -p no:cacheprovider
run "gpu: teacher" python -m ${M}.teacher --run-dir "${R}/teacher" --device "${NR_DEVICE}" --num-envs 512 --rollout 16 \
    --steps 32768 --pool-slots 16 --cpu-workers 2 --log-every 1 --checkpoint-every 2
run "gpu: distill" python -m ${M}.distill --run-dir "${R}/distill" --teacher "${R}/teacher/checkpoints/latest.pt" \
    --device "${NR_DEVICE}" --num-envs 256 --seq 16 --steps 16384 --pool-slots 16 --log-every 1 --checkpoint-every 2
run "gpu: league role" python -m ${M}.league_learner --run-dir "${R}/league/main0" --archive "${R}/league/archive" \
    --role main0 --init-from "${R}/distill/checkpoints/student_final.pt" --device "${NR_DEVICE}" --num-envs 256 --rollout 16 \
    --steps 16384 --pool-slots 16 --opponents 4 --snapshot-every 1 --log-every 1 --checkpoint-every 2
run "gpu: benchmark" python -m ${M}.benchmark --run-dir "${R}/bench" --device "${NR_DEVICE}" --counts 1024,4096 --seconds 20
snap="$(ls "${R}"/league/archive/main0/*.pt 2>/dev/null | tail -1)"
run "export + golden check" bash -c "
    python -m ${M}.export '${snap}' '${R}/exports' &&
    python - <<'EOF'
import sys
from pathlib import Path
import neural_racer_train  # noqa: F401 -- puts the car-side packages on sys.path
from neural_racer import model_store
from neural_racer.policy_numpy import PolicyNumpy
d = sorted(Path('${R}/exports').iterdir())[-1]
w, card = model_store.load_dir(d)
err = model_store.golden_error(PolicyNumpy(w, card), d / 'golden.npz')
print(f'golden error {err:.2e} (limit 1e-5)')
sys.exit(0 if err <= 1e-5 else 1)
EOF"

echo; echo "================ shakedown summary (${R})"
printf '%s\n' "${results[@]}"
[[ -f "${R}/bench/benchmark.json" ]] && { echo "benchmark:"; cat "${R}/bench/benchmark.json"; }
for r in "${results[@]}"; do [[ "${r}" == FAIL* ]] && exit 1; done
exit 0
