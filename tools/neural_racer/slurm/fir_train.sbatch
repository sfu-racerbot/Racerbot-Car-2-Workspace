#!/bin/bash
# One neural-racer learner on one FIR H100: a training stage, a league role, or
# the throughput benchmark. Resumable: run it again (or chain it with
# fir_league_submit.sh) and it continues from the last checkpoint.
#
#   sbatch --account=def-<pi> fir_train.sbatch <stage> <run_name> [trainer args...]
#
#   <stage>     teacher | distill | league_learner | benchmark
#               (a module under tools/neural_racer/neural_racer_train/)
#   <run_name>  directory under $NR_RUNS_ROOT; reuse it to resume
#
# Environment (pass with --export=ALL,VAR=value):
#   NR_ROLE          league role or seed label        (default: solo)
#   NR_REPO          workspace checkout               (default: $SLURM_SUBMIT_DIR)
#   NR_RUNS_ROOT     where runs live                  (default: $SCRATCH/neural_racer/runs)
#   NR_TRACKGEN_WORKERS  track generators the trainer starts (default: 3)
#   NR_EVAL_WORKERS  CPU referee workers, 0 = none    (default: the cores left)
#
# Trainer contract (milestones M1-M6 implement it):
#   --run-dir DIR --archive DIR --role NAME --cpu-workers N --resume
#                               continue from DIR/checkpoints/latest if present;
#                               start N CPU track-generator processes
#   SIGUSR1                     checkpoint, then exit 85
#   exit 0                      the run's whole budget is done
#
# Shares of one Fir GPU node (4x H100, 48 cores, 1 TB): one GPU is 12 cores and
# ~250 GB. Ask for no more than that share -- a larger request is billed as more
# GPUs and waits longer in the queue.
#SBATCH --job-name=nr-train
#SBATCH --gpus-per-node=h100:1
#SBATCH --cpus-per-task=12
#SBATCH --mem=64G
#SBATCH --time=12:00:00
#SBATCH --signal=B:USR1@600
#SBATCH --output=%x-%j.out

set -euo pipefail

STAGE="${1:?usage: fir_train.sbatch <stage> <run_name> [trainer args...]}"
RUN_NAME="${2:?usage: fir_train.sbatch <stage> <run_name> [trainer args...]}"
shift 2

NR_REPO="${NR_REPO:-${SLURM_SUBMIT_DIR:?set NR_REPO or submit from the workspace root}}"
NR_ROLE="${NR_ROLE:-solo}"
NR_RUNS_ROOT="${NR_RUNS_ROOT:-${SCRATCH:?set NR_RUNS_ROOT or SCRATCH}/neural_racer/runs}"
RUN_DIR="${NR_RUNS_ROOT}/${RUN_NAME}/${NR_ROLE}"
ARCHIVE_DIR="${NR_RUNS_ROOT}/${RUN_NAME}/archive"
CPUS="${SLURM_CPUS_PER_TASK:-12}"
JOB="${SLURM_JOB_ID:-local}"

log() { echo "[nr $(date +%H:%M:%S)] $*"; }

# --- Fail in seconds, before spending allocation on a venv build ------------
TRAINER_FILE="${NR_REPO}/tools/neural_racer/neural_racer_train/${STAGE}.py"
if [[ ! -f "${TRAINER_FILE}" ]]; then
    log "no trainer at ${TRAINER_FILE}"
    log "stage '${STAGE}' is not implemented yet -- see the milestones in"
    log "docs/superpowers/specs/2026-10-23-neural-racer-plan.md. Exiting without training."
    exit 3
fi
if [[ -f "${RUN_DIR}/DONE" ]]; then
    log "${RUN_DIR}/DONE exists: this run already finished. Nothing to do."
    exit 0
fi
if [[ -f "${RUN_DIR}/FAILED" ]]; then
    # Without this, every remaining chained segment would re-crash the same way.
    log "${RUN_DIR}/FAILED exists: an earlier segment crashed ($(cat "${RUN_DIR}/FAILED"))."
    log "Fix the cause, delete that file, and resubmit to resume."
    exit 4
fi
mkdir -p "${RUN_DIR}" "${ARCHIVE_DIR}"

# shellcheck source=fir_env.sh
source "${NR_REPO}/tools/neural_racer/slurm/fir_env.sh"
export PYTHONPATH="${NR_REPO}/tools/neural_racer:${NR_REPO}/src/neural_racer:${NR_REPO}/src/gap_follow:${PYTHONPATH:-}"

# --- GPU utilisation log: the evidence for the >80% target -----------------
GPU_LOG="${RUN_DIR}/gpu_util_${JOB}.csv"
MON_PID=""
if command -v nvidia-smi >/dev/null 2>&1; then
    nvidia-smi --query-gpu=timestamp,utilization.gpu,memory.used,memory.total \
        --format=csv,noheader,nounits -l 30 > "${GPU_LOG}" &
    MON_PID=$!
else
    log "nvidia-smi not found: no GPU utilisation log for this job"
fi

# --- CPU referee workers on the cores the GPU trainer leaves idle -----------
# A GPU-resident trainer keeps about one core busy. These workers score
# archive snapshots on held-out tracks in tools/f1tenth_sim, which is work the
# league needs anyway, and keeps the job's CPU efficiency honest.
# Core budget: 2 for the trainer, NR_TRACKGEN_WORKERS track generators that
# the trainer starts itself (they feed its GPU track pool), the rest referees.
TRACKGEN_WORKERS="${NR_TRACKGEN_WORKERS:-3}"
SPARE=$(( CPUS - 2 - TRACKGEN_WORKERS ))
EVAL_WORKERS="${NR_EVAL_WORKERS:-$(( SPARE > 0 ? SPARE : 0 ))}"
EVAL_PID=""
if [[ "${STAGE}" == "benchmark" ]]; then
    EVAL_WORKERS=0   # a benchmark measures the trainer alone
fi
if (( EVAL_WORKERS > 0 )) && [[ -f "${NR_REPO}/tools/neural_racer/neural_racer_train/eval_worker.py" ]]; then
    OMP_NUM_THREADS=1 NUMBA_NUM_THREADS=1 nice -n 10 \
        python -u -m neural_racer_train.eval_worker \
            --archive "${ARCHIVE_DIR}" --workers "${EVAL_WORKERS}" \
            > "${RUN_DIR}/eval_${JOB}.log" 2>&1 &
    EVAL_PID=$!
    log "started ${EVAL_WORKERS} CPU referee workers (pid ${EVAL_PID})"
fi

# --- The trainer, with signal forwarding ------------------------------------
# Bash runs a trap only between commands, so the trainer runs in the
# background and we wait on it; `wait` returns early when USR1 arrives.
log "stage=${STAGE} role=${NR_ROLE} run_dir=${RUN_DIR} job=${JOB}"
OMP_NUM_THREADS=2 python -u -m "neural_racer_train.${STAGE}" \
    --run-dir "${RUN_DIR}" --archive "${ARCHIVE_DIR}" --role "${NR_ROLE}" \
    --cpu-workers "${TRACKGEN_WORKERS}" --resume "$@" &
TRAIN_PID=$!
trap 'log "time limit near: asking trainer to checkpoint"; kill -USR1 "${TRAIN_PID}" 2>/dev/null || true' USR1
trap 'kill -TERM "${TRAIN_PID}" 2>/dev/null || true' TERM

set +e
while true; do
    wait "${TRAIN_PID}"
    status=$?
    # A trap interrupts `wait` with status >128 while the trainer still runs
    # (kill -0 also succeeds on an unreaped zombie, so the exit code is never lost).
    kill -0 "${TRAIN_PID}" 2>/dev/null || break
done
set -e
trap - USR1 TERM

# --- Wind down helpers, then report -----------------------------------------
[[ -n "${EVAL_PID}" ]] && { kill -TERM "${EVAL_PID}" 2>/dev/null || true; wait "${EVAL_PID}" 2>/dev/null || true; }
[[ -n "${MON_PID}" ]] && { kill "${MON_PID}" 2>/dev/null || true; wait "${MON_PID}" 2>/dev/null || true; }

if [[ -s "${GPU_LOG}" ]]; then
    # Column 2 is utilization.gpu (%). Report the whole job -- that is what the
    # Alliance sees -- and the steady state after the first 10 minutes
    # (20 samples) of venv build and torch.compile.
    awk -F', *' -v warm=20 -v job="${JOB}" '
        { n++; s += $2; if (n > warm) { m++; t += $2 } }
        END {
            if (n == 0) exit
            whole = s / n; steady = (m > 0 ? t / m : whole)
            printf "job %s: mean GPU utilisation %.1f%% over %d samples; steady state %.1f%%\n", job, whole, n, steady
            if (whole < 80) printf "WARNING: below the 80%% target -- run the benchmark stage and raise --num-envs\n"
        }' "${GPU_LOG}" | tee "${RUN_DIR}/gpu_util_summary_${JOB}.txt"
fi

case "${status}" in
    0)  touch "${RUN_DIR}/DONE"; log "run complete: ${RUN_DIR}/DONE written" ;;
    85) log "checkpointed at the time limit; the next segment resumes it" ;;
    *)  echo "job ${JOB}, status ${status}" > "${RUN_DIR}/FAILED"
        log "trainer exited with status ${status}; wrote ${RUN_DIR}/FAILED so later segments stop" ;;
esac
[[ -n "${SLURM_JOB_ID:-}" ]] && log "after the job: seff ${SLURM_JOB_ID}  (CPU and memory efficiency)"
# 85 is a planned stop, not a failure: report success so the chain continues.
(( status == 85 )) && exit 0
exit "${status}"
