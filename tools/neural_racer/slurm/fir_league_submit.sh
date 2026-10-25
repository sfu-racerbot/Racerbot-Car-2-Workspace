#!/bin/bash
# Submit the Stage 3 league to FIR: one 1-GPU learner per role, each a chain of
# resumable segments. Run from the workspace root on a FIR login node.
#
#   tools/neural_racer/slurm/fir_league_submit.sh --account def-<pi> --run <name> \
#       --init <student checkpoint> [--segments 6] [--time 12:00:00] [--dry-run]
#
# Every role shares <run>/archive, so snapshots from any role become opponents
# for all of them. Segments are chained with afterany: a segment that fails or
# is pre-empted still lets the next one start and resume from the checkpoint.
# A role that has finished writes DONE and its later segments exit in seconds.
set -euo pipefail

ROLES=(main0 main1_mlp main_exploiter league_exploiter)
ACCOUNT="" RUN="" INIT="" SEGMENTS=6 TIME="12:00:00" DRY_RUN=0 NUM_ENVS=""

while (( $# )); do
    case "$1" in
        --account)  ACCOUNT="$2"; shift 2 ;;
        --run)      RUN="$2"; shift 2 ;;
        --init)     INIT="$2"; shift 2 ;;
        --segments) SEGMENTS="$2"; shift 2 ;;
        --time)     TIME="$2"; shift 2 ;;
        --dry-run)  DRY_RUN=1; shift ;;
        --num-envs) NUM_ENVS="$2"; shift 2 ;;
        *) echo "unknown argument: $1" >&2; exit 2 ;;
    esac
done
[[ -n "${ACCOUNT}" ]] || { echo "--account def-<pi> is required" >&2; exit 2; }
[[ -n "${RUN}" ]]     || { echo "--run <name> is required" >&2; exit 2; }
[[ -n "${INIT}" ]]    || { echo "--init <Stage 2 student checkpoint> is required" >&2; exit 2; }
[[ "${SEGMENTS}" =~ ^[1-9][0-9]*$ ]] || { echo "--segments must be a positive integer" >&2; exit 2; }

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/../../.." && pwd)"

for role in "${ROLES[@]}"; do
    prev=""
    for (( segment = 1; segment <= SEGMENTS; segment++ )); do
        cmd=(sbatch --parsable --account "${ACCOUNT}" --time "${TIME}"
             --job-name "nr-${RUN}-${role}"
             --export "ALL,NR_ROLE=${role},NR_REPO=${REPO}")
        [[ -n "${prev}" ]] && cmd+=(--dependency "afterany:${prev}")
        cmd+=("${HERE}/fir_train.sbatch" league_learner "${RUN}" --init-from "${INIT}")
        [[ -n "${NUM_ENVS}" ]] && cmd+=(--num-envs "${NUM_ENVS}")
        if (( DRY_RUN )); then
            echo "${cmd[*]}"
            prev="DRYRUN_${role}_${segment}"
        else
            prev="$("${cmd[@]}")"
            echo "${role} segment ${segment}/${SEGMENTS}: job ${prev}"
        fi
    done
done
