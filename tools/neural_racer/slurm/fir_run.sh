#!/bin/bash
# Every step of the FIR pipeline, in order. Settings come from fir.conf.
#
#   tools/neural_racer/slurm/fir_run.sh setup              once, on a login node
#   tools/neural_racer/slurm/fir_run.sh shakedown          1 h GPU job: does everything work on an H100?
#   tools/neural_racer/slurm/fir_run.sh benchmark          30 min: how many envs fill the GPU
#   tools/neural_racer/slurm/fir_run.sh teacher [seeds]    Stage 1 (default seeds 0 1 2)
#   tools/neural_racer/slurm/fir_run.sh distill <seed>     Stage 2, from that teacher seed
#   tools/neural_racer/slurm/fir_run.sh league             Stage 3, all four roles
#   tools/neural_racer/slurm/fir_run.sh status             jobs, progress, failures
#   tools/neural_racer/slurm/fir_run.sh select             pick the release from the referee's results
#   tools/neural_racer/slurm/fir_run.sh export <entry>     export an archive entry for the car
#
# NR_DRY_RUN=1 prints every sbatch command instead of submitting it.
# Run from anywhere; jobs always run against this checkout.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/../../.." && pwd)"
# shellcheck source=fir.conf
source "${HERE}/fir.conf"
RUNS="${NR_RUNS_ROOT:-${SCRATCH:-/scratch/${USER}}/neural_racer/runs}"
DRY="${NR_DRY_RUN:-0}"
STEP="${1:-help}"
shift || true

need_account() {
    if [[ "${NR_ACCOUNT}" == "def-CHANGEME" ]]; then
        echo "Set NR_ACCOUNT in ${HERE}/fir.conf first (your def-... allocation)." >&2
        exit 2
    fi
}

sb() {   # sbatch with this repo, printing or submitting; echoes the job id
    if [[ "${DRY}" == "1" ]]; then
        echo "sbatch $*" >&2
        echo "DRY$RANDOM"
    else
        (cd "${REPO}" && sbatch --parsable --account="${NR_ACCOUNT}" "$@")
    fi
}

chain() {  # chain <label> <sbatch args...>: NR_SEGMENTS resumable segments
    local label="$1"; shift
    local prev="" jid
    for (( k = 1; k <= NR_SEGMENTS; k++ )); do
        jid="$(sb ${prev:+--dependency=afterany:${prev}} --time="${NR_TIME}" \
               --export="ALL,NR_REPO=${REPO},NR_PYTHON_MODULE=${NR_PYTHON_MODULE}${NR_EXTRA_EXPORT:-}" "$@")"
        echo "${label} segment ${k}/${NR_SEGMENTS}: job ${jid}"
        prev="${jid}"
    done
}

login_python() {
    local venv="${HOME}/.venvs/neural_racer"
    if [[ ! -x "${venv}/bin/python" ]]; then
        echo "No ${venv}: run '$0 setup' first." >&2
        exit 2
    fi
    set +u
    module load StdEnv/2023 "${NR_PYTHON_MODULE}" >/dev/null 2>&1 || true
    set -u
    PYTHONPATH="${REPO}/tools/neural_racer" "${venv}/bin/python" "$@"
}

case "${STEP}" in
    setup)
        exec "${HERE}/fir_setup.sh" ;;

    shakedown)
        need_account
        jid="$(sb --export="ALL,NR_REPO=${REPO},NR_PYTHON_MODULE=${NR_PYTHON_MODULE}" "${HERE}/fir_shakedown.sbatch")"
        echo "shakedown: job ${jid}. When it ends: less ${REPO}/nr-shakedown-${jid}.out (summary at the bottom)" ;;

    benchmark)
        need_account
        name="bench-$(date +%Y%m%d-%H%M)"
        jid="$(sb --time=00:30:00 --export="ALL,NR_REPO=${REPO},NR_PYTHON_MODULE=${NR_PYTHON_MODULE}" \
               "${HERE}/fir_train.sbatch" benchmark "${name}")"
        echo "benchmark: job ${jid}. Result: ${RUNS}/${name}/solo/benchmark.json -> put chosen.num_envs into fir.conf" ;;

    teacher)
        need_account
        seeds=("$@"); (( ${#seeds[@]} )) || seeds=(0 1 2)
        for s in "${seeds[@]}"; do
            NR_EXTRA_EXPORT=",NR_ROLE=seed${s}" chain "teacher seed ${s}" --job-name="nr-teacher-s${s}" \
                "${HERE}/fir_train.sbatch" teacher "${NR_TEACHER_RUN}" --seed "${s}" --num-envs "${NR_NUM_ENVS}"
        done ;;

    distill)
        need_account
        seed="${1:?usage: fir_run.sh distill <teacher seed>  (pick the best from: fir_run.sh status)}"
        ckpt="${RUNS}/${NR_TEACHER_RUN}/seed${seed}/checkpoints/latest.pt"
        [[ "${DRY}" == "1" || -f "${ckpt}" ]] || { echo "no teacher checkpoint at ${ckpt}" >&2; exit 2; }
        chain "distill" --job-name=nr-distill "${HERE}/fir_train.sbatch" distill "${NR_DISTILL_RUN}" \
            --teacher "${ckpt}" --num-envs "${NR_NUM_ENVS}" ;;

    league)
        need_account
        init="${RUNS}/${NR_DISTILL_RUN}/solo/checkpoints/student_final.pt"
        [[ "${DRY}" == "1" || -f "${init}" ]] || { echo "no student at ${init}: distill first" >&2; exit 2; }
        args=(--account "${NR_ACCOUNT}" --run "${NR_LEAGUE_RUN}" --init "${init}" --segments "${NR_SEGMENTS}"
              --time "${NR_TIME}" --num-envs "${NR_NUM_ENVS}")
        [[ "${DRY}" == "1" ]] && args+=(--dry-run)
        (cd "${REPO}" && "${HERE}/fir_league_submit.sh" "${args[@]}") ;;

    status)
        squeue -u "${USER}" -o "%.10i %.22j %.8T %.10M %.6D %R" 2>/dev/null || true
        echo
        for d in "${RUNS}"/*/*/; do
            [[ -d "${d}" ]] || continue
            [[ "$(basename "${d}")" == archive ]] && continue
            flag=""; [[ -f "${d}DONE" ]] && flag="DONE"; [[ -f "${d}FAILED" ]] && flag="FAILED: $(cat "${d}FAILED")"
            last="$(tail -n1 "${d}metrics.jsonl" 2>/dev/null || true)"
            echo "${d#${RUNS}/}  ${flag}"
            [[ -n "${last}" ]] && echo "    ${last}"
            ls "${d}"gpu_util_summary_*.txt >/dev/null 2>&1 && tail -n1 "$(ls -t "${d}"gpu_util_summary_*.txt | head -1)" | sed 's/^/    /'
            [[ -f "${d}benchmark.json" ]] && login_python -c "import json; print('    chosen num_envs:', json.load(open('${d}benchmark.json'))['chosen']['num_envs'])" 2>/dev/null || true
        done ;;

    select)
        cp="${RUNS}/${NR_LEAGUE_RUN}/archive/eval/crossplay.jsonl"
        [[ -f "${cp}" ]] || { echo "no referee results yet at ${cp}" >&2; exit 2; }
        login_python -m neural_racer_train.selection --crossplay "${cp}" --panel gap_follow pure_pursuit \
            --out "${RUNS}/${NR_LEAGUE_RUN}/selection_report.json" || true
        echo "report: ${RUNS}/${NR_LEAGUE_RUN}/selection_report.json" ;;

    export)
        entry="${1:?usage: fir_run.sh export <archive entry, e.g. main0/00001200>}"
        arch="${RUNS}/${NR_LEAGUE_RUN}/archive"
        out="${SCRATCH:-/scratch/${USER}}/neural_racer/exports"
        login_python - "${arch}" "${entry}" "${out}" "${RUNS}/${NR_LEAGUE_RUN}/selection_report.json" <<'EOF'
import json, shutil, sys
from pathlib import Path
from neural_racer_train import export
arch, entry, out, sel = Path(sys.argv[1]), sys.argv[2], Path(sys.argv[3]), Path(sys.argv[4])
policy, state = export.load_policy(arch / f'{entry}.pt')
d = export.export(policy, out, run_name=arch.parent.name, snapshot_id=entry)
report = json.loads(sel.read_text()) if sel.exists() else {}
chosen = report.get('winner') == entry          # passes only for the selected snapshot
(d / 'selection_report.json').write_text(json.dumps(dict(report, **{'pass': chosen})))
ev = arch / 'eval' / f'{entry}.json'
r = json.loads(ev.read_text()) if ev.exists() else {}
ok = bool(r) and all(m.get('verdict') == 'win' and m.get('fault_neural', 1) == 0
                     for m in r.get('matchups', {}).values() if 'verdict' in m)
(d / 'referee_report.json').write_text(json.dumps(dict(r, **{'pass': ok})))
print(f'exported {d.name} -> {d}')
print(f'selection pass: {chosen}, referee pass: {ok}')
print('copy it to the car (plan, B0 "Getting a model from FIR to the car"), then approve_model there.')
EOF
        ;;

    *)
        sed -n '2,15p' "$0" ;;
esac
