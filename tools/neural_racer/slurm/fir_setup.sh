#!/bin/bash
# One-time setup on a Fir LOGIN node (login nodes have internet; compute nodes do not).
#
#   tools/neural_racer/slurm/fir_setup.sh        # or: tools/neural_racer/slurm/fir_run.sh setup
#
#   1. the CPU referee's pinned gym, into .sim/   (tools/f1tenth_sim/setup.sh)
#   2. every training requirement checked against the Alliance wheelhouse
#   3. a login-node venv, ~/.venvs/neural_racer, for selection and export
#   4. the 20 procedural held-out maps, built once so parallel jobs never race to write them
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "${HERE}/../../.." && pwd)"
# shellcheck source=fir.conf
source "${HERE}/fir.conf"
cd "${REPO}"
step() { echo; echo "== $*"; }

set +u
module purge
module load StdEnv/2023 "${NR_PYTHON_MODULE}"
set -u

step "1/4 the referee simulator (pinned F1TENTH Gym into .sim/)"
tools/f1tenth_sim/setup.sh

step "2/4 checking the wheelhouse"
missing=()
while read -r line; do
    pkg="${line%%#*}"; pkg="${pkg// /}"
    [[ -z "${pkg}" ]] && continue
    name="${pkg%%[=<>]*}"
    if ! avail_wheels "${name}" 2>/dev/null | grep -qi "^${name}\b"; then
        missing+=("${name}")
    fi
done < tools/neural_racer/requirements-fir.txt
if (( ${#missing[@]} )); then
    echo "NOT in the wheelhouse: ${missing[*]}"
    echo "Download them into a folder and pass it to jobs as NR_WHEELS (slurm/README.md)."
    exit 1
fi
echo "all requirements available"

step "3/4 login-node venv at ~/.venvs/neural_racer"
VENV="${HOME}/.venvs/neural_racer"
if [[ ! -x "${VENV}/bin/python" ]]; then
    virtualenv --no-download "${VENV}"
fi
# shellcheck disable=SC1091
source "${VENV}/bin/activate"
pip install --no-index --upgrade pip >/dev/null
pip install --no-index -r tools/neural_racer/requirements-fir.txt
python -c "import torch, numpy; print('torch', torch.__version__, 'numpy', numpy.__version__)"

step "4/4 held-out maps"
PYTHONPATH="${REPO}/tools/neural_racer" python -m neural_racer_train.heldout

step "done"
echo "Next: edit NR_ACCOUNT in ${HERE}/fir.conf (now: ${NR_ACCOUNT}), then"
echo "  tools/neural_racer/slurm/fir_run.sh shakedown"
