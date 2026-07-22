#!/usr/bin/env bash
set -euo pipefail

GYM_REPOSITORY="https://github.com/f1tenth/f1tenth_gym.git"
GYM_BRANCH="dev-humble"
GYM_COMMIT="bdaec1420c3b0f103858d289866d0d4e2e597c30"

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"
SIM_ROOT="${WORKSPACE}/.sim"
GYM_ROOT="${SIM_ROOT}/f1tenth_gym"
PYTHON_TARGET="${SIM_ROOT}/python"
REQUIREMENTS="${SCRIPT_DIR}/requirements.txt"

mkdir -p "${SIM_ROOT}"

if [[ ! -d "${GYM_ROOT}/.git" ]]; then
    git clone --branch "${GYM_BRANCH}" "${GYM_REPOSITORY}" "${GYM_ROOT}"
fi

actual_remote="$(git -C "${GYM_ROOT}" remote get-url origin)"
if [[ "${actual_remote}" != "${GYM_REPOSITORY}" ]]; then
    echo "error: ${GYM_ROOT} has unexpected origin: ${actual_remote}" >&2
    exit 1
fi

if [[ -n "$(git -C "${GYM_ROOT}" status --porcelain)" ]]; then
    echo "error: ${GYM_ROOT} has local changes; refusing to replace them" >&2
    exit 1
fi

git -C "${GYM_ROOT}" fetch origin "${GYM_COMMIT}"
git -C "${GYM_ROOT}" checkout --detach "${GYM_COMMIT}"

python3 -m pip install --upgrade --target "${PYTHON_TARGET}" -r "${REQUIREMENTS}"

export PYTHONPATH="${PYTHON_TARGET}:${GYM_ROOT}:${WORKSPACE}/src/gap_follow:${WORKSPACE}/src/pure_pursuit${PYTHONPATH:+:${PYTHONPATH}}"
export NUMBA_CACHE_DIR="${SIM_ROOT}/numba-cache"
mkdir -p "${NUMBA_CACHE_DIR}"

python3 - <<'PY'
import gymnasium
import f1tenth_gym
from f1tenth_gym.envs.track import Track

track = Track.from_track_name("Spielberg")
print(
    "F1TENTH Gym ready:",
    f"Gymnasium {gymnasium.__version__},",
    f"Spielberg {track.raceline.length:.1f} m",
)
PY

echo "Run validation with:"
echo "  python3 ${SCRIPT_DIR}/run_validation.py --scenario all"
