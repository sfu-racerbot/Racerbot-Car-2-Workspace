#!/usr/bin/env bash
# Commit and push the runs saved in data/ (see data/README.md).
#
#   tools/save_runs.sh                 # commit + push everything new in data/
#   tools/save_runs.sh --no-push       # commit only
#
# Commits only data/ -- whatever else is uncommitted in the workspace is left
# alone. Refuses, without committing anything, if:
#   * a recorder or mapping run is still writing (a half-written bag has no
#     metadata.yaml and cannot be replayed), or
#   * any file in data/ is over GitHub's 100 MB limit, which would make the
#     push fail after the commit is already made.
set -euo pipefail

LIMIT_BYTES=$((100 * 1000 * 1000))
push=1
[[ "${1:-}" == "--no-push" ]] && push=0

ws="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ws"

if pgrep -f 'ros2 bag record' >/dev/null || pgrep -f auto_map_race_node >/dev/null; then
    echo "A run is still recording (ros2 bag record / auto_map_race_node)." >&2
    echo "Stop it (Ctrl+C in its terminal) and run this again." >&2
    exit 1
fi

too_big="$(find data -type f -size +"${LIMIT_BYTES}"c -printf '%s\t%p\n')"
if [[ -n "$too_big" ]]; then
    echo "These files are over GitHub's 100 MB limit and would make the push fail:" >&2
    echo "$too_big" >&2
    echo "Move them out of data/ (or set up Git LFS), then run this again." >&2
    exit 1
fi

git add -- data
if git diff --cached --quiet -- data; then
    echo "No new runs in data/ -- nothing to commit."
    exit 0
fi

git diff --cached --stat -- data | tail -1
git commit -q -m "data: runs saved $(date +%Y-%m-%d)" -- data
git log --oneline -1

if (( push )); then
    git push
fi
