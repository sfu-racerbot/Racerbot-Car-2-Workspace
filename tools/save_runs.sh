#!/usr/bin/env bash
# Commit and push the runs saved in data/ (see data/README.md).
#
#   tools/save_runs.sh                 # commit + push everything new in data/
#   tools/save_runs.sh --no-push       # commit only
#   tools/save_runs.sh --skip-idle     # leave runs that never moved uncommitted
#
# Runs where the car never moved, and stubs replaced at once by another
# recorder (both flagged in auto_record.yaml), are listed before committing.
# They are committed unless --skip-idle is given. Nothing is ever deleted.
#
# Commits only data/ -- whatever else is uncommitted in the workspace is left
# alone. Refuses, without committing anything, if:
#   * a recorder or mapping run is still writing (a half-written bag has no
#     metadata.yaml and cannot be replayed), or a finished run is still being
#     summarized into its auto_record.yaml, or
#   * any file in data/ is over GitHub's 100 MB limit, which would make the
#     push fail after the commit is already made.
set -euo pipefail

LIMIT_BYTES=$((100 * 1000 * 1000))
push=1
skip_idle=0
for arg in "$@"; do
    case "$arg" in
        --no-push) push=0 ;;
        --skip-idle) skip_idle=1 ;;
        *) echo "Unknown option: $arg (use --no-push, --skip-idle)" >&2; exit 2 ;;
    esac
done

ws="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ws"

if pgrep -f 'ros2 bag record' >/dev/null || pgrep -f auto_map_race_node >/dev/null; then
    echo "A run is still recording (ros2 bag record / auto_map_race_node)." >&2
    echo "Stop it (Ctrl+C in its terminal) and run this again." >&2
    exit 1
fi
if pgrep -f race_diagnostics.run_activity >/dev/null; then
    echo "A finished run is still being summarized (run_activity). Run this again in a moment." >&2
    exit 1
fi

too_big="$(find data -type f -size +"${LIMIT_BYTES}"c -printf '%s\t%p\n')"
if [[ -n "$too_big" ]]; then
    echo "These files are over GitHub's 100 MB limit and would make the push fail:" >&2
    echo "$too_big" >&2
    echo "Move them out of data/ (or set up Git LFS), then run this again." >&2
    exit 1
fi

# Only runs not committed yet. `moved: false` and `superseded: true` are exactly
# what auto_record.yaml says; a run with no such line is never treated as idle.
idle=()
while IFS= read -r info; do
    if grep -qE '^(  moved: false|superseded: true)$' "$info"; then
        idle+=("$(dirname "$info")")
    fi
done < <(git ls-files --others --exclude-standard -- data | grep '/auto_record\.yaml$' || true)

git add -- data
if (( ${#idle[@]} )); then
    echo "Runs where the car never moved, or stubs:"
    printf '  %s\n' "${idle[@]}"
    if (( skip_idle )); then
        git reset -q -- "${idle[@]}"
        echo "Left uncommitted (--skip-idle). They are still on disk."
    else
        echo "Committing them too. To leave them out: tools/save_runs.sh --skip-idle"
    fi
fi
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
