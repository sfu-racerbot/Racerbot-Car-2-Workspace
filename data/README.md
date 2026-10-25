# data/ — archived run recordings

A copy of car 2's recorded runs, kept in git so they survive the Jetson's disk
and can be looked at from any clone. **Nothing on the car writes here.** The
nodes still write to `~/.ros/`, and this folder is a snapshot you refresh by hand.

| Folder | Copied from | What's in it |
|---|---|---|
| `racerbot_runs/<timestamp>/` | `~/.ros/racerbot_runs/` | `race_diagnostics` recordings: rosbag (`bag/`), `launch.log`, `metadata/` (git state, node parameters). See `docs/run-diagnostics.md` |
| `racerbot_auto/<timestamp>/` | `~/.ros/racerbot_auto/` | maps, pose graphs and racing lines from `auto_map_race` / `map_course` on the real car |
| `racerbot_sim/auto/<timestamp>/` | `~/.ros/racerbot_sim/` | the same output from simulator runs (`racerbot_sim`) |

Folder names are the run's start time, `YYYYMMDD-HHMMSS`.

## Adding new runs

After a session on the car, copy the new runs in and commit them:

```bash
cd ~/racerbot-ws
for d in racerbot_runs racerbot_auto racerbot_sim; do
  rsync -a ~/.ros/$d/ data/$d/
done
git add data && git commit -m "data: add runs from <date>"
git push
```

`rsync -a` only adds and updates files; it never deletes anything here, so a
run deleted from `~/.ros/` (for example with the dashboard's delete panel)
stays archived in this folder.

## Things to know

- **The repo is public.** Everything here is visible to anyone. Run metadata
  includes the full uncommitted `git diff` of the workspace at run time, so
  don't record a run while a secret is sitting in an uncommitted file.
- **GitHub rejects any single file over 100 MB.** Today's largest file is a
  39 MB bag. A long run's `bag_0.mcap` can pass 100 MB; if `git push` refuses
  one, leave that bag out of the commit (or set up Git LFS) rather than forcing it.
- Every file added here stays in git history for good, and every clone
  downloads it, even if it is later deleted.
- `COLCON_IGNORE` stops `colcon build` from searching this folder for packages.
