# Neural racer training

> **Who this is for:** whoever trains, evaluates or extends the neural racer — on FIR or a CUDA workstation.
> **Read first:** [the neural racer plan](../../docs/superpowers/specs/2026-10-23-neural-racer-plan.md) (Part A for the design, Part B for the phases), and [slurm/README.md](slurm/README.md) for submitting jobs.
> **You'll be able to:** run every training stage, evaluate snapshots in the referee, pick a release, and export it for the car.

This folder trains the policy that `src/neural_racer` runs on the car. Nothing here is built by `colcon`, and nothing here can move the car.

## Highlights

- **A batched simulator that matches the referee.** `gpu_sim/` runs about 16k races at once in PyTorch. Over 10 s of driving it matches the calibrated CPU simulator (`FidelityPlant`, `car` profile) to within 2 cm and 0.5°. Its dynamics match gym's own functions to 1e-9.
- **The car's code is the spec.** The batched observation and safety wrapper import every constant from `src/neural_racer` and are tested against its NumPy code to 1e-6. The exported network is tested against the car's NumPy forward pass to 1e-5.
- **A league that never forgets.** Matchmaking uses PFSP over every archived snapshot, with a 10% uniform floor, plus a scripted pool that fades to a 3% floor. Snapshots are never deleted, and a static test enforces that.
- **Generalization by construction.** Tracks come from two procedural generators (smooth loops and rounded polygons with real straights and hairpins), replayed by prioritized level replay. 20 held-out procedural tracks and the 4 real course maps are refused by the training pool.
- **Releases chosen in the referee, never in training.** The exported NumPy policy races the real `gap_follow` mirror and `pure_pursuit` on held-out tracks. Matchups stop early once they're statistically settled, the ranking uses Nash averaging, and a worst-case veto applies.

## Layout

| Path | What it is |
|---|---|
| `neural_racer_train/gpu_sim/` | the batched simulator: dynamics, actuation, LiDAR, IMU, contact, race and reward, observation, safety |
| `neural_racer_train/tracks.py`, `track_pool.py`, `plr.py` | track generators and checks, the GPU track pool, prioritized level replay |
| `neural_racer_train/model.py`, `ppo.py` | the student, teacher and critic; PPO |
| `neural_racer_train/teacher.py`, `distill.py`, `league_learner.py`, `benchmark.py` | the Slurm stages (trainer contract in `slurm/README.md`) |
| `neural_racer_train/archive.py`, `matchmaking.py`, `opponents.py`, `scripted.py` | the league |
| `neural_racer_train/referee.py`, `eval_worker.py`, `selection.py` | evaluation and release selection |
| `neural_racer_train/export.py`, `heldout.py` | export for the car; the held-out set |
| `heldout.yaml` | the held-out tracks and their digests. Never train on these |
| `check_mutations.py` | proves the key tests fail against a deliberate break |
| `tests/` | the test suite |

## Running the tests

PyTorch is needed. On the Jetson it isn't installed system-wide, so install it privately into a folder that's ignored by git. This is for tests only; nothing on the car uses it.

**Terminal 1** — from the workspace root:

```bash
PYTHONPATH=~/racerbot-ws/.sim/pip python3 -m pip install --target .torchenv "torch==2.5.1"
PYTHONPATH=.torchenv python3 -m pytest tools/neural_racer/tests/ -v
```

**Working when:** 94 tests pass in about 3.5 minutes on the Jetson's CPU.

**If it doesn't:** the parity tests need the pinned gym. Run `tools/f1tenth_sim/setup.sh` once.

## The stages, in order

| Stage | Command (`python -m neural_racer_train.<stage> --run-dir DIR ...`) | Plan phase |
|---|---|---|
| `benchmark` | `--counts 4096,8192,16384,32768` | 3 |
| `teacher` | `--steps 2e9 --seed N` | 5 |
| `distill` | `--teacher <teacher>/checkpoints/latest.pt` | 6, after Phase 0's IMU gate |
| `league_learner` | `--role main0 --archive <dir> --init-from <distill>/checkpoints/student_final.pt` | 8 |
| `eval_worker` | `--archive <dir> --workers N` (started by the Slurm job on spare cores) | 9 |
| `selection` | `--crossplay <archive>/eval/crossplay.jsonl --panel gap_follow pure_pursuit --out selection_report.json` | 9 |
| `export` | `<checkpoint.pt> <out_dir>` | 6, 9 |

On FIR, use the scripts in `slurm/`. They pass the trainer contract's arguments for you.
