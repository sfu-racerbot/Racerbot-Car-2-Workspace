# Part B-1 (racing: overtaking, two-car simulator, team A) implementation plan

> **Who this is for:** the session that builds Part B-1, human or agent, task by task.
> **Read first:** [the Part B outline](2026-10-26-part-b-skeleton.md) items B6 and B9, the [spec](../specs/2026-10-26-slam-and-gap-follow-plan.md), [gap-follow.md](../../gap-follow.md) (its "Race defaults" and "Several cars at once" sections), and the [glossary](../../glossary.md) for [deadman](../../glossary.md#deadman) and [follow-the-gap](../../glossary.md#follow-the-gap).
> **You'll be able to:** make car 1's `gap_follow` follow and pass another car instead of stopping behind it, race two ROS-controlled cars in `racerbot_sim`, and race our `gap_follow` against team A's real `disparity_extender`.
> **Time:** one task at a time. Expect several sessions.

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** win races against team A on car 1's course. Stop stalling behind another car, pass where the course allows it, and prove both head to head: first in the fast simulator, then with the real nodes in `racerbot_sim`, and finally on the floor.

**Architecture:**
- **Overtaking logic** lives in `gap_follow`'s `opponent_tracking.py` (ROS-free) and is wired in `gap_follow_node.py`. It is tuned in `tools/f1tenth_sim/eval_gap_multi.py` (Part A Task 14: per-car `--car-set`, `--swap-starts`, `head_to_head` scoring).
- **`racerbot_sim`** gains a second ROS-controlled car under a `/car2` namespace, and a two-car launch that runs ours against ours, or ours against team A's compiled node.

**Tech stack:** Python 3.12, ROS 2 Jazzy, the F1TENTH Gym via `tools/f1tenth_sim` and `racerbot_sim`, C++ (team A's `racerbot_a`, built only), pytest.

**Starting point:** branch `slam-gapfollow-part-a` (Part A, pushed 2026-10-27). Merge it to `main` first, or branch from it.

## Global Constraints

- **The deadman is untouched.** The LB deadman, its four deny-path tests per node, TTC and emergency clearance do not change. Any new path that can set a speed gets the four deny-path tests with its feature on.
- **ROS tests are sourced and isolated:** `ROS_DOMAIN_ID=79`, `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST`. Node tests publish on `/test_only/drive`, remapped after any params file.
- **Test quality:**
  - Hard rule 1: every new or changed test is seen to fail against a deliberate break.
  - Hard rule 2: never loosen a failing test without stating its reasoning.
  - Every expected value has an oracle (TEST_QUALITY_STANDARDS.md A3).
- **Aggressive tuning** (user, 2026-10-27, memory `aggressive-race-tuning`): ship the fastest setting with **zero contacts and no added stalls** across the simulator protocol below. No cautious defaults.
- **Simulator protocol for any tuning claim:**
  - `eval_gap.py` / `eval_gap_multi.py` on the indoor tracks **and** `car1_course_2026_10_26`;
  - both directions (`--reverse`), at least 2 seeds;
  - report mean lap, contacts, stops, `steering_reversals_per_s`, and for races: wins, finish-gap change, overtakes, contacts and stalls;
  - judge contacts with sim_fidelity's chassis check, **never gym's collision flag** (CLAUDE.md).
- **`racerbot_sim` must never run beside the real drivers.** Its interlock (`hardware_guard.py`) must cover every new car and namespace; never defeat it.
- **Team A's code** (`src/racerbot_a`, a submodule) is read and built, never edited. Car 1's local `COLCON_IGNORE` on `racerbot_a`'s `pure_pursuit` stays, because the package name clashes with ours.
- **Docs** under `docs/` follow the `novice-docs` skill, and `check_docs.py` must add no new findings.
- **Commits** end with the attribution line the session's system prompt gives. Never push to `main`.

## Execution notes (2026-10-28)

Decisions taken before Task R1, with the user:
- **Branch:** `part-b1-racing`, from `main`, which already holds Part A (PR #5). The "merge Part A first" step above is done.
- **Who types:** opencode sessions on the OpenAI subscription. `openai/gpt-6.1-sol` implements R2 and R4 (the code that sets speed) and does the final whole-branch review. `openai/gpt-6.1-terra` implements R1, R3, R5, R6 and R7 and reviews each task. The controlling session writes every brief, re-runs every gate and reads every diff.
- **Where they work:** a worktree at `.oc-worktrees/oc/part-b1-racing`, with its own build. `RB_WS=<worktree> rb <command>` runs there instead of `~/racerbot-ws`; without `RB_WS`, `rb` is unchanged.
- **Team A is used as built, never copied or edited.** Task R8 (a Python port of `disparity_extender`) is **dropped**. Tuning against team A uses their compiled node in `racerbot_sim` (R7).
- **Team A's own deadman and TTC stay on** (their launch defaults), not their README's simulator setting `enable_deadman:=false`. Car 2's `sim_joy_node` forges LB on `/car2/joy` for them, so Review Focus 4 is tested on their side too.
- **Their `safety_node` subscribes to absolute names:** `/scan`, `/joy`, `/drive_raw`, `/ego_racecar/odom`. A `/car2` namespace does not move these, so R7 remaps each one explicitly (`/scan:=/car2/scan` and so on). The static check in R7 step 4 covers both the relative and the absolute names.

## Review Focus

1. **The leader stops dead** in front of the follower: the follower must stop with it, without contact, and **resume** when the leader moves on. Today it never resumes.
2. **A stationary car or obstacle that looks like an opponent:** the "follow" logic must not chase a wall cluster. Keep `opponent_moving_speed`'s gating.
3. **A pass started in a narrow section** (the 1.06 m minimum on car 1's course): it must abort to following without contact.
4. **Car 2's commands must never move car 1,** and the reverse. Each car's deadman (`/joy` vs `/car2/joy`) gates only its own commands.
5. **Team A's node publishing on unexpected topics** (it uses relative `drive` and `scan`): the namespacing must hold; nothing of theirs reaches `/drive` on car 1's graph.

---

### Task R1: A head-to-head score that start order can't decide

**Why:** on narrow tracks the car that starts in front won all 8 of Part A's races (0 overtakes), so "wins" measured start order, not pace.

**Files:** `tools/f1tenth_sim/eval_gap_multi.py`, `tools/f1tenth_sim/tests/test_eval_gap_multi_head_to_head.py`.

**Interfaces:**
- `head_to_head(...)` gains `gap_change_s`: (trailing car's time behind at the finish) minus (time behind at the start), per pair. Negative means the trailing car closed in.
- `start_gap_s` is measured from the progress traces at t=0, as the time the trailing car needs at its own mean speed to reach the leader's start arc.
- A `stalls` count per car: stopped (speed below 0.05 m/s) for more than 1.0 s while the other car moves.

- [ ] **Step 1: Failing tests,** on synthetic traces with closed forms: equal pace gives `gap_change_s` 0; the trailing car 10% faster over 2 laps gives a closed-form negative gap; a 2 s stop while the other moves counts 1 stall; a stop while both are stopped counts 0.
- [ ] **Step 2: Implement, then break it.** Hard rule 1: count stalls while both cars are stopped, and the "both stopped" test must fail.
- [ ] **Step 3: Commit.**

### Task R2: Follow, don't stall

**Why:** in Part A's head-to-head on `car1_course_2026_10_26` (seed 12346), the trailing car stopped 2.7 cm behind the leader and never restarted: `escape_creep_speed` only applies inside the forward reserve, and `gap_follow` never reverses. In the 2026-10-26 races the follow cap was usually 0.00 m/s.

**Files:** `src/gap_follow/gap_follow/opponent_tracking.py` (`follow_speed_cap`, ~line 414), `src/gap_follow/gap_follow/gap_follow_node.py` (`follow_cap`, ~line 1255-1272), `tools/f1tenth_sim/gap_mirror.py`, tests in `src/gap_follow/test/test_opponent_tracking.py` and `test_opponent_awareness_node.py`.

**Behaviour to specify (read the current `follow_speed_cap` first):**
- **The follow cap targets the leader's measured speed** at a time gap (`opponent_follow_time_gap`, today 1.0 s). It is reduced smoothly as the bumper gap closes towards `opponent_follow_min_gap`.
- **It is 0 only when the leader is itself stopped, or the gap is at or below the minimum.**
- **When the leader moves away, the cap rises with the gap.** The car must not stay latched at 0.

**Tests:**
- Closed form: a leader at 2.0 m/s, 2.5 m ahead, with a 1.0 s time gap and 0.5 m minimum gives the documented cap.
- A leader stopped gives 0.
- A leader moving away from 0.4 m gives a cap above 0 within 0.2 s, at the node level, on the published command.
- The four deadman deny paths with `enable_opponent_awareness: true`.
- Mirror parity with awareness on.

**Simulator proof:** run the R1 score with `--car-set 0:max_speed=<lower>` (the slower car in front) and `--swap-starts`, 4 seeds, on `car1_course_2026_10_26` and `indoor_wide`. It must show **0 stalls and 0 contacts**. Record the before and after.

### Task R3: See the other car sooner

**Why:** `opponent_min_hits: 8` scans (0.2 s) delays recognition. On the real races the car was recognised on only about 5% of logged seconds.

**Files:** `gap_follow.yaml` (`opponent_*`), `opponent_tracking.py`.

**Steps:**
- Sweep `opponent_min_hits` {3, 4, 6, 8} and `opponent_moving_speed` {0.4, 0.6} with the R1 protocol.
- Count wall clusters wrongly tracked as moving cars (a new counter in the mirror's telemetry), and recognition delay after the leader enters range.
- Ship the fastest-recognising setting with no increase in false moving tracks and 0 contacts.

### Task R4: Passes sized to the real car

**Why:** today's pass geometry needs about 2.4 m. Car 1's course is 1.52 m median wide (1.30 m at the 10th percentile, 1.06 m at the narrowest), and the team confirmed on 2026-10-27 that passing fits.

**Files:** `opponent_tracking.py` (`inflate_predicted`, `pass_allowed`), `gap_follow.yaml` (`opponent_radius` 0.30, `opponent_passing_margin` 0.15, `opponent_pass_max_steering` 0.10, `opponent_follow_lateral`, `opponent_relevant_lateral`).

**Steps:**
1. Replace the 0.30 m **radius** circle with the car's measured 0.30 × 0.58 m outline, oriented by the track's velocity heading. Use the closed-form geometry tests already in `test_opponent_tracking.py` as the pattern.
2. Sweep `opponent_passing_margin` {0.05, 0.10, 0.15} and `opponent_pass_max_steering` {0.10, 0.20, 0.33} with the R1 protocol, the faster car starting behind.
3. Ship the setting with the most overtakes at **0 contacts and 0 stalls** (aggressive).
4. Review Focus 3: a pass entering the 1.06 m section must fall back to following, with a closed-form test on a synthetic narrow corridor.

### Task R5: A second ROS-controlled car in `racerbot_sim`

**Files:** `src/racerbot_sim/racerbot_sim/sim_bridge.py`, `gym_bridge_node.py`, `sim_joy_node.py` (it already takes `joy_topic`), `hardware_guard.py`, `launch/sim_bringup_launch.py`, `test/`.

**Interfaces:**
- `gym_bridge_node` parameter `controlled_cars` (default 1, today's behaviour).
- For car index i ≥ 2, under namespace `car{i}`:
  - publishes `/car{i}/scan`, `/car{i}/odom`, and TF `car{i}/odom` → `car{i}/base_link` → `car{i}/laser` (same 0.26 m laser offset);
  - subscribes `/car{i}/ackermann_cmd`, with its own `command_timeout_sec` (a silent car stops).
- Controlled cars replace scripted `OpponentPlan` followers; `opponents` still adds scripted cars after them.
- `sim_bringup_launch.py`, when `controlled_cars:=2`, adds:
  - a second `sim_joy_node` with `joy_topic:=/car2/joy`;
  - a second `ackermann_mux` (`/car2/drive`, `/car2/teleop` → `/car2/ackermann_cmd`);
  - the static TF for car 2.

**Tests (pure functions in `sim_bridge.py` where possible):**
- The namespace and topic map for i=2.
- Car 2's command moves only car 2's state, and car 1's only car 1's.
- A silent car 2 stops after its timeout.
- The interlock refuses for both cars with a fake driver present.
- Car 2's scan sees car 1 as an obstacle, with a closed-form range.

### Task R6: The two-car race launch and referee

**Files:** `src/racerbot_sim/launch/sim_race_launch.py` (new), `src/racerbot_sim/racerbot_sim/race_referee_node.py` (new, logic in a ROS-free `referee.py`), tests.

**Interfaces:**
- `ros2 launch racerbot_sim sim_race_launch.py track:=car1_course_2026_10_26 opponent:=ours|team_a car2_params:=<yaml> laps:=3`
- **Car 1:** our `gap_follow` on the root namespace.
- **Car 2:**
  - `ours`: a second `gap_follow_node` under `/car2`, with `car2_params`;
  - `team_a`: see R7.
- **The referee:**
  - subscribes the simulator's ground truth (`/sim/ground_truth_pose` and car 2's equivalent, added in R5);
  - computes laps, finish order, the R1 score and contacts **from the simulator's own geometry**, never from either car;
  - prints a result table and writes `race.json`.
- The run is recorded by `record_run.py` with both namespaces' topics.

**Tests:**
- The referee's ROS-free logic on synthetic pose streams: lap counting, finish order, gap change.
- A static launch check: both control layers' topics resolve into their own namespaces, and no `/drive` publisher exists in `/car2`.

**Done when** ours-vs-ours with Part A's defaults on both cars runs 3 laps, and the ranking agrees with `eval_gap_multi.py` on the same settings.

### Task R7: Team A's `disparity_extender` on car 2

**Files:** launch only (`sim_race_launch.py`); docs. **Team A's files are never edited.**

**Steps:**
1. Build their packages: `colcon build --symlink-install --packages-select disparity_extender ftg safety_node`. Respect the local `COLCON_IGNORE` on their `pure_pursuit`.
2. Read their launch files and README for parameters and topics. Their README's simulator instructions use `enable_deadman:=false enable_ttc:=false`, and their `safety_node` publishes the brake topic their node subscribes.
3. In `sim_race_launch.py opponent:=team_a`, run their `safety_node` and `disparity_extender` under the `/car2` namespace. Remap their relative `scan` → `/car2/scan` and `drive` → `/car2/drive`, and their joy topic to `/car2/joy` if they read one.
4. A static check: no team A node publishes outside `/car2`.
5. Race: Part A defaults against team A's shipped parameters, 3 laps, `--swap-starts` equivalent (two runs, swapped start), car 1's course and `indoor_wide`. Record the results table in `docs/gap-follow.md` under "Racing team A in the simulator", with the caveat that it is simulation.

### Task R8: Optional, a fast copy of team A for tuning

Only if R7 shows that tuning against team A needs many races.
- Port `disparity_extender_node.cpp`'s per-scan algorithm to `tools/f1tenth_sim/team_a_mirror.py` as an `eval_gap_multi` controller type.
- **Before use, a parity test:** feed the compiled node (through a small ROS harness on recorded scans) and the port identical scans, and require commands equal to 1e-6.

### Task R9: Floor test hand-over

Give the team the order: two-car wheels-off check, then a slow two-car floor run with `max_speed:=1.0`, recorded by bringup's `auto_record`, then the race. Show how to read the recording with the R1 score where possible.

## Outcome and lessons (2026-10-29)

**Status:** R1-R7 done, merged to `main` on 2026-10-29 as PR #6 (`7842762`). R8 dropped. R9 handed to the team (outline item B10). The ledger with every ruling is `.oc-runs/part-b1-racing/progress.md` on the dev box.

### What changed from the plan

- **R2 grew three sub-tasks.**
  - **R2b (diagnosis).** It absorbed R6b, the ROS-against-fast-simulator disagreement. Across 616 fast-simulator runs, the stall behind a slower car is a controller-policy deadlock: gap_follow never reverses, so once its stop rules hold it at zero beside the other car, it stays there. With awareness on, the main way in is a track expiring while the cars are close. A deadlock-escape design was rejected, because emergency clearance holds the stuck car and any escape would touch the safety model.
  - **R2c (follow-cap memory).** The follow cap is held for `opponent_follow_memory_sec` (1.0 s) after its track disappears. It gave 32 of 32 runs with 0 stalls and 0 contacts. **User decision:** awareness **on by default for racing**, with the memory, and switchable off (`opponent_awareness:=false`).
  - **R2d (mapping launches).** **User decision:** mapping is always one car, so `map_course`, `auto_map_race` and `autonomous_mapping` default awareness **off**, with an `opponent_awareness` argument.
- **R3:** no recognition setting was faster without adding false movers. `min_hits` 8 and `moving_speed` 0.6 stay. About 49% of passes are recognised (35% on car 1's course).
- **R4:** the outline is a 0.58 × 0.31 m rectangle, with its centre recovered from the near face. Margin 0.15 was the only setting with 0 stalls, so the margins (0.15, 0.10) stay.
- **The final review added an alongside abort:** when the narrow-section rule refuses a pass with the other car alongside, cap the speed at `max(0, v_other − 0.5)` so this car drops back. Before, the cap there was infinite.
- **The abort needed two fixes of its own (2026-10-29):**
  - As first written, it stalled both cars: side by side, each capped below the other, down to zero (car 1's course reversed, 2.5 m/s leader: 0 of 2 finished). Now only the car whose centre is behind yields.
  - A wall segment briefly tracked as a mover beside the leader fired the abort for 2 ticks, and the 1 s follow memory held it, which led to a wedge (33 of 36 runs finished, against 36 of 36 without the abort). Now the memory holds only the follow law, and the abort applies only on scans that see the car.
- **The simulator interlock got a clean-graph window (2026-10-29):** discovery took about 2.45 s against a 3 s hold, and a forged `/joy` occasionally got out beside a live `urg_node`. The graph must now read clean for 3 s after the hold.
  - That made the interlock ROS test's 5 s "no leak" watch end before the simulator could open, so it would have passed without testing anything. The user approved a longer watch under Hard rule 2, and the windows are now computed from the nodes' own defaults (`LEAK_WINDOW_SEC` in `test_controlled_cars.py`). A late-leak break with a 15 s confirmation got past the old hard-coded 10 s window and is caught by the computed one.
- **R7:** team A's `disparity_extender` hits the wall within 3-5 s in `racerbot_sim`. It turns in late at 3.0 m/s and asks for 0.56-0.67 rad, while car 1's servo stops at 0.45-0.51 rad. Car 1 ran those races with awareness off.
- **R8 dropped** (user, 2026-10-28): no copy or edit of team A's code. They run from their own launch files under a `/car2` namespace with remaps, with their own deadman on, fed a forged `/car2/joy`.

### Lessons, carried into the combined B-2/3 plan

1. **Make sure the scenario exercises the behaviour.** The first R2 and R4 acceptance runs used a 2.5 m/s "slow" leader. A 5.0 m/s-capped car only averages about 2.4 m/s on car 1's course, so the follower never caught up, and the results said nothing. Size the speeds to the course, and report a measure that proves the behaviour happened.
2. **Hard rule 1 for deadman tests:** flipping an expectation proves nothing. Monkeypatch the deadman gate in memory in a scratch process, and require every deny test to fail.
3. **Simulator ROS tests:** use `/test_only/...` drive chains, plus a conftest guard that fails fast without an isolated domain.
4. **Two simulators, two steering laws:** `racerbot_sim` has no IMU, so steering damping is inert there. B-2/3's simulated IMU fixes it.
5. **Parallel tracks work:** sibling worktrees on separate ROS domains (78 and 79), merged at the end, with no conflicts.
6. **Dev-box hazards:**
   - Duplicate package names (team A's and team B's `safety_node`) leave a stale CMake cache.
   - `pytest --basetemp` needs an existing parent directory.
   - The box is shared: `--workers 2`, and hold sweeps while a real-time race runs.
7. **Models:** OpenAI, tokenzap and Haiku all hit usage limits during the run. Small checkpoint commits and the ledger made every hand-over between models a resume, not a restart.
8. **Good implementers stop on contradictions** (NEEDS_CONTEXT 9 times). Most of those stops were errors in the controller's own briefs: "and" written for "or", wrong test geometry, a missing file. Check each brief against the plan before sending it.
9. **One controller per branch.** A session resumed in a VS Code terminal and a forked background copy of it both carried the note "B-1 was handed to me", ran the same suites on the same ROS domains for hours, and broke each other's tests. Before starting, run `ListAgents` (or `ps` for `claude`) and check that no other session is working the plan.
