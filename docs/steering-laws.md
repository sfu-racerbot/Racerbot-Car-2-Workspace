# Steering laws: pure pursuit, Stanley, LQR and MPC

> **Who this is for:** anyone racing with `pure_pursuit` (the map-based race controller) who wants to try a different way of steering along the racing line, or tune one.
> **Read first:** [racing-autonomy.md](racing-autonomy.md#choosing-a-steering-law) — what the racing line is and what the race controller does around the steering.
> **You'll be able to:** pick a steering law at launch, explain what each one does, tune one in the simulator, and judge it fairly against pure pursuit.
> **Time:** 25 minutes to read. A tuning run in the simulator takes about a minute; a full judging run about 40 minutes.

The race controller follows a recorded racing line. Each tick (40 times a second) something has to decide **how far to turn the wheels** to stay on that line. That something is the *steering law*. There are four to choose from, and you pick one when you launch.

## Contents

1. [Highlights](#highlights)
2. [What a steering law is, and what it isn't](#what-a-steering-law-is-and-what-it-isnt)
3. [Safety](#safety)
4. [Choosing a law at launch](#choosing-a-law-at-launch)
5. [The four laws](#the-four-laws) — [pure pursuit](#pure-pursuit), [Stanley](#stanley), [LQR](#lqr), [MPC](#mpc)
6. [Delay compensation and smoothing](#delay-compensation-and-smoothing)
7. [How the laws were tuned](#how-the-laws-were-tuned)
8. [Tuning Stanley and LQR](#tuning-stanley-and-lqr) (and MPC)
9. [Judging a law](#judging-a-law)
10. [Installing osqp for MPC](#installing-osqp-for-mpc)
11. [Troubleshooting](#troubleshooting)

## Highlights

- **Four laws, one switch.** `steering_law:=pure_pursuit | stanley | lqr | mpc` on the launch command. Nothing else in the controller changes: deadman, watchdogs, safety net, overtaking and speed caps all stay the same.
- **All three alternatives beat pure pursuit in the simulator** on 15 held-out runs each (3 circuits × 5 seeds), with no wall contact. None of the margins is big:
  - **Stanley:** 1.6% better cross-track, 2.8% slower laps.
  - **LQR:** 1.6% faster laps, 3.5% worse cross-track.
  - **MPC:** 1.5% faster laps, 0.6% worse cross-track.
- **Tuned and judged on different seeds**, so a law cannot pass by being fitted to the runs it is judged on. The promotion rule was never changed to make a law pass.
- **MPC solves in 1.9 ms typical, 3.5 ms at the 99.9th percentile** on the Jetson, with the full simulated race stack running beside it. The budget is 8 ms. A late solve falls back to pure pursuit for that one tick.
- **Laws only steer.** None of them can raise the speed. Speed always comes from the racing line's speed profile and the controller's caps.
- **Simulator only so far.** No alternative law has driven the real car. Pure pursuit stays the default.

## What a steering law is, and what it isn't

Every tick, the race controller works out where the car is (from localization: the particle filter matching LiDAR to the saved map) and which point on the racing line it is nearest to.

The steering law takes that and returns one number: the steering angle. That is all it does.

Everything around it is shared by all four laws and does not change when you switch:

- the **LB deadman** (no LB held → no driving),
- the **watchdogs** that stop the car if localization goes stale or the car ends up too far off the line,
- the **reactive safety net** that slows or stops for something close in the LiDAR,
- **overtaking**, which steers with pure-pursuit geometry whatever law is selected,
- the **steering slew limit** (how fast the wheels may turn) and the **cornering speed cap**.

So a bad steering law can make the car track the line worse or lap slower. It cannot switch off a safety layer.

## Safety

**Hold LB on the F710 the whole time the car moves. Let go and it stops.** This is true for every steering law; the deadman is in the controller, not the law.

If the deadman were turned off (`enable_deadman: false`), a law steering the wrong way would have nothing stopping the car short of someone catching it. That setting is never allowed on the car ([architecture.md](architecture.md#workspace-policy-the-lb-deadman-button-is-mandatory-for-every-node-that-can-move-the-car)).

**Stanley, LQR and MPC have only been driven in the simulator. The first time any of them runs on the car, do it in this order:**

1. Wheels off the ground, full stack up, LB held: check the wheels turn the right way as you move the car by hand around the line.
2. On the floor at low speed (`max_speed` around 1 m/s, which is live-tunable from the dashboard).
3. Race speed only after that looks right.

The simulator's position noise and delay are estimates of this car's. Stanley and LQR respond strongly to both (see [Delay compensation and smoothing](#delay-compensation-and-smoothing)), so a law that is smooth in the simulator may weave on the car.

## Choosing a law at launch

**Terminal 2** — the race controller, on top of the hardware (terminal 1: `bringup_launch.py`) and localization:

```bash
ros2 launch pure_pursuit pure_pursuit_launch.py steering_law:=stanley
# or the race-day combination (localization + controller):
ros2 launch racerbot_launch race_launch.py steering_law:=lqr
```

Leave `steering_law` off to get the default from `src/pure_pursuit/config/pure_pursuit.yaml`, which is `pure_pursuit`.

**Working when:** the node's startup log has a line like `steering law: stanley StanleyParams(gain=5.0, ...)` with the values it loaded. Once you hold LB, the decision state on the dashboard (or in `/drive_intent`) reads `stanley`, `lqr` or `mpc` while it is steering.

**If it doesn't:**

- An unknown name stops the node at startup with an error naming `steering_law`. That is deliberate: a typo must not quietly fall back to another law.
- `mpc` without the `osqp` package installed also refuses to start. See [Installing osqp](#installing-osqp-for-mpc).

The law and its parameters are **launch-time only**. The dashboard cannot change them while driving, and the node refuses if anything tries. To change a law's gains, edit `pure_pursuit.yaml` and relaunch. (No rebuild needed — the workspace uses `--symlink-install`.)

## The four laws

All four answer the same question. They differ in *what error they look at* and *how far ahead they look*.

Two errors come up again and again:

- **Cross-track error** — how far the car is to the side of the line, in metres.
- **Heading error** — how crooked the car is relative to the line's direction, in radians.

### Pure pursuit

The default, and the baseline every other law has to beat.

Pick a point on the line some distance ahead (the *lookahead*, 0.6–1.5 m depending on speed). Steer along the circle that passes through that point.

It is simple and hard to upset. Looking a metre ahead averages away a lot of position noise and hides the car's reaction delay.

The cost is that it cuts corners a little: it is always aiming at a point that is already round the bend. Its knobs are `min_lookahead`, `max_lookahead` and `lookahead_speed_gain` (these three *are* live-tunable).

### Stanley

Stanley was used on Stanford's DARPA Grand Challenge winner. It looks at the **front axle** instead of a point ahead:

> steering = heading error + a correction that pulls the front axle back to where it should be

The correction is `atan(gain × front-axle error / (speed + softening))`. It is strong at low speed and gentle at high speed. `softening` stops it from blowing up near standstill.

There is no lookahead to tune. That makes Stanley very direct, and also sensitive to noise in the position estimate.

**Two corrections to the textbook version are in this code.** Both come from the kinematic bicycle model (the standard simple model of a car: a rear wheel and a steered front wheel a wheelbase apart). Spec 02's formula had them wrong, and the car hit the wall on every Silverstone run because of it:

1. **No extra curvature term.** Spec 02 added `atan(wheelbase × curvature)` for bends.

   On a bend the heading error is already measured at the front axle's point on the line, where the line has turned further. So it already carries that steering, and adding the term counted it twice.

   In a 1 m-radius corner at 3 m/s that pushed the car about 0.6 m to the inside.
2. **The front axle should sit slightly outside the line in a bend.** Every racing line here is recorded at the *rear* axle (`base_link`). If the rear axle follows a circle of radius R, the front axle runs `√(R² + L²) − R` outside it (L = wheelbase, 0.36 m). Stanley now steers the front axle to that offset, not to zero.

A car with its rear axle exactly on a circle of radius R now gets exactly `atan(L/R)`, the steering that keeps it there. The unit tests check this against the formula worked out by hand (`test_stanley_test6/7` in `src/pure_pursuit/test/test_tracking_laws.py`).

Parameters: `stanley.gain` (5.0), `stanley.softening` (1.0 m/s), plus `stanley.delay_compensation` and `stanley.steering_time_constant` ([below](#delay-compensation-and-smoothing)).

### LQR

LQR stands for *linear-quadratic regulator*. It describes the car's error as two numbers — cross-track error `e` and heading error `θ` — and steers with a fixed rule:

> steering = steering the bend needs − K₁·e − K₂·θ

The two gains K₁ and K₂ are computed, not hand-picked. You say how much you care about each error (`lqr.q_lateral`, `lqr.q_heading`) against how much you mind steering hard (`lqr.r_steer`). LQR then finds the gains that best trade those off for a car moving at a given speed.

The best gains depend on speed, so they are precomputed for a range of speeds (up to `lqr.v_grid_max`, 6 m/s) and looked up each tick.

Like Stanley, it corrects the error *now* and has no lookahead, so the same delay and noise handling applies.

<details>
<summary><b>The model and the maths</b> — click to expand. Skip it unless you're changing <code>LqrGainTable</code>.</summary>

The error model is the kinematic bicycle linearised about the line, discretised at the control period dt:

```
e[k+1]     = e[k] + v·dt·θ[k]
θ[k+1]     = θ[k] + (v·dt/L)·(δ[k] − δ_ff)
```

with δ_ff = atan(L·κ) the steering the line's curvature κ needs.

The gain K(v) solves the discrete algebraic Riccati equation for Q = diag(q_lateral, q_heading), R = r_steer.

The table is built once at startup on a speed grid from 0.1 m/s to `v_grid_max` and interpolated. The tests check K against a direct Riccati iteration and check the closed loop is stable at every grid speed.

</details>

### MPC

MPC stands for *model predictive control*. Every tick it plans the next **second** of steering: 20 moves, 50 ms apart. The plan is chosen to keep the car on the line through the bends ahead.

It plans *knowing* the servo's limits: never past the maximum angle, and never turning faster than the servo can. It applies the first move, throws the rest away, and plans again next tick.

That makes it the only law that sees a bend coming precisely, and the only one that treats the steering rate as a hard limit rather than finding out afterwards.

It is also the only law that can run out of time. Each plan is a small optimisation problem, solved with the `osqp` library. If a solve is late or fails, that is a **miss**:

| What happens | What the car does | Decision state |
|---|---|---|
| One miss | Steers with pure pursuit for that one tick | `mpc_fallback` |
| `mpc.max_consecutive_misses` (3) in a row | **Stops**, and stays stopped until you release and re-press LB | `mpc_miss_latched` |

"Late" means slower than `mpc.deadline_sec` (8 ms). On the Jetson, with the whole simulated race stack running beside it, 4 solves in 20,000 were late, all isolated. None came close to three in a row ([mpc-benchmark-2026-10-19.json](mpc-benchmark-2026-10-19.json)).

Parameters:

- the plan: `mpc.dt`, `mpc.horizon_steps`;
- the weights: `mpc.q_lateral` (4.0), `mpc.q_heading`, `mpc.r_steer`, `mpc.r_rate` (smoothness);
- the miss rules: `mpc.deadline_sec`, `mpc.max_iter`, `mpc.max_consecutive_misses`;
- `mpc.delay_compensation`.

<details>
<summary><b>How the plan is solved</b> — click to expand. Skip it unless you're changing <code>MpcLaw</code>.</summary>

The model is LQR's error model, but time-varying. Each of the N = 20 steps gets the speed and curvature the line has at that point ahead, and the tan δ linearisation is carried along the horizon.

The states are eliminated ("condensed"), leaving a quadratic programme in the N steering moves only:

- the cost is tracking error, steering effort and steering change, plus LQR's Riccati cost at the end, so the short horizon behaves like an infinite one;
- each move has a box constraint for the angle limit, and each difference a constraint for the rate limit. The first move's rate is measured from the last command over one control tick (25 ms), the rest over `mpc.dt`.

OSQP is set up once with a fixed sparsity pattern and updated in place each tick, warm-started from the last plan.

OSQP meets constraints only to about 1e-5. So a solution up to 1e-4 rad past a bound counts as solved (anything further is a miss), and the plan that leaves the law is then moved onto the exact bounds, move by move.

</details>

## Delay compensation and smoothing

These two settings are what made Stanley and LQR usable. Both are off when set to 0.

**Delay compensation.** A steering command does not act the moment it is computed:

- the position estimate is already a LiDAR scan ([glossary](glossary.md#scan)) or two old,
- the command waits a tick to be sent,
- the servo takes about 40 ms to get there.

That adds up to about 0.1 s in the simulator's model of this car (`tools/f1tenth_sim/sim_fidelity/calibration.py`).

Pure pursuit hides this by aiming a metre ahead. Stanley and LQR correct the error *now*. So they are always correcting an error the car has already moved past, and at speed that turns into a weave.

With `delay_compensation` set to d seconds, the law first predicts where the car will be d seconds from now, holding its current speed and steering, and steers from there. MPC uses it too, as the start of its plan.

**Smoothing** (`steering_time_constant`, Stanley and LQR only) is a light low-pass filter on the law's output. Each tick it moves part of the way from the last command toward the new one.

The reason: with the simulator's 2 cm of position noise, Stanley at its tuned gain would jitter about 0.04 rad a tick at 1 m/s, three times pure pursuit.

That jitter is not harmless. The controller's cornering speed cap reads sharp steering as a tight corner and slows the car. So jitter cost lap time directly: up to about 10 s a lap on Spielberg before the filter.

A time constant of 0.05 s (two ticks) removes the jitter, and the delay compensation absorbs the lag it adds.

| Parameter | Stanley | LQR | MPC | Range |
|---|---|---|---|---|
| `<law>.delay_compensation` | 0.10 s | 0.08 s | 0.05 s | 0 to 0.5 s |
| `<law>.steering_time_constant` | 0.05 s | 0.05 s | — | 0 to 0.5 s |

## How the laws were tuned

The defaults in `pure_pursuit.yaml` were chosen in the simulator on the **tuning seeds** (1000–1002) and then judged once on separate **judging seeds** (12345–12349).

| Law | Before tuning (tuning seeds) | What fixed it | Tuned values |
|---|---|---|---|
| Stanley | Hit the wall on every Silverstone run | the formula fix above, delay compensation, smoothing, higher gain | gain 5.0, softening 1.0, delay 0.10 s, smoothing 0.05 s |
| LQR | Weaved, then stopped by the safety net mid-lap | delay compensation, smoothing | delay 0.08 s, smoothing 0.05 s, weights unchanged |
| MPC | 8 of 9 runs passed; cross-track 5% worse | delay compensation, heavier cross-track weight | `q_lateral` 4.0, delay 0.05 s |

Along the way the harness itself was found to model the car with a stale steering limit (0.26 rad, not the 0.349 rad in `pure_pursuit.yaml`). It now reads every limit from that YAML, so the harness and the car cannot disagree.

## Tuning Stanley and LQR

The tuning loop runs in the no-ROS simulator ([simulator.md](simulator.md)). It never touches the car.

**Any terminal**, from `~/racerbot-ws`. Always on the **tuning** seeds:

```bash
python3 tools/f1tenth_sim/law_sweep.py --laws stanley --seed 1000 --repeat-seeds 3 \
    --law-param stanley.gain=4.0 --law-param stanley.delay_compensation=0.12
```

It runs pure pursuit and the candidate on the same 3 circuits × 3 seeds, three runs at a time, and prints:

- a row per law and circuit: runs passed, mean and worst cross-track, lap time, safety-net stops (`stops`) and slow-downs (`avoid`), wall contacts, compute time;
- a verdict line, `PROMOTE` or `REJECT`, with the reasons for a reject.

**Working when:** it finishes in about a minute with a verdict line. Add `--baseline-cache /tmp/pp_1000.json` to reuse pure pursuit's runs between tries.

What to look for:

- **`pass` below 3/3 or any `contact`** — the law is unsafe at these settings. That always rejects.
- **Lap time much slower than pure pursuit, with more `avoid` steps** — usually steering jitter or weave. Raise `delay_compensation` or `steering_time_constant` a little.
- **Cross-track worse** — raise the law's error gain (`stanley.gain`, `lqr.q_lateral`, `mpc.q_lateral`). Too high and lap time suffers.

When a setting looks good, write it into **both** places together:

1. the law's `...Params` dataclass at the top of its section in `src/pure_pursuit/pure_pursuit/tracking_laws.py`, and
2. `src/pure_pursuit/config/pure_pursuit.yaml`.

Then fix the node's `declare_parameter` defaults in `pure_pursuit_node.py` to match. Two tests fail if any of the three disagree (`test_yaml_mirrors_the_param_dataclasses` and `test_node_defaults_are_the_law_defaults`). That is what stops the simulator and the car running different gains.

## Judging a law

Tuning seeds tell you what to try. Only the judging run says whether a law is better, and it is run **once**, after tuning is finished. If you tune until the judging run passes, you have tuned on the judging seeds.

**Any terminal**:

```bash
python3 tools/f1tenth_sim/law_sweep.py --laws stanley lqr mpc --seed 12345 --repeat-seeds 5 \
    --output docs/tracking-law-comparison.json
```

(`run_validation.py --compare` gives the same verdict, one run at a time — see [sim-validation.md](sim-validation.md#comparing-steering-laws-head-to-head).)

**The promotion rule** (`tools/f1tenth_sim/law_compare.py`) needs all three of these:

1. **Safe on every run:** at least one lap, no contact, never more than 0.5 m off the line, no law failures, no MPC latch (the stop-until-LB-is-released state, [glossary](glossary.md#latch)).
2. **Strictly better than pure pursuit** on mean cross-track **or** mean lap time.
3. **No more than 5% worse** on the other one.

A REJECT means the law doesn't ship. It never means the rule gets relaxed.

**Results, 2026-10-19** (Jetson, simulator profile `car`, Spielberg / Silverstone / BrandsHatch × seeds 12345–12349; full rows in [tracking-law-comparison.json](tracking-law-comparison.json)):

| Law | Runs passed | Mean cross-track | Mean lap | Verdict |
|---|---|---|---|---|
| `pure_pursuit` | 15/15 | 0.0690 m | 147.6 s | (baseline) |
| `stanley` | 15/15 | 0.0679 m (−1.6%) | 151.7 s (+2.8%) | **PROMOTE** |
| `lqr` | 15/15 | 0.0714 m (+3.5%) | 145.2 s (−1.6%) | **PROMOTE** |
| `mpc` | 15/15 | 0.0694 m (+0.6%) | 145.3 s (−1.5%) | **PROMOTE** |

No run of any law touched a wall.

PROMOTE means a law **may** be selected on the car. It does not make it the default. Every margin is under 5%, all in a simulator. **Pure pursuit stays the default** until one of these has been tried on the real car in the order under [Safety](#safety).

<details>
<summary><b>Why lap times differ at all, when laws only steer</b> — click to expand. Optional.</summary>

Two parts of the controller turn steering into speed:

- The **cornering speed cap** limits speed by the sharper of the requested and the actual steering. A law that steers more, or jitters, is slowed.
- The simulator harness's **safety net** drops to 1 m/s whenever something is within 0.7 m in the forward ±30° cone. It stops if something is within 0.4 m.

On Spielberg, pure pursuit itself spends about 30% of its ticks in that 1 m/s mode, because the shipped racing line runs close to the walls. A law that points the car's nose at a wall a little more often spends longer there.

That is most of the lap-time difference between the laws. It is also why Silverstone occasionally stalls a run: the harness has no way to restart after a 0.4 m stop. That is a property of the harness and the racing line, and it hits pure pursuit too (6 stop ticks on the judging seeds).

</details>

## Installing osqp for MPC

MPC needs the `osqp` solver in the same Python that ROS uses. On the car it was installed with:

**Any terminal** on the car:

```bash
pip install --user --break-system-packages osqp==1.1.3
```

**Working when:** `python3 -c "import osqp; print(osqp.__version__)"` prints `1.1.3`.

**If it doesn't:** check that `numpy` stayed at 1.26.4 (`python3 -c "import numpy; print(numpy.__version__)"`). osqp 1.1.3 installs alongside the system numpy without upgrading it.

`--break-system-packages` is needed because Ubuntu 24.04 protects the system Python. `--user` keeps the package in your home directory, so it cannot break anything apt installed.

The other three laws never import osqp, so they work without it.

To re-measure the solve time on the Jetson, use `python3 tools/mpc_benchmark/benchmark_mpc.py --steps 20000`. The 2026-10-19 numbers are in [mpc-benchmark-2026-10-19.json](mpc-benchmark-2026-10-19.json).

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| Node exits at startup naming `steering_law` | the name is misspelt | use one of `pure_pursuit`, `stanley`, `lqr`, `mpc` |
| `steering_law 'mpc' needs the 'osqp' package` | osqp not installed for this user's Python | [Installing osqp](#installing-osqp-for-mpc) |
| Decision state `mpc_miss_latched`, car stopped | three late or failed MPC solves in a row | release LB and press it again to re-arm. If it repeats, something else is loading the CPU: check `top`, and consider `pure_pursuit` for this session |
| A dashboard tune of `stanley.gain` (etc.) is refused | law parameters are launch-time only | edit `pure_pursuit.yaml` and relaunch |
| Weaving on the car with Stanley or LQR | more delay or position noise than the simulator models | raise `<law>.delay_compensation` by 0.02 s at a time, or the time constant; or go back to `pure_pursuit` |
| A unit test says the YAML and the dataclass disagree | a gain was changed in one place | change all three places ([Tuning](#tuning-stanley-and-lqr)) |

## See also

- [racing-autonomy.md](racing-autonomy.md) — the whole race pipeline the steering law sits inside.
- [sim-validation.md](sim-validation.md#comparing-steering-laws-head-to-head) — the comparison harness and its flags.
- [simulator.md](simulator.md) — the no-ROS simulator and its car fidelity profile.
- `docs/superpowers/specs/2026-09-27-tracking-0*.md` — the original design specs. They describe Stanley's curvature term the old way; this doc and the code are current.
