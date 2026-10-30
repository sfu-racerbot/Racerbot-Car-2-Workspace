"""What gap_follow's adaptive grip would have chosen on a recorded run.

Replays a recorded car-1 run through gap_follow's own adaptive-grip logic:
gap_logic.SteeringLag, LowPass, AdaptiveGrip and adaptive_grip_step, called in
the order gap_follow_node._lateral_limit calls them, on the recorded /drive
ticks, /imu/data yaw rates and /odom wheel speeds. It reports the lateral
acceleration limit the feature would have set while the car was on the floor.

The gym's tyres never slide, so a recorded run is the only place this feature
can be judged. The tool changes nothing and writes only its cache
(<run>/analysis/grip_replay.npz) and its report (<run>/analysis/grip_replay.txt).

    RB_WS="$(git rev-parse --show-toplevel)" rb env ROS_DOMAIN_ID=74 python3 tools/run_analysis/grip_replay.py <run> [--set name=value ...] [--compare-fixed V]

Reading the bag needs ROS sourced (rosbag2_py). Once the cache exists, the
replay and the summaries need no ROS.

Gyro: /imu/status is not in these bags, so the gyro is trusted on staleness
alone: the latest /imu/data sample must be younger than imu_timeout_sec at the
tick, with a finite yaw rate. Odometry is fresh when the latest /odom is
younger than odom_timeout_sec. Ticks are the /drive receive times, and dt is
the difference of consecutive tick times. A tick with dt <= 0 or dt > 0.5 s is
skipped and counted as a restart. The ratio filter and the steering lag start
afresh. The limit is kept: gap_follow resets it only when the gyro is untrusted
or max_lateral_accel is retuned (AdaptiveGrip.follow_base).

"Judged" means the grip ratio was measured on the tick: gap_logic.grip_ratio is
finite, which needs a quasi-steady steering (SteeringLag), a speed of at least
1 m/s and a steering at least 0.05 rad off centre. Whether the limit moved on
that tick also needs the demand to reach adaptive_grip_judge_fraction of the
limit. AdaptiveGrip.update applies that gate, so a judged tick can leave the
limit where it is.
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[2]
GAP_FOLLOW_SRC = ROOT / 'src' / 'gap_follow'
if str(GAP_FOLLOW_SRC) not in sys.path:
    sys.path.insert(0, str(GAP_FOLLOW_SRC))

from gap_follow import gap_logic  # noqa: E402

GRIP_RATIO_TAU = gap_logic.GRIP_RATIO_TAU_SEC
NODE_CONFIG = GAP_FOLLOW_SRC / 'config' / 'gap_follow.yaml'
RUNS_DIR = Path.home() / 'racerbot-ws' / 'data' / 'racerbot_runs'  # car 2's runs (data/README.md)
CACHE = 'grip_replay.npz'
REPORT = 'grip_replay.txt'

# A tick more than this after the previous one is not the same control stream.
MAX_DT_SEC = 0.5

# Demand bands in m/s^2, lower edge inclusive and upper edge exclusive: the bands
# of the run report's grip table (tools/run_analysis/actuation.py, analyze()).
BAND_NAMES = ('0.3-1.0', '1.0-1.5', '1.5-2.0', '2.0-2.5', '2.5-3.0', '3.0-5.0')
BAND_EDGES = ((0.3, 1.0), (1.0, 1.5), (1.5, 2.0), (2.0, 2.5), (2.5, 3.0), (3.0, 5.0))


# --------------------------------------------------------------- parameters

PARAM_KEYS = (
    'max_lateral_accel', 'adaptive_grip_ceiling', 'adaptive_grip_rise_per_s',
    'adaptive_grip_cut_factor', 'adaptive_grip_hold_ratio',
    'adaptive_grip_slip_ratio', 'adaptive_grip_judge_fraction',
    'adaptive_grip_steering_lag_sec', 'adaptive_grip_max_steering_change',
    'imu_timeout_sec', 'odom_timeout_sec', 'wheelbase',
)


def load_params(overrides=()) -> dict:
    """The adaptive-grip and timeout parameters, from gap_follow.yaml.

    overrides: NAME=VALUE strings (--set), applied on top as floats.
    """
    node = yaml.safe_load(NODE_CONFIG.read_text())['gap_follow_node']
    params = {k: float(node['ros__parameters'][k]) for k in PARAM_KEYS}
    for item in overrides:
        name, _, value = item.partition('=')
        if name not in PARAM_KEYS or not value:
            raise ValueError(f'--set {item}: expected NAME=VALUE with NAME one of '
                             + ', '.join(PARAM_KEYS))
        params[name] = float(value)
    return params


# ------------------------------------------------------------------- reading

MESSAGE_CLASSES = {
    '/drive': 'ackermann_msgs/msg/AckermannDriveStamped',
    '/imu/data': 'sensor_msgs/msg/Imu',
    '/odom': 'nav_msgs/msg/Odometry',
}


def read_replay_bag(bag_dir: Path) -> dict:
    """(receive time, value) rows from the run bag: /drive steering, /imu/data
    yaw rate, /odom wheel speed. Needs ROS sourced (rosbag2_py)."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    bag_dir = Path(bag_dir)
    storage = 'mcap' if any(bag_dir.glob('*.mcap')) else 'sqlite3'
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    present = {t.name for t in reader.get_all_topics_and_types()}
    missing = [topic for topic in MESSAGE_CLASSES if topic not in present]
    if missing:
        raise ValueError(f'{bag_dir}: bag is missing {missing}')
    reader.set_filter(rosbag2_py.StorageFilter(topics=list(MESSAGE_CLASSES)))
    classes = {topic: get_message(text) for topic, text in MESSAGE_CLASSES.items()}
    rows = {'drive': [], 'gyro': [], 'odom': []}
    while reader.has_next():
        topic, raw, t_ns = reader.read_next()
        t = t_ns / 1e9
        msg = deserialize_message(raw, classes[topic])
        if topic == '/drive':
            rows['drive'].append((t, msg.drive.steering_angle))
        elif topic == '/imu/data':
            rows['gyro'].append((t, msg.angular_velocity.z))
        else:
            rows['odom'].append((t, msg.twist.twist.linear.x))
    out = {name: np.array(values, dtype=np.float64).reshape(-1, 2)
           for name, values in rows.items()}
    for name, topic in (('drive', '/drive'), ('gyro', '/imu/data')):
        if len(out[name]) < 2:
            raise ValueError(f'{bag_dir}: {topic}: only {len(out[name])} messages')
    return out


# ------------------------------------------------------------------- replay

def _gyro_gate(gyro_t, gyro_w, t, timeout_sec):
    """(trusted, yaw rate) at tick t: the latest sample at or before t, fresh
    enough, with a finite yaw rate. No sample, a stale one or a non-finite one
    is untrusted (the node's 'no imu', 'imu stale' and 'imu yaw not finite')."""
    i = int(np.searchsorted(gyro_t, t, side='right')) - 1
    if i < 0:
        return False, math.nan
    yaw = float(gyro_w[i])
    if not math.isfinite(yaw) or not (t - gyro_t[i]) < timeout_sec:
        return False, math.nan
    return True, yaw


def _speed_at(odom_t, odom_v, t, timeout_sec):
    """The latest /odom speed at or before t, or NaN when there is none, it is
    stale (at least timeout_sec old) or it is not finite. NaN means no
    judgement: the limit holds (the node's _odom_fresh rule)."""
    i = int(np.searchsorted(odom_t, t, side='right')) - 1
    if i < 0:
        return math.nan
    speed = float(odom_v[i])
    if not math.isfinite(speed) or not (t - odom_t[i]) < timeout_sec:
        return math.nan
    return speed


def replay(ticks, gyro, odom, params) -> dict:
    """Ticks: rows (t, steering) from /drive; gyro: rows (t, yaw_rate);
    odom: rows (t, speed); params: the adaptive-grip and timeout parameters
    (dict). Runs the node's per-tick sequence with gap_logic's own classes.
    Returns arrays per tick: 't', 'limit', 'ratio' (the filtered ratio after
    the tick, NaN when not judged), 'judged' (bool), 'trusted' (bool),
    'speed', 'demand' (speed**2 * |tan(judged steering)| / wheelbase, NaN
    when not judged); plus 'restarts' (count of dt gaps)."""
    ticks = np.asarray(ticks, dtype=np.float64).reshape(-1, 2)
    gyro = np.asarray(gyro, dtype=np.float64).reshape(-1, 2)
    odom = np.asarray(odom, dtype=np.float64).reshape(-1, 2)
    base = float(params['max_lateral_accel'])
    ceiling = max(float(params['adaptive_grip_ceiling']), base)
    lag_sec = float(params['adaptive_grip_steering_lag_sec'])
    max_change = float(params['adaptive_grip_max_steering_change'])
    imu_timeout = float(params['imu_timeout_sec'])
    odom_timeout = float(params['odom_timeout_sec'])
    wheelbase = float(params['wheelbase'])

    # AdaptiveGrip as gap_follow_node.__init__ builds it.
    grip = gap_logic.AdaptiveGrip(
        base=base, ceiling=ceiling,
        rise_per_s=float(params['adaptive_grip_rise_per_s']),
        cut_factor=float(params['adaptive_grip_cut_factor']),
        hold_ratio=float(params['adaptive_grip_hold_ratio']),
        slip_ratio=float(params['adaptive_grip_slip_ratio']),
        judge_fraction=float(params['adaptive_grip_judge_fraction']))
    ratio_filter = gap_logic.LowPass(GRIP_RATIO_TAU)
    lag = gap_logic.SteeringLag(lag_sec)
    gyro_t, gyro_w = gyro[:, 0], gyro[:, 1]
    odom_t, odom_v = odom[:, 0], odom[:, 1]

    rows = {k: [] for k in ('t', 'limit', 'ratio', 'judged', 'trusted', 'speed', 'demand')}
    restarts = 0
    prev_t = None
    for t, steering in ticks:
        t = float(t)
        if prev_t is None or not 0.0 < t - prev_t <= MAX_DT_SEC:
            if prev_t is not None:
                restarts += 1
            # A skipped tick is not recorded. The filters start afresh.
            ratio_filter = gap_logic.LowPass(GRIP_RATIO_TAU)
            lag = gap_logic.SteeringLag(lag_sec)
            prev_t = t
            continue
        dt = t - prev_t
        prev_t = t
        steering = float(steering)

        # gap_follow_node._lateral_limit, in the same order.
        grip.follow_base(base, ceiling)
        judged_steering = lag.step(dt, steering, max_change)
        trusted, yaw = _gyro_gate(gyro_t, gyro_w, t, imu_timeout)
        if trusted:
            speed = _speed_at(odom_t, odom_v, t, odom_timeout)
            limit = gap_logic.adaptive_grip_step(
                grip, ratio_filter, yaw, speed, judged_steering, wheelbase, dt)
            ratio = ratio_filter.value
            ratio = math.nan if ratio is None else float(ratio)
            if math.isfinite(speed) and math.isfinite(judged_steering):
                demand = speed * speed * abs(math.tan(judged_steering)) / wheelbase
            else:
                demand = math.nan
            judged = math.isfinite(ratio) and math.isfinite(demand)
        else:
            # Untrusted gyro: the limit restarts at the base and the ratio
            # smoothing starts again.
            grip.limit = grip.base
            ratio_filter.reset()
            limit, speed, demand, ratio, judged = grip.limit, math.nan, math.nan, math.nan, False
        rows['t'].append(t)
        rows['limit'].append(float(limit))
        rows['ratio'].append(ratio if judged else math.nan)
        rows['judged'].append(judged)
        rows['trusted'].append(trusted)
        rows['speed'].append(speed)
        rows['demand'].append(demand if judged else math.nan)

    result = {k: np.asarray(v, dtype=bool if k in ('judged', 'trusted') else np.float64)
              for k, v in rows.items()}
    result['restarts'] = restarts
    return result


# ---------------------------------------------------------------- summarize

def _cornering(result, base):
    """The judged ticks whose steering asks for at least half the base lateral
    acceleration."""
    return result['judged'] & np.isfinite(result['demand']) & (result['demand'] >= 0.5 * base)


def summarize(result, base) -> dict:
    """{'ticks', 'judged_fraction', 'trusted_fraction', 'limit_max',
    'limit_mean_when_cornering' (demand >= 0.5 * base), 'time_above_base_s',
    'cuts' (ticks where the limit dropped), 'ratio_by_demand_band':
    {band: (n, median ratio)} for bands 0.3-1.0, 1.0-1.5, 1.5-2.0, 2.0-2.5,
    2.5-3.0, 3.0-5.0 m/s^2}."""
    t, limit, ratio = result['t'], result['limit'], result['ratio']
    judged, trusted, demand = result['judged'], result['trusted'], result['demand']
    n = len(t)
    cornering = _cornering(result, base)
    cuts = 0
    time_above = math.nan
    limit_mean = math.nan
    if n:
        cuts = int(np.sum(np.diff(limit) < 0.0))
        if n >= 2:
            time_above = float(np.sum(np.diff(t)[limit[:-1] > base]))
        if np.any(cornering):
            limit_mean = float(np.mean(limit[cornering]))

    bands = {}
    for name, (lo, hi) in zip(BAND_NAMES, BAND_EDGES):
        in_band = judged & np.isfinite(demand) & (demand >= lo) & (demand < hi)
        count = int(np.sum(in_band))
        bands[name] = (count, float(np.median(ratio[in_band])) if count else math.nan)
    return {
        'ticks': n,
        'judged_fraction': float(np.mean(judged)) if n else math.nan,
        'trusted_fraction': float(np.mean(trusted)) if n else math.nan,
        'limit_max': float(np.max(limit)) if n else math.nan,
        'limit_mean_when_cornering': limit_mean,
        'time_above_base_s': time_above,
        'cuts': cuts,
        'ratio_by_demand_band': bands,
    }


# ------------------------------------------------------------------- report

def _share_above(result, base, threshold) -> float:
    """Share of cornering ticks whose limit exceeded threshold."""
    cornering = _cornering(result, base)
    if not np.any(cornering):
        return math.nan
    return float(np.mean(result['limit'][cornering] > threshold))


def report_lines(run_name, result, summary, base, params, compare_fixed) -> list:
    p = params
    bands = summary['ratio_by_demand_band']
    if math.isfinite(summary['limit_mean_when_cornering']):
        cornering = (f"mean when cornering (demand >= {0.5 * base:.2f} m/s^2) "
                     f"{summary['limit_mean_when_cornering']:.2f}")
    else:
        cornering = 'never cornering'
    lines = [
        f'Adaptive grip replay: {run_name}',
        "  gap_follow's own adaptive-grip logic on the recorded /drive ticks",
        '  (gyro trusted on staleness only: /imu/status is not in these bags)',
        f"  base {p['max_lateral_accel']:.2f} ceiling {p['adaptive_grip_ceiling']:.2f} "
        f"rise {p['adaptive_grip_rise_per_s']:.2f} cut {p['adaptive_grip_cut_factor']:.2f} "
        f"hold {p['adaptive_grip_hold_ratio']:.2f} slip {p['adaptive_grip_slip_ratio']:.2f} "
        f"judge {p['adaptive_grip_judge_fraction']:.2f} lag {p['adaptive_grip_steering_lag_sec']:.2f}s",
        f"  ticks {summary['ticks']} (restarts {result['restarts']}: dt <= 0 or > "
        f"{MAX_DT_SEC} s, skipped and the filters restarted)",
        f"  trusted {summary['trusted_fraction'] * 100:.1f}%  "
        f"judged {summary['judged_fraction'] * 100:.1f}%",
        f"  lateral limit: max {summary['limit_max']:.2f} m/s^2, {cornering}",
        f"  above the base {p['max_lateral_accel']:.2f} for {summary['time_above_base_s']:.2f} s "
        "(intervals starting at a tick with limit > base)",
        f"  cuts {summary['cuts']} (ticks where the limit dropped)",
        '  grip ratio by demand band (judged ticks; median filtered ratio;',
        '  lower edge inclusive, upper edge exclusive, as in analysis/report.md):',
    ]
    for name in BAND_NAMES:
        count, median = bands[name]
        if count:
            lines.append(f'    {name} m/s^2: n={count:5d} ratio {median:.3f}')
        else:
            lines.append(f'    {name} m/s^2:   none judged')
    for threshold in compare_fixed:
        share = _share_above(result, base, threshold)
        if math.isfinite(share):
            lines.append(f'  above {threshold:.1f} while cornering: {share * 100:.1f}% '
                         'of cornering ticks')
        else:
            lines.append(f'  above {threshold:.1f} while cornering: no cornering ticks')
    return lines


def resolve_run(name) -> Path:
    run_dir = Path(name).expanduser()
    if run_dir.exists():
        return run_dir
    guess = RUNS_DIR / name
    if guess.exists():
        return guess
    raise FileNotFoundError(f'run {name}: no such directory (tried {run_dir} and {guess})')


def load_replay(run_dir) -> dict:
    """The run's replay arrays ('drive', 'gyro', 'odom'): read from the bag on
    first use, then from <run>/analysis/grip_replay.npz. Delete the cache to
    read the bag again. Point this at a copy of a run, not the original."""
    run_dir = Path(run_dir)
    analysis = run_dir / 'analysis'
    analysis.mkdir(exist_ok=True)
    cache = analysis / CACHE
    if not cache.exists():
        np.savez_compressed(cache, **read_replay_bag(run_dir / 'bag'))
    with np.load(cache) as z:
        return {k: z[k] for k in z.files}


# --------------------------------------------------------------------- main

def main(argv=None) -> int:
    """grip_replay.py RUN [RUN ...] [--set name=value ...] [--compare-fixed V]
    Prints one summary block per run (and writes <run>/analysis/grip_replay.txt).
    --compare-fixed V also reports, per run, the share of cornering ticks
    where the adaptive limit exceeded V."""
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('runs', nargs='+', metavar='RUN',
                    help='run directory, or a name under ~/racerbot-ws/data/racerbot_runs/')
    ap.add_argument('--set', dest='overrides', action='append', default=[],
                    metavar='NAME=VALUE', help='override a parameter from gap_follow.yaml')
    ap.add_argument('--compare-fixed', dest='compare_fixed', type=float, default=None,
                    metavar='V', help='also report the share of cornering ticks with limit > V')
    args = ap.parse_args(argv)
    try:
        params = load_params(args.overrides)
    except ValueError as exc:
        ap.error(str(exc))
    base = params['max_lateral_accel']
    compare = [] if args.compare_fixed is None else [args.compare_fixed]
    for run in args.runs:
        run_dir = resolve_run(run)
        data = load_replay(run_dir)
        result = replay(data['drive'], data['gyro'], data['odom'], params)
        summary = summarize(result, base)
        text = '\n'.join(report_lines(
            run_dir.name, result, summary, base, params, compare)) + '\n'
        (run_dir / 'analysis' / REPORT).write_text(text)
        print(text, end='')
    return 0


if __name__ == '__main__':
    sys.exit(main())
