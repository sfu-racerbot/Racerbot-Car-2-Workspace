"""eval_worker: score archive snapshots in the referee, on the job's spare CPU cores.

    python -m neural_racer_train.eval_worker --archive DIR --workers N [--once]

For every snapshot without a result, against each panel member, it races
2-car matches on held-out tracks (grid order alternating) in batches of 10
until the sequential test settles the matchup or 200 races pass (plan A8,
Phase 9). A panel member a role has beaten above 95% for 10 snapshots in a
row is re-checked only every 10th snapshot; one failed re-check restores it.

Writes <archive>/eval/<role>/<update>.json and appends cross-play lines
({"a", "b", "wins_a", "races", "fault_a", "fault_b"}) to
<archive>/eval/crossplay.jsonl for selection.py.
"""
import argparse
import json
import multiprocessing as mp
import signal
import sys
import time
from pathlib import Path

from neural_racer_train import common  # noqa: F401  (paths)
from neural_racer_train import heldout, selection
from neural_racer_train.archive import Archive

PANEL = ('gap_follow', 'pure_pursuit')


def _race_job(job):
    """One referee race, in a worker process: (snapshot weights, panel, track, seed, neural first?)."""
    import torch
    from neural_racer_train import referee
    from neural_racer_train.export import weights_of
    from neural_racer_train.model import PolicyNet
    entry_path, recurrent, panel, track, seed, neural_first, v_cap = job
    policy = PolicyNet(recurrent=recurrent)
    policy.load_state_dict(torch.load(entry_path, map_location='cpu', weights_only=False)['policy'])
    w = weights_of(policy)
    drivers = {'neural': referee.neural_factory(w, v_cap=v_cap),
               'panel': referee.gap_factory() if panel == 'gap_follow' else referee.pursuit_factory()}
    if not neural_first:
        drivers = {'panel': drivers['panel'], 'neural': drivers['neural']}
    try:
        r = referee.race(track, drivers, seed=seed, laps=1, timeout_s=150.0)
    except RuntimeError as exc:
        return {'error': str(exc)}
    i_n, i_p = r.names.index('neural'), r.names.index('panel')
    return {'won': r.order.index('neural') < r.order.index('panel'),
            'fault_n': int(r.at_fault[i_n]), 'fault_p': int(r.at_fault[i_p]), 'track': track, 'seed': seed}


def evaluate(entry, pool, tracks, retire: dict, snapshot_index: int, v_cap: float) -> dict:
    out = {'entry': entry.entry_id, 'matchups': {}}
    for panel in PANEL:
        key = f'{entry.role}|{panel}'
        ret = selection.Retirement(**retire.get(key, {}))
        if not ret.should_check(snapshot_index):
            out['matchups'][panel] = {'skipped': 'retired'}
            continue
        wins = races = fn = fp = 0
        k = 0
        verdict = 'continue'
        while verdict == 'continue':
            jobs = []
            for _ in range(selection.BATCH):
                jobs.append((entry.path, entry.meta.get('recurrent', True), panel, tracks[k % len(tracks)],
                             k, k % 2 == 0, v_cap))
                k += 1
            for res in pool.map(_race_job, jobs):
                if 'error' in res:
                    continue
                wins += int(res['won'])
                races += 1
                fn += res['fault_n']
                fp += res['fault_p']
            verdict = selection.sequential_verdict(wins, races)
        ret.record(snapshot_index, wins / max(races, 1))
        retire[key] = vars(ret)
        out['matchups'][panel] = {'wins': wins, 'races': races, 'verdict': verdict,
                                  'fault_neural': fn, 'fault_panel': fp}
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--archive', type=Path, required=True)
    ap.add_argument('--workers', type=int, default=4)
    ap.add_argument('--v-cap', type=float, default=2.5)
    ap.add_argument('--once', action='store_true', help='evaluate what is there, then exit')
    ap.add_argument('--poll-sec', type=float, default=60.0)
    args = ap.parse_args(argv)
    archive = Archive(args.archive)
    eval_dir = args.archive / 'eval'
    eval_dir.mkdir(parents=True, exist_ok=True)
    retire_path = eval_dir / 'retirement.json'
    retire = json.loads(retire_path.read_text()) if retire_path.exists() else {}
    tracks = heldout.names()
    heldout.ensure_maps()
    stop = {'now': False}
    signal.signal(signal.SIGTERM, lambda *_: stop.update(now=True))
    with mp.get_context('spawn').Pool(max(args.workers, 1)) as pool:
        while not stop['now']:
            todo = [e for e in archive.entries() if not (eval_dir / f'{e.entry_id}.json').exists()]
            per_role = {}
            for e in todo:
                if stop['now']:
                    break
                idx = per_role.setdefault(e.role, len([x for x in archive.entries() if x.role == e.role
                                                       and (eval_dir / f'{x.entry_id}.json').exists()]))
                res = evaluate(e, pool, tracks, retire, idx, args.v_cap)
                per_role[e.role] += 1
                path = eval_dir / f'{e.entry_id}.json'
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_text(json.dumps(res, indent=2))
                with open(eval_dir / 'crossplay.jsonl', 'a') as f:
                    for panel, m in res['matchups'].items():
                        if 'races' in m:
                            f.write(json.dumps({'a': e.entry_id, 'b': panel, 'wins_a': m['wins'], 'races': m['races'],
                                                'fault_a': m['fault_neural'], 'fault_b': m['fault_panel']}) + '\n')
                retire_path.write_text(json.dumps(retire))
            if args.once:
                break
            time.sleep(args.poll_sec)
    return 0


if __name__ == '__main__':
    sys.exit(main())
