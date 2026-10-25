"""The benchmark stage (plan Phase 3, A9): how many environments fill this GPU.

    sbatch --time=00:30:00 fir_train.sbatch benchmark bench-v1

Steps the 4-car simulator with random actions for --seconds at each count in
--counts and writes benchmark.json: steps/s, GPU memory and the smallest
count within 5% of the best throughput (or the largest under 70% memory).
"""
import json
import sys
import time

import torch

from neural_racer_train import common
from neural_racer_train.learn import Runner


def parser():
    ap = common.base_parser(__doc__.split('\n')[0])
    ap.add_argument('--counts', default='4096,8192,16384,32768,65536')
    ap.add_argument('--seconds', type=float, default=60.0)
    ap.add_argument('--max-cars', type=int, default=4)
    return ap


def measure(n: int, args) -> dict:
    dev = args.device
    runner = Runner(n, args.max_cars, dev, args.seed, pool_slots=32, initial_tracks=8)
    sim = runner.sim
    if dev.startswith('cuda'):
        torch.cuda.synchronize()
        torch.cuda.reset_peak_memory_stats()
    ticks, t0 = 0, time.time()
    while time.time() - t0 < args.seconds:
        a = torch.rand(n, args.max_cars, 2, device=dev) * torch.tensor([0.66, 3.0], device=dev) - torch.tensor([0.33, 0.0], device=dev)
        out = sim.step(a)
        sim.observe()
        runner.end_episodes(out['env_done'])
        ticks += 1
    if dev.startswith('cuda'):
        torch.cuda.synchronize()
    dt = time.time() - t0
    mem = torch.cuda.max_memory_allocated() / torch.cuda.get_device_properties(0).total_memory if dev.startswith('cuda') else 0.0
    return {'num_envs': n, 'steps_per_s': ticks * n / dt, 'ticks': ticks, 'memory_fraction': mem}


def train(args, stop) -> bool:
    common.seed_everything(args.seed)
    results = []
    for n in (int(c) for c in args.counts.split(',')):
        if stop.requested:
            break
        try:
            r = measure(n, args)
        except torch.cuda.OutOfMemoryError:
            break
        results.append(r)
        print(json.dumps(r), flush=True)
        if r['memory_fraction'] > 0.70:
            break
        torch.cuda.empty_cache() if args.device.startswith('cuda') else None
    if not results:
        return False
    best = max(r['steps_per_s'] for r in results)
    ok = [r for r in results if r['steps_per_s'] >= 0.95 * best and r['memory_fraction'] <= 0.70]
    chosen = min(ok or results, key=lambda r: r['num_envs'])
    args.run_dir.mkdir(parents=True, exist_ok=True)
    (args.run_dir / 'benchmark.json').write_text(json.dumps({'results': results, 'chosen': chosen}, indent=2))
    return True


def main(argv=None) -> int:
    return common.run_stage(train, parser().parse_args(argv))


if __name__ == '__main__':
    sys.exit(main())
