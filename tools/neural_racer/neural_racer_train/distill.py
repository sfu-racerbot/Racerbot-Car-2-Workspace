"""Stage 2: distil the privileged teacher into the deployable student by DAgger
(plan A8, Phase 6).

    python -m neural_racer_train.distill --run-dir DIR --teacher T.pt --resume

The student drives, and the teacher labels what it would have done in every
state the student reaches; at iteration i the teacher drives a whole episode
with probability 0.5^i. Opponents are scripted only. Run after Phase 0's
gate, so the simulated IMU uses measured noise and delay.
"""
import sys
import time
from collections import deque

import torch

from neural_racer import observation as obs
from neural_racer_train import common, matchmaking
from neural_racer_train.learn import Runner, SeatAssignment, aux_loss, aux_targets, opponent_actions
from neural_racer_train.model import PolicyNet, TeacherNet, squash
from neural_racer_train.ppo import PPOConfig


def parser():
    ap = common.base_parser(__doc__.split('\n')[0])
    ap.add_argument('--teacher', type=str, required=True)
    ap.add_argument('--steps', type=float, default=3e8)
    ap.add_argument('--seq', type=int, default=64)
    ap.add_argument('--iteration-steps', type=float, default=2e7)
    ap.add_argument('--buffer-rollouts', type=int, default=8)
    ap.add_argument('--lr', type=float, default=1e-3)
    ap.add_argument('--max-cars', type=int, default=4)
    ap.add_argument('--pool-slots', type=int, default=4096)
    ap.add_argument('--width', choices=['full', 'half'], default='full')
    return ap


def action_loss(a_student, a_teacher_mean, v_cap):
    """MSE on the squashed, normalised commands -- what the car would do."""
    s = squash(a_student, v_cap)
    t = squash(a_teacher_mean, v_cap)
    return (((s[:, 0] - t[:, 0]) / obs.STEER_LIMIT) ** 2 + ((s[:, 1] - t[:, 1]) / obs.V_SCALE) ** 2).mean()


def train(args, stop) -> bool:
    common.seed_everything(args.seed)
    dev = args.device
    gen = torch.Generator(device=dev).manual_seed(args.seed)
    from neural_racer_train.model import FULL, HALF
    student = PolicyNet(FULL if args.width == 'full' else HALF).to(dev)
    teacher = TeacherNet().to(dev)
    teacher.load_state_dict(torch.load(args.teacher, map_location=dev, weights_only=False)['actor'])
    teacher.eval()
    opt = torch.optim.Adam(student.parameters(), lr=args.lr)
    total_iters = max(int(args.steps / max(args.iteration_steps, 1)), 1)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, T_max=total_iters * 100)
    runner = Runner(args.num_envs, args.max_cars, dev, args.seed, pool_slots=args.pool_slots,
                    cpu_workers=args.cpu_workers)
    sim, E, T = runner.sim, args.num_envs, args.seq
    sources = [('scripted', name) for name in matchmaking.SCRIPTED]
    seats = SeatAssignment(E, args.max_cars, dev)
    seats.assign(list(range(E)), len(sources), gen)
    teacher_drives = torch.zeros(E, dtype=torch.bool, device=dev)
    h = torch.zeros(E, 128, device=dev)
    log = common.MetricLog(args.run_dir)
    steps, update = 0, 0
    ck = common.load_checkpoint(args.run_dir) if args.resume else None
    if ck:
        student.load_state_dict(ck['policy']), opt.load_state_dict(ck['opt'])
        steps, update = ck['steps'], ck['update']
    buffer = deque(maxlen=args.buffer_rollouts)

    def save(name='latest'):
        common.save_checkpoint(args.run_dir, {
            'policy': student.state_dict(), 'opt': opt.state_dict(), 'steps': steps, 'update': update,
            'policy_config': {'width': student.width, 'recurrent': True}, 'run_name': args.run_dir.name}, name)

    while steps < args.steps:
        if stop.requested:
            save()
            return False
        runner.refresh_tracks()
        beta = 0.5 ** int(steps // args.iteration_steps)
        h0 = h.clone()
        seq = {k: [] for k in ('scan', 'ego', 'preview', 'race', 'camera', 'target', 'ego_t', 'opp_t',
                                'start', 'alive', 'v_cap')}
        start = torch.zeros(E, dtype=torch.bool, device=dev)
        t0 = time.time()
        for _ in range(T):
            o, priv = sim.observe(), sim.privileged()
            ego_t, opp_t = aux_targets(sim)
            o0 = {k: v[:, 0] for k, v in o.items()}
            with torch.no_grad():
                a_s, h, _, _ = student(o0, h * (~start).float().unsqueeze(1))
                a_t = teacher(priv[:, 0])
            for k in ('scan', 'ego', 'preview', 'race', 'camera'):
                seq[k].append(o0[k])
            for k, val in (('target', a_t), ('ego_t', ego_t[:, 0]), ('opp_t', opp_t[:, 0]), ('start', start.clone()),
                           ('alive', sim.alive[:, 0].clone()), ('v_cap', sim.v_cap.clone())):
                seq[k].append(val)
            act = opponent_actions(sim, o, priv, seats, sources)
            driver = torch.where(teacher_drives.unsqueeze(1), a_t, a_s)
            act[:, 0] = squash(driver, sim.v_cap)
            out = sim.step(act)
            done = runner.end_episodes(out['env_done'])
            start = torch.zeros(E, dtype=torch.bool, device=dev)
            if done:
                d = torch.as_tensor(done, device=dev)
                start[d] = True
                seats.assign(done, len(sources), gen)
                teacher_drives[d] = torch.rand(len(done), generator=gen, device=gen.device).to(dev) < beta
        buffer.append(({k: torch.stack(v) for k, v in seq.items()}, h0))
        steps += T * E
        # one pass over the buffered rollouts, minibatched by env
        for b, hb in buffer:
            for mb in torch.randperm(E, device=dev).chunk(4):
                hh = hb[mb]
                loss_a = loss_p = loss_o = 0.0
                for t in range(T):
                    o_t = {k: b[k][t, mb] for k in ('scan', 'ego', 'preview', 'race', 'camera')}
                    hh = hh * (~b['start'][t, mb]).float().unsqueeze(1)
                    a, hh, path, opp = student(o_t, hh)
                    w = b['alive'][t, mb]
                    if w.any():
                        loss_a = loss_a + action_loss(a[w], b['target'][t, mb][w], b['v_cap'][t, mb][w])
                        lp, lo = aux_loss(path[w], opp[w], b['ego_t'][t, mb][w], b['opp_t'][t, mb][w])
                        loss_p, loss_o = loss_p + lp, loss_o + lo
                loss = (loss_a + PPOConfig.aux_path * loss_p + PPOConfig.aux_opp * loss_o) / T
                opt.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(student.parameters(), 1.0)
                opt.step()
            sched.step()
        update += 1
        if update % args.log_every == 0:
            log.write(update=update, steps=steps, beta=beta, loss=float(loss),
                      steps_per_s=T * E / (time.time() - t0))
        if update % args.checkpoint_every == 0:
            save()
    save()
    save('student_final')
    return True


def main(argv=None) -> int:
    return common.run_stage(train, parser().parse_args(argv))


if __name__ == '__main__':
    sys.exit(main())
