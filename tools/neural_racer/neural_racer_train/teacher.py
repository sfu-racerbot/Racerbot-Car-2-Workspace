"""Stage 1: the privileged solo teacher (plan A8, Phase 5).

    python -m neural_racer_train.teacher --run-dir DIR --resume [--steps 2e9]

One car, no opponents. The actor sees privileged state, so it learns raw
car control at the limit fast. PPO over the PLR curriculum; fresh levels
are scored but not trained on.
"""
import sys
import time

import torch

from neural_racer_train import common
from neural_racer_train.model import Critic, TeacherNet, gaussian_logp, squash
from neural_racer_train.learn import Runner
from neural_racer_train.ppo import PPOConfig, gae, normalize, policy_loss, value_loss


def parser():
    ap = common.base_parser(__doc__.split('\n')[0])
    ap.add_argument('--steps', type=float, default=2e9)
    ap.add_argument('--rollout', type=int, default=32)
    ap.add_argument('--pool-slots', type=int, default=4096)
    return ap


def train(args, stop) -> bool:
    common.seed_everything(args.seed)
    dev = args.device
    cfg = PPOConfig()
    runner = Runner(args.num_envs, 1, dev, args.seed, pool_slots=args.pool_slots,
                    cpu_workers=args.cpu_workers, solo=True)
    actor, critic = TeacherNet().to(dev), Critic().to(dev)
    opt = torch.optim.Adam(list(actor.parameters()) + list(critic.parameters()), lr=cfg.lr)
    log = common.MetricLog(args.run_dir)
    update, steps = 0, 0
    ck = common.load_checkpoint(args.run_dir) if args.resume else None
    if ck:
        actor.load_state_dict(ck['actor']), critic.load_state_dict(ck['critic'])
        opt.load_state_dict(ck['opt'])
        runner.levels.load(ck['plr'])
        update, steps = ck['update'], ck['steps']
    sim, E, T = runner.sim, args.num_envs, args.rollout

    def save():
        common.save_checkpoint(args.run_dir, {'actor': actor.state_dict(), 'critic': critic.state_dict(),
                                              'opt': opt.state_dict(), 'plr': runner.levels.state(),
                                              'update': update, 'steps': steps, 'run_name': args.run_dir.name})

    while steps < args.steps:
        if stop.requested:
            save()
            return False
        runner.refresh_tracks()
        buf = {k: [] for k in ('priv', 'a', 'logp', 'v', 'r', 'term', 'trunc', 'train')}
        t0 = time.time()
        for _ in range(T):
            priv = sim.privileged()[:, 0]
            with torch.no_grad():
                mean = actor(priv)
                a = mean + torch.exp(actor.log_std) * torch.randn_like(mean)    # on-device RNG
                logp, v = gaussian_logp(a, mean, actor.log_std), critic(priv)
            out = sim.step(squash(a, sim.v_cap).unsqueeze(1))
            for k, val in (('priv', priv), ('a', a), ('logp', logp), ('v', v), ('r', out['reward'][:, 0]),
                           ('term', out['terminal'][:, 0]), ('trunc', out['timeout'] & ~out['terminal'][:, 0]),
                           ('train', runner.env_replay.clone())):
                buf[k].append(val)
            runner.end_episodes(out['env_done'])
        b = {k: torch.stack(v) for k, v in buf.items()}
        with torch.no_grad():
            next_v = critic(sim.privileged()[:, 0])
        adv, ret = gae(b['r'], b['v'], next_v, b['term'], b['trunc'], cfg.gamma, cfg.lam)
        runner.add_scores(adv)
        mask = b['train'].reshape(-1)
        flat = {k: v.reshape(T * E, *v.shape[2:])[mask] for k, v in b.items() if k != 'train'}
        flat_adv, flat_ret = normalize(adv.reshape(-1)[mask]), ret.reshape(-1)[mask]
        n = flat_adv.shape[0]
        for _ in range(cfg.epochs):
            perm = torch.randperm(n, device=dev)
            for mb in perm.chunk(cfg.minibatches):
                mean = actor(flat['priv'][mb])
                loss = policy_loss(gaussian_logp(flat['a'][mb], mean, actor.log_std), flat['logp'][mb],
                                   flat_adv[mb], cfg.clip) + cfg.value_coef * value_loss(critic(flat['priv'][mb]), flat_ret[mb])
                opt.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(list(actor.parameters()) + list(critic.parameters()), cfg.max_grad_norm)
                opt.step()
        update += 1
        steps += T * E
        if update % args.log_every == 0:
            log.write(update=update, steps=steps, reward=float(b['r'].mean()), crash=float(b['term'].float().mean()),
                      steps_per_s=T * E / (time.time() - t0))
        if update % args.checkpoint_every == 0:
            save()
    save()
    return True


def main(argv=None) -> int:
    return common.run_stage(train, parser().parse_args(argv))


if __name__ == '__main__':
    sys.exit(main())
