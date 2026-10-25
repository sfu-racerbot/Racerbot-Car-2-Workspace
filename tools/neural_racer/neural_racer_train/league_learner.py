"""Stage 3: one league role (plan A8, Phase 8).

    python -m neural_racer_train.league_learner --run-dir DIR --archive DIR --role main0 \
        --init-from distill/checkpoints/student_final.pt --resume

Roles: main0, main1_mlp (the MLP ablation), main_exploiter, league_exploiter.
Recurrent PPO on seat 0 with a privileged critic. Opponents for every rollout
come from matchmaking (scripted pool, archive by PFSP with a uniform floor,
or the current self); their policies run as one batched pass. Snapshots go
to the shared archive every --snapshot-every updates and are never deleted.
"""
import sys
import time

import numpy as np
import torch

from neural_racer_train import common, matchmaking
from neural_racer_train.archive import Archive
from neural_racer_train.learn import Runner, SeatAssignment, aux_loss, aux_targets, opponent_actions
from neural_racer_train.model import Critic, PolicyNet, gaussian_logp, squash
from neural_racer_train.opponents import StackedOpponents
from neural_racer_train.ppo import PPOConfig, gae, normalize, policy_loss, value_loss


def parser():
    ap = common.base_parser(__doc__.split('\n')[0])
    ap.add_argument('--init-from', type=str, required=True)
    ap.add_argument('--steps', type=float, default=5e9)
    ap.add_argument('--rollout', type=int, default=32)
    ap.add_argument('--opponents', type=int, default=16)
    ap.add_argument('--snapshot-every', type=int, default=200)
    ap.add_argument('--lr', type=float, default=1e-4)
    ap.add_argument('--max-cars', type=int, default=4)
    ap.add_argument('--pool-slots', type=int, default=4096)
    return ap


def build_sources(refs, archive, policy, rng):
    """Matchmaking refs -> (sources, StackedOpponents or None, names per source)."""
    sources, state_dicts, names = [], [], []
    for ref in refs:
        if ref.kind == 'scripted':
            sources.append(('scripted', ref.name))
            names.append(f'scripted:{ref.name}')
            continue
        sd = (policy.state_dict() if ref.kind == 'self' else archive.load(ref.name)['policy'])
        sd = {k: v for k, v in sd.items()}
        sources.append(('policy', len(state_dicts)))
        state_dicts.append(sd)
        names.append(ref.name)
    stacked = StackedOpponents(state_dicts, device=next(policy.parameters()).device) if state_dicts else None
    return sources, stacked, names


def train(args, stop) -> bool:
    common.seed_everything(args.seed)
    dev = args.device
    gen = torch.Generator(device=dev).manual_seed(args.seed)
    rng = np.random.default_rng(args.seed)
    cfg = PPOConfig(lr=args.lr)
    init = torch.load(args.init_from, map_location=dev, weights_only=False)
    recurrent = 'mlp' not in args.role
    policy = PolicyNet(recurrent=recurrent).to(dev)
    policy.load_state_dict(init['policy'])
    with torch.no_grad():
        policy.log_std.fill_(-1.5)
    critic = Critic().to(dev)
    opt = torch.optim.Adam(list(policy.parameters()) + list(critic.parameters()), lr=cfg.lr)
    archive = Archive(args.archive or (args.run_dir.parent / 'archive'))
    rates = matchmaking.WinRates()
    runner = Runner(args.num_envs, args.max_cars, dev, args.seed, pool_slots=args.pool_slots,
                    cpu_workers=args.cpu_workers)
    sim, E, T = runner.sim, args.num_envs, args.rollout
    seats = SeatAssignment(E, args.max_cars, dev)
    log = common.MetricLog(args.run_dir)
    update, steps, role_steps = 0, 0, 0
    ck = common.load_checkpoint(args.run_dir) if args.resume else None
    if ck:
        policy.load_state_dict(ck['policy']), critic.load_state_dict(ck['critic']), opt.load_state_dict(ck['opt'])
        rates.load(ck['rates'])
        update, steps, role_steps = ck['update'], ck['steps'], ck.get('role_steps', 0)
    h = torch.zeros(E, 128, device=dev)
    start = torch.ones(E, dtype=torch.bool, device=dev)

    def save():
        common.save_checkpoint(args.run_dir, {
            'policy': policy.state_dict(), 'critic': critic.state_dict(), 'opt': opt.state_dict(),
            'rates': rates.state(), 'update': update, 'steps': steps, 'role_steps': role_steps,
            'policy_config': {'width': policy.width, 'recurrent': recurrent}, 'run_name': args.run_dir.name})

    while steps < args.steps:
        if stop.requested:
            save()
            return False
        runner.refresh_tracks()
        entries = [e.entry_id for e in archive.entries()]
        mains = [e for e in entries if e.startswith('main0/') or e.startswith('main1')]
        refs = matchmaking.sample_opponents(args.role, args.opponents, entries, rates, rng, main_ids=mains)
        sources, stacked, names = build_sources(refs, archive, policy, rng)
        seats.assign(list(range(E)), len(sources), gen)        # regroup every rollout; GRU state restarts
        h0 = h.clone()
        buf = {k: [] for k in ('scan', 'ego', 'preview', 'race', 'camera', 'priv', 'a', 'logp', 'v', 'r',
                                'term', 'trunc', 'start', 'alive', 'train', 'ego_t', 'opp_t')}
        t0 = time.time()
        for _ in range(T):
            o, priv = sim.observe(), sim.privileged()
            ego_t, opp_t = aux_targets(sim)
            o0 = {k: v[:, 0] for k, v in o.items()}
            with torch.no_grad():
                h = h * (~start).float().unsqueeze(1)
                mean, h, _, _ = policy(o0, h)
                a = mean + torch.exp(policy.log_std) * torch.randn_like(mean)
                logp, v = gaussian_logp(a, mean, policy.log_std), critic(priv[:, 0])
            act = opponent_actions(sim, o, priv, seats, sources, stacked)
            act[:, 0] = squash(a, sim.v_cap)
            out = sim.step(act)
            for k in ('scan', 'ego', 'preview', 'race', 'camera'):
                buf[k].append(o0[k])
            for k, val in (('priv', priv[:, 0]), ('a', a), ('logp', logp), ('v', v), ('r', out['reward'][:, 0]),
                           ('term', out['terminal'][:, 0]), ('trunc', out['timeout'] & ~out['terminal'][:, 0]),
                           ('start', start.clone()), ('alive', out['alive'][:, 0]), ('train', runner.env_replay.clone()),
                           ('ego_t', ego_t[:, 0]), ('opp_t', opp_t[:, 0])):
                buf[k].append(val)
            ended = out['env_done'].nonzero().flatten()
            if len(ended):
                place = out['place'][ended].cpu().numpy()
                src = seats.source[ended].cpu().numpy()
                act_mask = sim.active[ended].cpu().numpy()
                for row in range(len(ended)):
                    for k in range(1, sim.C):
                        if act_mask[row, k]:
                            rates.record(names[src[row, k]], bool(place[row, 0] < place[row, k]))
            done = runner.end_episodes(out['env_done'])
            start = torch.zeros(E, dtype=torch.bool, device=dev)
            if done:
                start[torch.as_tensor(done, device=dev)] = True
        b = {k: torch.stack(v) for k, v in buf.items()}
        with torch.no_grad():
            next_v = critic(sim.privileged()[:, 0])
        adv, ret = gae(b['r'], b['v'], next_v, b['term'], b['trunc'], cfg.gamma, cfg.lam)
        runner.add_scores(adv)
        w_all = (b['alive'] & b['train'])
        adv_n = torch.zeros_like(adv)
        if w_all.any():
            adv_n[w_all] = normalize(adv[w_all])
        for _ in range(cfg.epochs):
            for mb in torch.randperm(E, device=dev).chunk(cfg.minibatches):
                hh = h0[mb]
                lp_new, vals, lpath, lopp = [], [], 0.0, 0.0
                for t in range(T):
                    o_t = {k: b[k][t, mb] for k in ('scan', 'ego', 'preview', 'race', 'camera')}
                    hh = hh * (~b['start'][t, mb]).float().unsqueeze(1)
                    mean, hh, path, opp = policy(o_t, hh)
                    lp_new.append(gaussian_logp(b['a'][t, mb], mean, policy.log_std))
                    vals.append(critic(b['priv'][t, mb]))
                    w = b['alive'][t, mb]
                    if w.any():
                        p1, p2 = aux_loss(path[w], opp[w], b['ego_t'][t, mb][w], b['opp_t'][t, mb][w])
                        lpath, lopp = lpath + p1, lopp + p2
                lp_new, vals = torch.stack(lp_new), torch.stack(vals)
                w = w_all[:, mb]
                if not w.any():
                    continue
                loss = (policy_loss(lp_new[w], b['logp'][:, mb][w], adv_n[:, mb][w], cfg.clip)
                        + cfg.value_coef * value_loss(vals[w], ret[:, mb][w])
                        + (cfg.aux_path * lpath + cfg.aux_opp * lopp) / T)
                opt.zero_grad()
                loss.backward()
                torch.nn.utils.clip_grad_norm_(list(policy.parameters()) + list(critic.parameters()), cfg.max_grad_norm)
                opt.step()
        update += 1
        steps += T * E
        role_steps += T * E
        if update % args.snapshot_every == 0:
            archive.add(args.role, update, {k: v.detach().cpu() for k, v in policy.state_dict().items()},
                        {'steps': steps, 'recurrent': recurrent})
            if args.role.startswith('main_exploiter'):
                vs_main = [rates.p(m) for m in mains[-1:]] or [0.0]
                if matchmaking.exploiter_should_reset(float(np.mean(vs_main)), role_steps):
                    policy.load_state_dict(init['policy'])
                    role_steps = 0
            elif args.role.startswith('league_exploiter') and rng.random() < 0.25:
                policy.load_state_dict(init['policy'])
                role_steps = 0
        if update % args.log_every == 0:
            log.write(update=update, steps=steps, reward=float(b['r'].mean()),
                      at_fault=float(b['term'].float().mean()), steps_per_s=T * E / (time.time() - t0))
        if update % args.checkpoint_every == 0:
            save()
    save()
    return True


def main(argv=None) -> int:
    return common.run_stage(train, parser().parse_args(argv))


if __name__ == '__main__':
    sys.exit(main())
