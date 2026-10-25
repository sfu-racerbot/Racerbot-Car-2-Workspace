"""PPO pieces: generalized advantage estimation and the clipped losses (plan A8).

Kept free of any environment so each part can be tested against a hand
computation (tests/test_ppo.py).
"""
from dataclasses import dataclass

import torch


@dataclass(frozen=True)
class PPOConfig:
    gamma: float = 0.995
    lam: float = 0.95
    clip: float = 0.2
    value_coef: float = 0.5
    epochs: int = 4
    minibatches: int = 4
    lr: float = 3e-4
    max_grad_norm: float = 1.0
    aux_path: float = 0.5
    aux_opp: float = 0.5


def gae(rewards, values, next_value, terminal, truncated, gamma: float, lam: float):
    """Advantages and returns over a (T, N) rollout.

    terminal: the episode really ended (no bootstrap). truncated: it was cut
    by the time limit (bootstrap from the value of the state it reached --
    passed per step as the value at t+1, which the caller sets correctly).
    """
    T = rewards.shape[0]
    adv = torch.zeros_like(rewards)
    last = torch.zeros_like(next_value)
    for t in reversed(range(T)):
        v_next = next_value if t == T - 1 else values[t + 1]
        nonterminal = (~terminal[t]).float()
        boot = (~(terminal[t] | truncated[t])).float()
        delta = rewards[t] + gamma * v_next * nonterminal - values[t]
        last = delta + gamma * lam * boot * last
        adv[t] = last
    return adv, adv + values


def policy_loss(logp_new, logp_old, adv, clip: float):
    ratio = torch.exp(logp_new - logp_old)
    return -torch.min(ratio * adv, torch.clamp(ratio, 1 - clip, 1 + clip) * adv).mean()


def value_loss(v_new, returns):
    return 0.5 * ((v_new - returns) ** 2).mean()


def normalize(adv):
    return (adv - adv.mean()) / (adv.std() + 1e-8)
