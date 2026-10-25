"""Many archived policies in one batched pass (plan A8).

Each rollout uses up to M opponent policies. Their weights are stacked with
torch.func.stack_module_state and evaluated with one vmap call, so M
different opponents cost about one forward pass, not M.
"""
import copy

import torch
from torch.func import functional_call, stack_module_state

from neural_racer_train.model import PolicyNet


class StackedOpponents:
    def __init__(self, state_dicts: list, device='cpu', width=None):
        if not state_dicts:
            raise ValueError('StackedOpponents needs at least one policy')
        models = []
        for sd in state_dicts:
            m = PolicyNet() if width is None else PolicyNet(width)
            m.load_state_dict(sd)
            models.append(m.to(device).eval())
        self.M = len(models)
        self.params, self.buffers = stack_module_state(models)
        self.base = copy.deepcopy(models[0]).to('meta')

    @torch.no_grad()
    def act(self, o: dict, h):
        """o: blocks shaped (M, B, ...), h (M, B, 128) -> (a (M, B, 2), h')."""
        def one(params, buffers, ob, hh):
            a, h2, _, _ = functional_call(self.base, (params, buffers), (ob, hh))
            return a, h2
        return torch.vmap(one)(self.params, self.buffers, o, h)
