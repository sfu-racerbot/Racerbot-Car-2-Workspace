"""The league archive: every snapshot from every role, kept forever (plan A8).

    <archive>/<role>/<update:08d>.pt     the policy's state_dict + metadata
    <archive>/index.jsonl                one line per snapshot, append-only

Writes are atomic (temp file, then rename), so a reader -- another role's
learner, or a CPU referee worker -- never sees a half-written snapshot.
There is deliberately no delete in this module (a static test enforces it).
"""
import json
import os
import time
from dataclasses import dataclass
from pathlib import Path

import torch


@dataclass(frozen=True)
class ArchiveEntry:
    entry_id: str          # '<role>/<update:08d>'
    role: str
    update: int
    path: str
    time: float
    meta: dict


class Archive:
    def __init__(self, root: Path):
        self.root = Path(root)
        self.root.mkdir(parents=True, exist_ok=True)
        self.index = self.root / 'index.jsonl'

    def add(self, role: str, update: int, state_dict: dict, meta: dict) -> str:
        entry_id = f'{role}/{update:08d}'
        path = self.root / f'{entry_id}.pt'
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix('.pt.tmp')
        torch.save({'policy': state_dict, 'meta': meta}, tmp)
        os.replace(tmp, path)
        line = json.dumps({'id': entry_id, 'role': role, 'update': update, 'path': str(path),
                           'time': time.time(), 'meta': meta}) + '\n'
        with open(self.index, 'a') as f:                # one write: O_APPEND keeps lines whole
            f.write(line)
        return entry_id

    def entries(self) -> list:
        if not self.index.exists():
            return []
        out, seen = [], set()
        for raw in self.index.read_text().splitlines():
            try:
                d = json.loads(raw)
            except ValueError:
                continue                                # a line still being written
            if d['id'] in seen or not Path(d['path']).exists():
                continue
            seen.add(d['id'])
            out.append(ArchiveEntry(d['id'], d['role'], d['update'], d['path'], d['time'], d.get('meta', {})))
        return out

    def load(self, entry_id: str, map_location='cpu') -> dict:
        return torch.load(self.root / f'{entry_id}.pt', map_location=map_location, weights_only=False)
