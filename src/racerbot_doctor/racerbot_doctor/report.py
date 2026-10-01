"""Render DeviceReports as a terminal checklist."""
from .snapshot import Status

_COLOR = {Status.PASS: '32', Status.WARN: '33', Status.FAIL: '1;31',
          Status.INFO: '36', Status.SKIP: '2'}


def _paint(text, status, color):
    return f'\x1b[{_COLOR[status]}m{text}\x1b[0m' if color else text


def render(reports, *, color, notes=()):
    width = max((len(l.name) for r in reports for l in r.layers), default=10)
    lines = []
    for r in reports:
        lines.append(_paint(r.device, r.worst(), color) if color else r.device)
        for group in _group_blocked(r.layers):
            l = group[0]
            tag = _paint(f'{l.status.value:<4}', l.status, color)
            if len(group) > 1:
                names = f'{l.name} .. {group[-1].name}'
                lines.append(f'  {tag}  {names}  ({len(group)} layers) '
                             f'{l.detail}')
                continue
            lines.append(f'  {tag}  {l.name:<{width}}  {l.detail}')
            if l.fix and l.status in (Status.FAIL, Status.WARN, Status.INFO):
                lines.append(f'  {"":<4}  {"":<{width}}  fix: {l.fix}')
        lines.append('')
    for note in notes:
        lines.append(f'note: {note}')
    if notes:
        lines.append('')

    counts = {s: sum(1 for r in reports for l in r.layers if l.status is s)
              for s in (Status.FAIL, Status.WARN)}
    if counts[Status.FAIL]:
        verdict = _paint(f'{counts[Status.FAIL]} FAIL', Status.FAIL, color)
        lines.append(f'{verdict}, {counts[Status.WARN]} WARN -- first broken '
                     'layer per device: ' + '; '.join(
                         _first_fail_per_device(reports)))
    else:
        verdict = _paint('0 FAIL', Status.PASS, color)
        lines.append(f'{verdict}, {counts[Status.WARN]} WARN')
    return '\n'.join(lines) + '\n'


def _group_blocked(layers):
    """Consecutive SKIPs blocked by the same layer -> one group, so a dead
    USB cable is one FAIL line plus one SKIP line, not eight."""
    groups = []
    for l in layers:
        prev = groups[-1][-1] if groups else None
        if (prev is not None and l.status is Status.SKIP
                and prev.status is Status.SKIP
                and l.detail.startswith('blocked by:')
                and l.detail == prev.detail):
            groups[-1].append(l)
        else:
            groups.append([l])
    return groups


def _first_fail_per_device(reports):
    out = []
    for r in reports:
        for l in r.layers:
            if l.status is Status.FAIL:
                out.append(f'{r.device}: {l.name}')
                break
    return out
