"""
mapstore.py

The dashboard's half of "get rid of a bad map from the browser": finding the
saved SLAM maps on this machine, deciding which of them a browser is allowed
to delete, and deleting one.

Like protocol.py, tuning.py and proccontrol.py, this file imports no ROS, no
Tornado and no network code, so it is directly unit-testable
(test/test_mapstore.py) against a temporary directory tree, without a running
robot or a browser -- and without ever deleting anything real.

------------------------------------------------------------------------
Why this exists
------------------------------------------------------------------------
Every map this car builds lands in a timestamped run directory written by
auto_map_race_node (`output_directory`, default ~/.ros/racerbot_auto). Each
one is ~30-38MB, dominated by the pose graph. Nothing in this workspace has
ever been able to list them, and nothing has ever been able to remove one:
`rm -rf` by hand was the entire workflow, which means in practice they are
never removed at all.

------------------------------------------------------------------------
The deletion unit is the WHOLE RUN DIRECTORY, and that is deliberate
------------------------------------------------------------------------
A run directory is one artifact, not a bag of files:

  map.pgm + map.yaml          the occupancy grid
  posegraph.posegraph + .data slam_toolbox's graph, the only route back to
                              the grid if the map save raced
  raceline_raw.csv            the recorded lap
  raceline_profiled.csv       what pure_pursuit actually drives
  raceline_optimized.csv      minimum-curvature, reshaped INSIDE this map
  events.jsonl                race_diagnostics, when it was recording

They are mutually dependent. `map.yaml` names its image *relatively*
(`image: map.pgm`), so the pair is only meaningful inside its own directory.
The racing line was optimized against that specific grid and checked for wall
clearance against it. So:

  * Deleting only map.pgm/map.yaml leaves a racing line with nothing left to
    verify it against. That is not hypothetical -- ~/.ros/racerbot_auto/
    20260727-202458 is already in exactly that state from a map_saver race,
    and it is useless.

  * Deleting map.pgm while leaving map.yaml is worse. nav2's map_server then
    fails to configure, the lifecycle manager stalls behind it, and
    particle_filter BLOCKS IN ITS CONSTRUCTOR waiting on /map_server/map
    (see auto_map_race_node.py's comment at _wait_for_particle_filter).

A partial delete is therefore the one outcome worth making impossible. The
caller shows what the directory holds before asking for the confirmation, so
"this also takes the racing line" is on screen rather than discovered later.

------------------------------------------------------------------------
What may never be deleted
------------------------------------------------------------------------
Same shape as proccontrol.PROTECTED: refused before the configured list is
consulted, and putting one in `map_roots` does not enable it -- it is logged
and dropped.

  * Anything inside a git working tree. src/particle_filter/maps/levine.* and
    src/range_libc/maps/* are tracked upstream files, not run output, and a
    browser deleting tracked files would show up as a mystery `git status`.
  * ~/.ros/racerbot_sim/tracks -- generated *inputs* to the simulator
    (racerbot_sim/tracks.py), consumed via `track_directory`. Deleting one
    breaks the sim rather than freeing a stale result.
  * $HOME itself, any ancestor of it, and any path shallower than two
    components. A typo in a config file must not be able to erase a home
    directory.
"""

import hashlib
import os
import re
import shutil

#: Directory names that are simulator inputs rather than run output. Matched
#: on the trailing two path components, so it catches the parameter's default
#: and anyone pointing at the same place by another route.
PROTECTED_TAILS = (
    os.path.join('racerbot_sim', 'tracks'),
)

#: What a run directory can hold, and how to describe it to a person. Order
#: is the order the UI lists them in: the map first, because it is what the
#: person came to delete, then everything that goes with it.
CONTENT_FILES = (
    ('map.yaml', 'map'),
    ('map.pgm', None),               # counted with map.yaml, not listed twice
    ('posegraph.posegraph', 'pose graph'),
    ('posegraph.data', None),
    ('raceline_raw.csv', 'recorded lap'),
    ('raceline_profiled.csv', 'racing line'),
    ('raceline_optimized.csv', 'optimized racing line'),
    ('events.jsonl', 'diagnostics log'),
)

#: Subdirectories a run may legitimately contain. `race_diagnostics` records
#: a rosbag into the run directory when diagnostics are on (see
#: race_diagnostics/summarize_run.py, which reads it back as "rosbag for
#: offline replay"), so without this every diagnosed run would classify as
#: unrecognised and become undeletable -- which is the opposite of useful,
#: since a bag is the largest thing in the directory after the pose graph.
CONTENT_DIRS = (
    ('bag', 'rosbag'),
)

#: Run directories that something in this workspace reads as a *test oracle*.
#: Deleting one does not break the build -- the test skips -- and that is
#: exactly why it is worth saying out loud: a real test quietly becoming a
#: permanent skip is the "occupies the name of coverage it isn't providing"
#: failure TEST_QUALITY_STANDARDS.md is about. Shown before the confirm;
#: never a refusal, because keeping a map forever is not this panel's call.
#: Matched on the trailing <root-name>/<run-id>, so it survives the roots
#: being configured by another route.
CITED_BY = {
    os.path.join('racerbot_auto', '20260727-200103'):
        'src/pure_pursuit/test/test_map_despeckle.py (recorded-map oracle; '
        'it skips if this run is gone)',
}

#: A run directory name, as a whitelist. See is_plain_component() for why
#: this is a whitelist rather than a list of rejected characters.
_PLAIN_NAME_RE = re.compile(r'[A-Za-z0-9][A-Za-z0-9._-]*')

#: Longer than any filesystem accepts as a single component. A name past
#: this is a probe, not a typo, and is refused before it reaches the disk.
_MAX_NAME_LEN = 255

#: Process names that read a saved map or a racing line off disk. Passed as
#: proccontrol.scan()'s allowlist for the "is anything using this run?"
#: check, because scan() only returns processes it was asked about and none
#: of these belong in killable_nodes.
#:
#: Deleting a run out from under a running map_server is the documented way
#: to wedge particle_filter in its constructor waiting on /map_server/map.
MAP_CONSUMERS = (
    'map_server',
    'nav2_map_server',
    'particle_filter',
    'particle_filter_node',
    'pure_pursuit_node',
    'auto_map_race_node',
    'slam_toolbox',
    'async_slam_toolbox_node',
)

#: How much of a .pgm to read looking for its header. The header is a few
#: dozen bytes; this is headroom for comment lines without ever pulling a
#: multi-megabyte image into memory.
_PGM_HEADER_BYTES = 512

#: Beyond this a "width" is a corrupt header, not a map. The largest grid
#: this workspace has produced is 2048x2048.
_PGM_MAX_DIM = 1 << 20

# map.yaml is read with regexes rather than a YAML parser for the same reason
# tuning.py writes it back with regexes: it keeps this package's dependency
# list to what it already has. `^` under re.MULTILINE is what makes a
# commented-out line stay commented out -- '# resolution: 9' has '#' before
# the key, so it cannot match.
_RESOLUTION_RE = re.compile(r'^[ \t]*resolution[ \t]*:[ \t]*([-+0-9.eE]+)', re.M)
_ORIGIN_RE = re.compile(
    r'^[ \t]*origin[ \t]*:[ \t]*\[[ \t]*([-+0-9.eE]+)[ \t]*,'
    r'[ \t]*([-+0-9.eE]+)[ \t]*,[ \t]*([-+0-9.eE]+)[ \t]*\]', re.M)
_IMAGE_RE = re.compile(r'^[ \t]*image[ \t]*:[ \t]*(\S+)', re.M)


# ---------------------------------------------------------------------------
# Roots
# ---------------------------------------------------------------------------

def _real(path):
    """Absolute, symlink-free, ~-expanded. The only form compared anywhere."""
    return os.path.realpath(os.path.abspath(os.path.expanduser(str(path))))


def in_git_tree(path):
    """True if `path` or any ancestor holds a .git entry.

    Checked as an *entry*, not a directory: a git worktree's .git is a file,
    and a worktree of this repo is exactly the case where a browser deleting
    tracked maps would be most confusing.
    """
    current = _real(path)
    while True:
        if os.path.exists(os.path.join(current, '.git')):
            return True
        parent = os.path.dirname(current)
        if parent == current:
            return False
        current = parent


def _too_shallow(path):
    """A path with fewer than two components below the filesystem root.

    '/', '/home', '/mnt' and friends. Nothing this dashboard manages ever
    lives there, and the cost of being wrong is unbounded.
    """
    parts = [p for p in _real(path).split(os.sep) if p]
    return len(parts) < 2


def _is_home_or_above(path):
    home = _real(os.path.expanduser('~'))
    target = _real(path)
    return target == home or _contains(target, home)


def _contains(parent, child):
    """Is `child` strictly inside `parent`? Both are realpath'd first."""
    parent = _real(parent)
    child = _real(child)
    if parent == child:
        return False
    try:
        return os.path.commonpath([parent, child]) == parent
    except ValueError:
        # Different drives / one relative -- cannot be contained.
        return False


def _protected_root_reason(path):
    """Why this root may never be managed from a browser, or '' if it may."""
    real = _real(path)
    if _too_shallow(real):
        return 'it is a top-level system directory'
    if _is_home_or_above(real):
        return 'it is the home directory, or contains it'
    for tail in PROTECTED_TAILS:
        if real.endswith(os.sep + tail) or real == tail:
            return ('it holds generated simulator track inputs, not run '
                    'output -- deleting one breaks the simulator')
    if in_git_tree(real):
        return ('it is inside a git working tree -- those maps are tracked '
                'files, not run output')
    return ''


def sanitize_roots(roots, logger=None):
    """Drop anything from a configured root list that must never be managed.

    Returns the usable roots, realpath'd and de-duplicated, in the order
    given. A rejected entry is a configuration mistake worth shouting about
    rather than silently honouring or silently dropping, so it is logged if a
    logger is supplied -- the same contract as
    proccontrol.sanitize_allowlist().

    A root that does not exist yet is KEPT. ~/.ros/racerbot_auto is created
    by the first auto_map_race run, and this dashboard is started once and
    left running across sessions, so dropping it at startup would mean the
    panel never populated on a fresh machine.
    """
    kept, refused = [], []
    for raw in roots:
        text = str(raw).strip()
        if not text:
            continue
        real = _real(text)
        reason = _protected_root_reason(real)
        if reason:
            refused.append((text, reason))
            continue
        if real not in kept:
            kept.append(real)
    if refused and logger is not None:
        for text, reason in refused:
            logger.warn(
                f'map control: refusing to manage {text} -- {reason}. '
                f'Ignoring it; see mapstore._protected_root_reason.')
    return kept


# ---------------------------------------------------------------------------
# Reading what a saved map says about itself
# ---------------------------------------------------------------------------

def _pgm_tokens(raw, limit):
    """The first `limit` whitespace-separated tokens of a Netpbm header.

    Netpbm allows a '#' comment anywhere whitespace is allowed, running to
    the end of the line, so a header written by one tool and re-saved by
    another is routinely not three tokens on one line.
    """
    tokens = []
    index, size = 0, len(raw)
    while index < size and len(tokens) < limit:
        char = raw[index:index + 1]
        if char in b' \t\r\n\v\f':
            index += 1
            continue
        if char == b'#':
            while index < size and raw[index:index + 1] not in (b'\n', b'\r'):
                index += 1
            continue
        start = index
        while index < size and raw[index:index + 1] not in b' \t\r\n\v\f#':
            index += 1
        tokens.append(raw[start:index])
    return tokens


def read_pgm_size(path):
    """(width, height) in cells from a .pgm header, or None.

    Returns None -- never raises -- for anything that is not a readable
    graymap: a missing file, a PNG, a truncated or commented-out header,
    non-ASCII bytes where a number should be, or dimensions that cannot be a
    map. The size is display-only, so being unable to read it must never be
    worse than not showing it.
    """
    try:
        with open(path, 'rb') as handle:
            raw = handle.read(_PGM_HEADER_BYTES)
    except OSError:
        return None
    tokens = _pgm_tokens(raw, 3)
    if len(tokens) < 3:
        return None
    # P5 is the binary graymap nav2's map_saver writes; P2 is its ASCII
    # equivalent. A PPM (P3/P6) or bitmap (P1/P4) is not a ROS map.
    if tokens[0] not in (b'P5', b'P2'):
        return None
    try:
        width = int(tokens[1].decode('ascii'))
        height = int(tokens[2].decode('ascii'))
    except (ValueError, UnicodeDecodeError):
        return None
    if width <= 0 or height <= 0:
        return None
    if width > _PGM_MAX_DIM or height > _PGM_MAX_DIM:
        return None
    return width, height


def read_map_yaml(path):
    """{'resolution', 'origin', 'image'} from a map.yaml, missing keys None.

    Returns None if the file cannot be read at all. Decoded with
    errors='replace' so a corrupt byte costs one key rather than raising.
    """
    try:
        with open(path, 'r', encoding='utf-8', errors='replace') as handle:
            text = handle.read()
    except OSError:
        return None

    result = {'resolution': None, 'origin': None, 'image': None}

    match = _RESOLUTION_RE.search(text)
    if match:
        try:
            result['resolution'] = float(match.group(1))
        except ValueError:
            pass

    match = _ORIGIN_RE.search(text)
    if match:
        try:
            result['origin'] = [float(match.group(i)) for i in (1, 2, 3)]
        except ValueError:
            pass

    match = _IMAGE_RE.search(text)
    if match:
        result['image'] = match.group(1)

    return result


def directory_size(path):
    """Total bytes of a directory tree, following no symlinks."""
    total = 0
    for root, dirs, files in os.walk(path, followlinks=False):
        for name in files:
            full = os.path.join(root, name)
            try:
                stat = os.lstat(full)
            except OSError:
                continue
            total += stat.st_size
        # Directories themselves are a few kB of metadata; ignored on
        # purpose so the number matches what a person expects to free.
        del dirs
    return total


# ---------------------------------------------------------------------------
# One saved run
# ---------------------------------------------------------------------------

class SavedRun:
    """One run directory the browser could be shown, and possibly delete."""

    __slots__ = ('run_id', 'root', 'path', 'has_map', 'has_posegraph',
                 'contents', 'bytes', 'mtime', 'width', 'height',
                 'resolution', 'origin', 'known_files', 'known_dirs',
                 'unknown', 'digest', 'cited_by')

    def __init__(self, run_id, root, path):
        self.run_id = str(run_id)
        self.root = str(root)
        self.path = str(path)
        self.has_map = False
        self.has_posegraph = False
        self.contents = []
        self.bytes = 0
        self.mtime = 0.0
        self.width = None
        self.height = None
        self.resolution = None
        self.origin = None
        # Exactly what may be removed, by name. delete_run() unlinks
        # these and nothing else -- see its docstring.
        self.known_files = []
        self.known_dirs = []
        # Anything the classifier does not recognise. Non-empty means
        # not deletable: this panel removes run output, and a directory
        # holding something else is not one it understands.
        self.unknown = []
        self.digest = ''
        self.cited_by = ''

    @property
    def deletable_reason(self):
        """'' if a browser may delete this, else why it may not."""
        if self.unknown:
            listed = ', '.join(sorted(self.unknown)[:4])
            more = '' if len(self.unknown) <= 4 else ', ...'
            return (f'holds files this dashboard does not recognise '
                    f'({listed}{more}) -- delete it from a terminal if '
                    f'you are sure')
        if not (self.known_files or self.known_dirs):
            return 'is empty -- nothing to delete'
        return ''

    @property
    def size_m(self):
        """(width, height) in metres, or None when the map did not parse."""
        if not (self.width and self.height and self.resolution):
            return None
        return (self.width * self.resolution, self.height * self.resolution)

    def as_dict(self):
        span = self.size_m
        return {
            'id': self.run_id,
            'root': self.root,
            'path': self.path,
            'has_map': self.has_map,
            'has_posegraph': self.has_posegraph,
            # What deleting this actually takes with it, in words. The whole
            # point of the confirmation step is that this is on screen first.
            'contents': list(self.contents),
            'bytes': int(self.bytes),
            'mtime': float(self.mtime),
            'width': self.width,
            'height': self.height,
            'resolution': self.resolution,
            'origin': list(self.origin) if self.origin else None,
            'span_m': list(span) if span else None,
            'unknown': list(self.unknown),
            'deletable': not self.deletable_reason,
            'reason': self.deletable_reason,
            # Recomputed at delete time. A tab that listed this run
            # before its map finished writing must not be able to
            # delete a map it never showed anyone.
            'digest': self.digest,
            'cited_by': self.cited_by,
        }

    def __repr__(self):  # pragma: no cover - debugging aid
        return f'<SavedRun {self.run_id} map={self.has_map} {self.bytes}B>'


def _digest(path):
    """A short fingerprint of a directory's contents.

    Covers every entry's relative name, size and modification time in
    nanoseconds, so a file appearing, growing or being rewritten all change
    it. Published with each run and re-checked immediately before the
    delete.

    This -- not the typed name -- is what actually catches a stale browser
    tab. A tab that listed a run while map_saver was still writing shows it
    as having no map; deleting from that tab would destroy a map the person
    was never shown. The typed name cannot see that, because the name did
    not change.
    """
    parts = []
    for root, dirs, files in os.walk(path, followlinks=False):
        dirs.sort()
        for name in sorted(files):
            full = os.path.join(root, name)
            try:
                stat = os.lstat(full)
            except OSError:
                continue
            relative = os.path.relpath(full, path)
            parts.append(f'{relative}\0{stat.st_size}\0{stat.st_mtime_ns}')
    raw = '\n'.join(parts).encode('utf-8', 'surrogatepass')
    return hashlib.sha256(raw).hexdigest()[:16]


def _cited_by(path):
    """Whatever in this workspace reads this run as a test oracle, or ''."""
    real = _real(path)
    for tail, note in CITED_BY.items():
        if real.endswith(os.sep + tail):
            return note
    return ''


def _describe(path, run):
    """Fill in what this directory holds, and what the map says about itself."""
    known_file_names = {name for name, _ in CONTENT_FILES}
    known_dir_names = {name for name, _ in CONTENT_DIRS}

    try:
        entries = sorted(os.listdir(path))
    except OSError:
        entries = []

    for entry in entries:
        full = os.path.join(path, entry)
        # A symlink is never "run output", whatever it is called. Unlinking
        # one is harmless, but classifying it as a known file would let a
        # link named map.pgm sit inside an otherwise clean run.
        if os.path.islink(full):
            run.unknown.append(entry)
        elif os.path.isdir(full):
            if entry in known_dir_names:
                run.known_dirs.append(entry)
            else:
                run.unknown.append(entry)
        elif os.path.isfile(full):
            if entry in known_file_names:
                run.known_files.append(entry)
            else:
                run.unknown.append(entry)
        else:
            run.unknown.append(entry)

    # Labels, in the declared order rather than the filesystem's, so the
    # list reads the same way every time.
    for filename, label in CONTENT_FILES:
        if filename not in run.known_files:
            continue
        if filename == 'map.yaml':
            run.has_map = True
        elif filename == 'posegraph.posegraph':
            run.has_posegraph = True
        if label:
            run.contents.append(label)
    for dirname, label in CONTENT_DIRS:
        if dirname in run.known_dirs and label:
            run.contents.append(label)

    run.digest = _digest(path)
    run.cited_by = _cited_by(path)

    if not run.has_map:
        return
    meta = read_map_yaml(os.path.join(path, 'map.yaml')) or {}
    run.resolution = meta.get('resolution')
    run.origin = meta.get('origin')
    # The image name comes out of the yaml rather than being assumed to be
    # map.pgm: map_saver writes the name it was given, and a hand-saved map
    # in one of these directories may not be called map at all.
    image = meta.get('image') or 'map.pgm'
    if os.path.basename(image) != image:
        # A yaml pointing outside its own directory is not something this
        # panel follows -- see the module docstring on relative image names.
        return
    size = read_pgm_size(os.path.join(path, image))
    if size:
        run.width, run.height = size


def scan(roots):
    """Every saved run under `roots`, newest first.

    `roots` must already have been through sanitize_roots(). Only immediate
    subdirectories are considered -- both layouts this workspace writes
    (~/.ros/racerbot_auto/<ts>/ and ~/.ros/racerbot_sim/auto/<ts>/) are one
    level deep, and recursing would let a nested checkout appear as a run.
    """
    runs = []
    for root in roots:
        try:
            entries = os.listdir(root)
        except OSError:
            continue
        for name in entries:
            path = os.path.join(root, name)
            # A symlinked "run" is not a run. Following one would let a link
            # dropped into the roots redirect a delete anywhere.
            if os.path.islink(path) or not os.path.isdir(path):
                continue
            if not is_plain_component(name):
                continue
            run = SavedRun(name, root, path)
            try:
                run.mtime = os.path.getmtime(path)
            except OSError:
                run.mtime = 0.0
            run.bytes = directory_size(path)
            _describe(path, run)
            runs.append(run)
    runs.sort(key=lambda r: (r.mtime, r.run_id), reverse=True)
    return runs


def find(runs, run_id):
    for run in runs:
        if run.run_id == run_id:
            return run
    return None


def in_use_by(targets, run_path):
    """Running processes whose command line mentions this run directory.

    Takes the list proccontrol.scan() already produced rather than reading
    /proc again, and stays a pure function of it so it can be tested without
    a process tree.

    This is what stops a browser deleting the map a running map_server is
    serving (`yaml_filename: <run>/map.yaml`) or the racing line a running
    pure_pursuit_node is following (`waypoints_file: <run>/raceline_
    profiled.csv`). Deleting either mid-run is the documented way to wedge
    particle_filter in its constructor.
    """
    needle = _real(run_path)
    hits = []
    for target in targets:
        cmdline = getattr(target, 'cmdline', '') or ''
        if needle in cmdline or str(run_path) in cmdline:
            hits.append(target)
    return hits


# ---------------------------------------------------------------------------
# Deleting
# ---------------------------------------------------------------------------

def is_plain_component(name):
    """True for a single ordinary directory name.

    A whitelist, not a blacklist. Run directories are named by
    `strftime('%Y%m%d-%H%M%S')` (auto_map_race_node), so letters, digits,
    dot, dash and underscore cover every name this workspace produces and
    every sensible rename on top of it -- while one rule rejects separators,
    both dot entries, a leading dot, `~`, NUL, shell metacharacters and
    anything non-ASCII in a single check.

    The browser sends a name, never a path, and this is what makes that true
    rather than merely intended. A blacklist here would be a list of the
    tricks someone thought of.
    """
    text = str(name)
    if not text or len(text) > _MAX_NAME_LEN:
        return False
    if text in ('.', '..') or text.startswith('.'):
        return False
    return _PLAIN_NAME_RE.fullmatch(text) is not None


def deletable(path, roots):
    """(True, '') if this path may be removed, else (False, reason).

    Re-checked immediately before the removal as well as at request time,
    because the two are separated by a queue hop and a thread hand-off, and
    a symlink swapped into the roots in between is exactly the thing a
    once-only check would miss.
    """
    if os.path.islink(path):
        return False, 'that entry is a symlink, not a run directory'
    if not os.path.isdir(path):
        return False, 'that run directory no longer exists'
    real = _real(path)
    reason = _protected_root_reason(real)
    if reason:
        return False, f'refused -- {reason}'
    for root in roots:
        if _contains(root, real):
            return True, ''
    return False, ('that path is outside every directory this dashboard '
                   'manages (see map_roots)')


def resolve_delete(runs, run_id, typed_name, roots, digest=None):
    """Vet a delete request. Returns (SavedRun, '') or (None, reason).

    Every check is made here, on the server, against `runs` from a scan taken
    just now. The browser sends a name and the string the person typed; it
    never sends a path and never sends a "confirmed" flag that would have to
    be believed. So a stale tab (the name is no longer in a fresh scan), a
    replayed message, and a hand-rolled WebSocket client (no typed name, or
    the wrong one) all land here and are refused with a reason.
    """
    run_id = '' if run_id is None else str(run_id)
    typed = '' if typed_name is None else str(typed_name)

    if not run_id:
        return None, 'no map was named'
    if not is_plain_component(run_id):
        return None, (f'"{run_id}" is not a plain directory name -- a path, a '
                      f'parent reference or a hidden entry is never accepted')

    run = find(runs, run_id)
    if run is None:
        return None, (f'no saved map called "{run_id}" is there now -- the '
                      f'list has just been re-read')

    if not typed:
        return None, (f'type the map name ({run.run_id}) to confirm the delete')
    if typed != run.run_id:
        return None, (f'the typed name does not match -- type {run.run_id} '
                      f'exactly to delete it')

    # Containment and symlink FIRST, before anything looks inside. A path
    # outside the managed roots must be refused as exactly that -- reporting
    # it as "empty" would describe the symptom and hide the problem.
    ok, reason = deletable(run.path, roots)
    if not ok:
        return None, reason

    refusal = run.deletable_reason
    if refusal:
        return None, f'"{run.run_id}" {refusal}'

    # A digest is optional so a caller that has no snapshot (a test, a
    # future CLI) still works, but the browser always sends one.
    if digest is not None and str(digest) != run.digest:
        return None, ('this run changed since your page listed it -- '
                      'refresh and look again before deleting')

    return run, ''


def delete_run(run, roots):
    """Remove one run directory. Returns (ok, detail, bytes_freed).

    Deliberately NOT shutil.rmtree on the directory. Every entry is
    re-classified here, immediately before anything is touched, and the
    removal then names each file individually. Two things fall out of that:

      * Nothing is ever removed that this module did not recognise as run
        output. A file that appeared between the scan and now -- someone's
        notes, a half-written map from a run still in progress -- aborts the
        delete instead of being swept up with it.
      * `os.rmdir` at the end cannot succeed unless the directory really is
        empty, so "something else was in there" surfaces as an error rather
        than as silence. rmtree would have removed it and said nothing.

    The one recursive removal is a `bag/` subdirectory, and only that name.

    The containment and symlink checks are repeated here on purpose: this is
    the function that actually removes files, so it does not rely on any
    caller having done the checking. See deletable().
    """
    path = run.path if hasattr(run, 'path') else str(run)
    ok, reason = deletable(path, roots)
    if not ok:
        return False, reason, 0

    # Re-classify now. `run` may be seconds old and is not trusted for
    # anything but its path.
    fresh = SavedRun(getattr(run, 'run_id', os.path.basename(path)),
                     getattr(run, 'root', os.path.dirname(path)), path)
    _describe(path, fresh)
    refusal = fresh.deletable_reason
    if refusal:
        return False, f'refused -- it {refusal}', 0

    freed = directory_size(path)

    for name in fresh.known_dirs:
        full = os.path.join(path, name)
        if os.path.islink(full):
            return False, f'refused -- {name} is a symlink', 0
        try:
            shutil.rmtree(full)
        except OSError as exc:
            return False, f'could not delete {name}: {exc}', 0

    for name in fresh.known_files:
        full = os.path.join(path, name)
        try:
            os.unlink(full)
        except FileNotFoundError:
            continue
        except OSError as exc:
            return False, f'could not delete {name}: {exc}', 0

    try:
        os.rmdir(path)
    except OSError as exc:
        # Reached when something appeared during the removal. The files that
        # were classified are gone; say so rather than implying nothing
        # happened, and leave the rest alone.
        return False, (f'removed the run files, but the directory would not '
                       f'go away: {exc}'), freed

    return True, 'deleted', freed
