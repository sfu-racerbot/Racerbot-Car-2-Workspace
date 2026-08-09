#!/usr/bin/env python3
"""Check this workspace's docs against the novice-docs standard.

Finds the mechanical problems that make docs hard for a beginner, so human
attention can go to the part that needs judgment: whether the explanation
actually explains anything.

Standard library only, no install, no ROS. Run from anywhere in the workspace:

    python3 .claude/skills/novice-docs/scripts/check_docs.py
    python3 .claude/skills/novice-docs/scripts/check_docs.py docs/operations.md
    python3 .claude/skills/novice-docs/scripts/check_docs.py --verbose

Findings are advice, not a grade. Some are wrong for a given file -- a reference
table legitimately has long rows. Use judgment; say what you deliberately left.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path

# --------------------------------------------------------------------------
# Configuration
# --------------------------------------------------------------------------

# A paragraph is flagged when a single sentence runs past SENTENCE_LIMIT, or the
# whole paragraph passes PARAGRAPH_LIMIT. Two separate problems: the sentence
# that's really three sentences in a trenchcoat, and the undifferentiated wall.
# Three crisp sentences totalling 210 characters are fine and shouldn't fire.
SENTENCE_LIMIT = 200
PARAGRAPH_LIMIT = 350

# Abbreviations whose '.' does not end a sentence.
_ABBREV = r"(?:e\.g|i\.e|etc|vs|cf|Dr|Mr|Ms|approx|Fig|no|No)"

# Directories under src/ that are upstream code, not ours. Their READMEs are
# out of scope -- we don't hold vendored docs to our house style.
VENDORED_DIRS = {
    "f1tenth_system",
    "transport_drivers",
    "particle_filter",
    "range_libc",
    "realsense-ros",
}

HEADER_FIELDS = {
    "who": (r"\*\*Who this is for:?\*\*", "**Who this is for:**"),
    "first": (r"\*\*Read first:?\*\*", "**Read first:**"),
    "payoff": (
        r"\*\*(You'll be able to|You will be able to|What's in it|What is in it):?\*\*",
        "**You'll be able to:** (or **What's in it:** for reference docs)",
    ),
}

# Terms a beginner will not know. Flagged when first used without a nearby gloss.
JARGON = [
    "node", "topic", "publish", "subscribe", "launch file", "package",
    "workspace", "sourcing", "mux", "multiplexer", "arbitration", "deadman",
    "odometry", "odom", "TF", "frame", "SLAM", "localization",
    "occupancy grid", "particle filter", "scan", "Ackermann", "lookahead",
    "curvature", "latch", "watchdog", "teleop", "LiDAR", "VESC",
    "velocity profile", "racing line", "pure pursuit", "follow-the-gap",
]

# Signals that a term was explained at or near first use.
GLOSS_PATTERNS = [
    r"\([^)]{8,}\)",          # a parenthetical with real content
    r"glossary\.md",           # links to the glossary
    r"\bmeans\b",
    r"\bshort for\b",
    r"\bthat is\b",
    r"\bi\.e\.",
    r"\bwhich is\b",
    r"\bstands for\b",
    r"\bin other words\b",
    r"\bis called\b",
    r"\bare called\b",
    r"\brefers to\b",
    r"\bworks like\b",
]

# Commands where "did that work?" is a real question for a beginner.
RUNNY_COMMAND = re.compile(r"^\s*(ros2\s+(launch|run)|colcon\s+build)\b")

PLACEHOLDER = re.compile(r"<[a-z_][a-z0-9_.-]*>", re.I)

TERMINAL_LABEL = re.compile(r"terminal\s*\d|\*\*terminal\b", re.I)
SUCCESS_SIGNAL = re.compile(
    r"working when|you should see|you'll see|you will see|success|"
    r"expect to see|output looks like|it worked",
    re.I,
)

LINK_RE = re.compile(r"\[([^\]]*)\]\(([^)]+)\)")

# Docs describing something a person can actually run, which therefore owe the
# reader a Highlights block. Every src/<pkg>/README.md counts too.
RUNNABLE_DOCS = {
    "web-dashboard.md",
    "simulator.md",
    "ros-simulator.md",
    "run-diagnostics.md",
    "racing-autonomy.md",
    "drive-intent.md",
    "realsense-camera.md",
    "usb-camera-livestream.md",
    "odom-calibration.md",
}

HIGHLIGHTS_HEADING = re.compile(r"^#{2,4}\s+Highlights\b", re.I)

SEVERITY_ORDER = {"error": 0, "warn": 1, "info": 2}


# --------------------------------------------------------------------------
# Finding model
# --------------------------------------------------------------------------


@dataclass
class Finding:
    line: int
    severity: str
    category: str
    message: str


@dataclass
class FileReport:
    path: Path
    findings: list[Finding] = field(default_factory=list)

    def add(self, line: int, severity: str, category: str, message: str) -> None:
        self.findings.append(Finding(line, severity, category, message))


# --------------------------------------------------------------------------
# Markdown helpers
# --------------------------------------------------------------------------


def classify_lines(lines: list[str]) -> list[str]:
    """Label each line: 'code', 'fence', 'table', 'heading', 'blank', 'prose'."""
    kinds: list[str] = []
    fence: str | None = None
    in_summary = False

    for raw in lines:
        stripped = raw.strip()

        if fence is not None:
            kinds.append("fence" if stripped.startswith(fence) else "code")
            if stripped.startswith(fence):
                fence = None
            continue

        m = re.match(r"^(`{3,}|~{3,})", stripped)
        if m:
            fence = m.group(1)[0] * len(m.group(1))
            kinds.append("fence")
            continue

        # <details> deep dives: markup lines are scaffolding, not prose. A
        # <summary> may wrap onto several lines before it closes.
        if in_summary:
            kinds.append("html")
            if "</summary>" in stripped:
                in_summary = False
            continue
        if stripped.startswith(("<details", "</details", "<summary", "</summary")):
            kinds.append("html")
            if stripped.startswith("<summary") and "</summary>" not in stripped:
                in_summary = True
            continue

        if not stripped:
            kinds.append("blank")
        elif stripped.startswith("#"):
            kinds.append("heading")
        elif stripped.startswith("|"):
            kinds.append("table")
        elif re.match(r"^ {4,}\S", raw) and not stripped.startswith(("-", "*", "1.")):
            kinds.append("code")  # indented code block
        else:
            kinds.append("prose")

    return kinds


def visible_length(line: str) -> int:
    """Length as a human reads it: link URLs and blockquote markers don't count."""
    text = LINK_RE.sub(lambda m: m.group(1), line)
    text = re.sub(r"^\s*>\s?", "", text)
    text = re.sub(r"<[^>]+>", "", text)
    return len(text.strip())


def slugify(heading: str) -> str:
    """GitHub's heading-anchor algorithm, near enough for link checking."""
    text = heading.strip().lstrip("#").strip()
    text = LINK_RE.sub(lambda m: m.group(1), text)
    # Strip markdown emphasis/code markers, but keep '_': GitHub's slugger treats
    # underscores as word characters, so 'gap_follow' stays 'gap_follow'.
    text = re.sub(r"[`*~]", "", text)
    text = text.lower()
    text = re.sub(r"[^\w\s-]", "", text)
    # One hyphen per whitespace character, not per run: GitHub turns the two
    # spaces left behind by a stripped em-dash into '--'.
    text = re.sub(r"\s", "-", text)
    return text


def headings_of(path: Path) -> set[str]:
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError):
        return set()
    kinds = classify_lines(lines)
    return {
        slugify(line)
        for line, kind in zip(lines, kinds)
        if kind == "heading"
    }


# --------------------------------------------------------------------------
# Individual checks
# --------------------------------------------------------------------------


def check_header_block(lines: list[str], kinds: list[str], rep: FileReport) -> None:
    """Every doc opens with who it's for, what to read first, and the payoff."""
    h1 = next((i for i, k in enumerate(kinds) if k == "heading"
               and lines[i].strip().startswith("# ")), None)
    if h1 is None:
        rep.add(1, "warn", "header", "No top-level '# Title' heading.")
        return

    window = "\n".join(lines[h1:h1 + 16])
    missing = [
        label for _, (pattern, label) in HEADER_FIELDS.items()
        if not re.search(pattern, window)
    ]
    if len(missing) == len(HEADER_FIELDS):
        rep.add(h1 + 1, "warn", "header",
                "No header block. Add one right after the title: "
                "Who this is for / Read first / You'll be able to. "
                "See assets/doc-template.md.")
    elif missing:
        rep.add(h1 + 1, "warn", "header",
                "Header block is missing: " + "; ".join(missing))


def sentences(text: str) -> list[str]:
    """Rough sentence split -- good enough to spot a runaway sentence."""
    protected = re.sub(_ABBREV + r"\.", lambda m: m.group(0).replace(".", "\0"),
                       text, flags=re.I)
    # Split on markup, not after it has been stripped: a sentence may open with
    # `a_code_identifier`, and a bold run may close after the full stop
    # ("...the ladder.** Simulator..."), so allow trailing markers before the
    # space and any word character or marker after it.
    parts = re.split(r"(?<=[.!?])[*`_\"')\]]*\s+(?=[`*_\[(A-Za-z0-9])", protected)
    return [p.replace("\0", ".").strip() for p in parts if p.strip()]


def check_long_paragraphs(lines: list[str], kinds: list[str], rep: FileReport) -> None:
    """Dense multi-idea paragraphs are this repo's characteristic problem."""
    for i, (line, kind) in enumerate(zip(lines, kinds)):
        if kind != "prose":
            continue

        raw = LINK_RE.sub(lambda m: m.group(1), line)
        raw = re.sub(r"^\s*>\s?", "", raw)
        # Emphasis and code markers aren't read, so they don't count toward
        # length -- but they must survive until after the sentence split.
        text = re.sub(r"[*`_]", "", raw).strip()
        length = len(text)
        if not length:
            continue

        longest = max((len(re.sub(r"[*`_]", "", s)) for s in sentences(raw)),
                      default=0)

        if longest > SENTENCE_LIMIT:
            rep.add(i + 1, "warn", "dense",
                    f"One sentence runs {longest} characters. That's usually two "
                    f"or three sentences joined by dashes or semicolons -- split "
                    f"it (standard.md#2-one-idea-per-paragraph).")
        elif length > PARAGRAPH_LIMIT:
            rep.add(i + 1, "warn", "dense",
                    f"Paragraph is {length} characters. Break it up so each "
                    f"paragraph carries one idea "
                    f"(standard.md#2-one-idea-per-paragraph).")


def end_of_header_block(lines: list[str], kinds: list[str]) -> int:
    """First line index past the '# Title' and its '> Who this is for' block."""
    h1 = next((i for i, k in enumerate(kinds) if k == "heading"
               and lines[i].strip().startswith("# ")), None)
    if h1 is None:
        return 0

    i = h1 + 1
    while i < len(lines) and not lines[i].strip():
        i += 1
    while i < len(lines) and lines[i].strip().startswith(">"):
        i += 1
    return i


def check_jargon(path: Path, lines: list[str], kinds: list[str],
                 rep: FileReport) -> None:
    """Flag robotics/ROS terms used before they're explained."""
    if path.name in {"glossary.md", "glossary-seed.md"}:
        return

    prose_idx = [i for i, k in enumerate(kinds)
                 if k in ("prose", "table", "heading", "html")]
    prose_idx = [i for i in prose_idx if i >= end_of_header_block(lines, kinds)]

    for term in JARGON:
        pattern = re.compile(r"\b" + re.escape(term) + r"\b",
                             0 if term.isupper() else re.I)
        for i in prose_idx:
            line = lines[i]
            # Ignore matches inside inline code or link targets -- `/scan` and
            # `urg_node` are names being used, not concepts being introduced.
            bare = re.sub(r"`[^`]*`", "", line)
            bare = LINK_RE.sub(lambda m: m.group(1), bare)
            if not pattern.search(bare):
                continue

            context = "\n".join(lines[i:i + 3])
            if any(re.search(p, context, re.I) for p in GLOSS_PATTERNS):
                break  # glossed at first use: good
            rep.add(i + 1, "info", "jargon",
                    f"'{term}' used here (first time in this file) with no "
                    f"nearby explanation. Gloss it or link glossary.md.")
            break


def check_links(path: Path, lines: list[str], kinds: list[str],
                rep: FileReport) -> None:
    """Broken relative links and dead #anchors."""
    for i, (line, kind) in enumerate(zip(lines, kinds)):
        if kind in ("code", "fence"):
            continue
        for _, target in LINK_RE.findall(line):
            target = target.strip().split(" ")[0]
            if target.startswith(("http://", "https://", "mailto:")):
                continue

            file_part, _, anchor = target.partition("#")

            if not file_part:  # same-file anchor
                if anchor and anchor not in headings_of(path):
                    rep.add(i + 1, "error", "link",
                            f"Anchor '#{anchor}' does not match any heading "
                            f"in this file.")
                continue

            dest = (path.parent / file_part).resolve()
            if not dest.exists():
                rep.add(i + 1, "error", "link",
                        f"Link target does not exist: {file_part}")
                continue

            if anchor and dest.suffix == ".md":
                if anchor not in headings_of(dest):
                    rep.add(i + 1, "error", "link",
                            f"Anchor '#{anchor}' not found in {file_part}.")


def check_command_blocks(lines: list[str], kinds: list[str],
                         rep: FileReport) -> None:
    """Launch/build commands need a terminal label and a success signal."""
    i = 0
    while i < len(lines):
        if kinds[i] != "fence":
            i += 1
            continue

        start = i
        end = start + 1
        while end < len(lines) and kinds[end] == "code":
            end += 1

        body = lines[start + 1:end]
        runnable = [b for b in body if RUNNY_COMMAND.match(b)]
        # `ros2 run <package> <executable>` is showing the shape of a command,
        # not giving one. "How do you know it worked?" doesn't apply.
        runnable = [b for b in runnable if not PLACEHOLDER.search(b)]
        if not runnable:
            i = end + 1
            continue

        # A block holding several alternative invocations is a menu ("useful
        # overrides", a list of variants), not a step in a procedure. Asking it
        # for one terminal number and one success signal makes no sense.
        if len(runnable) >= 2:
            i = end + 1
            continue

        before = "\n".join(lines[max(0, start - 5):start])
        after = "\n".join(lines[end:min(len(lines), end + 7)])

        problems = []
        if not TERMINAL_LABEL.search(before):
            problems.append("no terminal label above it")
        if not SUCCESS_SIGNAL.search(after):
            problems.append("no success signal below it")

        if problems:
            rep.add(start + 1, "info", "command",
                    "Command block has " + " and ".join(problems) +
                    ". A beginner can't tell a working launch from a broken one "
                    "(standard.md#6-command-blocks).")
        i = end + 1


def check_details_blocks(lines: list[str], rep: FileReport) -> None:
    """The <details> blank-line rules, which GitHub enforces silently.

    Get these wrong and the block still folds, but every table and code fence
    inside it renders as raw text -- easy to miss, since it looks fine locally.
    """
    depth = 0
    for i, line in enumerate(lines):
        s = line.strip()

        if s.startswith("<details"):
            depth += 1
            if depth > 1:
                rep.add(i + 1, "warn", "details",
                        "Nested <details>. Restructure the section instead -- "
                        "readers rarely find the inner one.")
            nxt = lines[i + 1].strip() if i + 1 < len(lines) else ""
            if not nxt.startswith("<summary"):
                rep.add(i + 1, "error", "details",
                        "<details> must be followed immediately by <summary>, "
                        "with no blank line between them, or the fold breaks.")

        if s.endswith("</summary>") and i + 1 < len(lines) and lines[i + 1].strip():
            rep.add(i + 2, "error", "details",
                    "Add a blank line after </summary>. Without it GitHub "
                    "renders the Markdown inside this block as raw text.")

        if s.startswith("</details"):
            depth -= 1
            if i > 0 and lines[i - 1].strip():
                rep.add(i, "error", "details",
                        "Add a blank line before </details>. Without it GitHub "
                        "renders the Markdown inside this block as raw text.")

    if depth > 0:
        rep.add(len(lines), "error", "details",
                f"{depth} <details> block(s) never closed with </details>.")


def check_highlights(path: Path, lines: list[str], rep: FileReport) -> None:
    """Docs for things you can run say what they're good for, up front."""
    name = path.name
    runnable = name == "README.md" and path.parent.parent.name == "src"
    runnable = runnable or name in RUNNABLE_DOCS
    if not runnable:
        return

    if not any(HIGHLIGHTS_HEADING.match(l) for l in lines):
        rep.add(1, "warn", "highlights",
                "No '## Highlights' section. Anything a person can run says "
                "what it does and what's good about it, up front, in terms an "
                "outsider can follow (standard.md#8-the-highlights-block).")


def check_index(docs_dir: Path, rep: FileReport) -> None:
    """Every doc should be reachable from the index."""
    index = docs_dir / "README.md"
    if not index.exists():
        rep.add(1, "warn", "index",
                f"{index} does not exist. It is the canonical docs index "
                f"(structure.md).")
        return

    text = index.read_text(encoding="utf-8")
    for doc in sorted(docs_dir.glob("*.md")):
        if doc.name == "README.md":
            continue
        if doc.name not in text:
            rep.add(1, "warn", "index",
                    f"{doc.name} is not linked from docs/README.md.")


# --------------------------------------------------------------------------
# Driver
# --------------------------------------------------------------------------


def find_root(start: Path) -> Path:
    for candidate in [start, *start.parents]:
        if (candidate / "docs").is_dir() and (candidate / "src").is_dir():
            return candidate
    return start


def submodule_paths(root: Path) -> set[str]:
    gitmodules = root / ".gitmodules"
    if not gitmodules.exists():
        return set()
    text = gitmodules.read_text(encoding="utf-8", errors="replace")
    return {
        Path(m).name
        for m in re.findall(r"^\s*path\s*=\s*(.+)$", text, re.M)
    }


def in_scope_files(root: Path) -> list[Path]:
    skip = VENDORED_DIRS | submodule_paths(root)
    files: list[Path] = []

    readme = root / "README.md"
    if readme.exists():
        files.append(readme)

    files.extend(sorted((root / "docs").glob("*.md")))

    for pkg_readme in sorted((root / "src").glob("*/README.md")):
        if pkg_readme.parent.name not in skip:
            files.append(pkg_readme)

    return files


def check_file(path: Path) -> FileReport:
    rep = FileReport(path)
    try:
        lines = path.read_text(encoding="utf-8").splitlines()
    except (OSError, UnicodeDecodeError) as exc:
        rep.add(1, "error", "read", f"Could not read file: {exc}")
        return rep

    kinds = classify_lines(lines)
    check_header_block(lines, kinds, rep)
    check_long_paragraphs(lines, kinds, rep)
    check_jargon(path, lines, kinds, rep)
    check_links(path, lines, kinds, rep)
    check_command_blocks(lines, kinds, rep)
    check_details_blocks(lines, rep)
    check_highlights(path, lines, rep)
    return rep


def render(reports: list[FileReport], root: Path, verbose: bool) -> tuple[int, int]:
    per_category_cap = 100 if verbose else 3
    totals: dict[str, int] = {}
    errors = 0

    for rep in reports:
        if not rep.findings:
            continue

        try:
            shown_path = rep.path.relative_to(root)
        except ValueError:
            shown_path = rep.path

        print(f"\n\033[1m{shown_path}\033[0m")

        ordered = sorted(
            rep.findings,
            key=lambda f: (SEVERITY_ORDER.get(f.severity, 9), f.line),
        )

        seen: dict[str, int] = {}
        for f in ordered:
            totals[f.category] = totals.get(f.category, 0) + 1
            if f.severity == "error":
                errors += 1

            seen[f.category] = seen.get(f.category, 0) + 1
            if seen[f.category] > per_category_cap:
                continue

            colour = {"error": "\033[31m", "warn": "\033[33m",
                      "info": "\033[36m"}.get(f.severity, "")
            print(f"  {colour}{f.severity:<5}\033[0m "
                  f"{shown_path}:{f.line}  [{f.category}] {f.message}")

        for category, count in seen.items():
            if count > per_category_cap:
                print(f"  \033[2m... and {count - per_category_cap} more "
                      f"'{category}' findings (--verbose to see all)\033[0m")

    print("\n" + "=" * 70)
    if not totals:
        print("No findings. (The checker measures shape, not sense -- "
              "read it as a beginner too.)")
    else:
        print("Findings by category:")
        labels = {
            "header": "missing header block",
            "dense": "overlong paragraphs",
            "jargon": "jargon used before it's explained",
            "link": "broken links/anchors",
            "command": "command blocks with no terminal label or success signal",
            "index": "docs missing from the index",
            "details": "broken <details> markup",
            "highlights": "runnable things with no Highlights block",
            "read": "unreadable files",
        }
        for category, count in sorted(totals.items(), key=lambda kv: -kv[1]):
            print(f"  {count:>4}  {labels.get(category, category)}")
        print("\nThese are a to-do list, not a score. Some will be wrong for a "
              "given file.")

    return errors, sum(totals.values())


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Check docs against the novice-docs standard.")
    parser.add_argument("paths", nargs="*", type=Path,
                        help="Files to check. Default: all docs in scope.")
    parser.add_argument("--verbose", "-v", action="store_true",
                        help="Show every finding instead of the first few per category.")
    parser.add_argument("--strict", action="store_true",
                        help="Exit non-zero if there are error-level findings.")
    parser.add_argument("--root", type=Path, default=None,
                        help="Workspace root (auto-detected by default).")
    args = parser.parse_args()

    root = (args.root or find_root(Path.cwd())).resolve()

    if args.paths:
        targets = [p.resolve() for p in args.paths]
        missing = [p for p in targets if not p.exists()]
        for p in missing:
            print(f"error: no such file: {p}", file=sys.stderr)
        targets = [p for p in targets if p.exists()]
        if not targets:
            return 2
    else:
        targets = in_scope_files(root)
        if not targets:
            print(f"error: found no docs under {root}. Is this the workspace root?",
                  file=sys.stderr)
            return 2

    reports = [check_file(p) for p in targets]

    # The index check is workspace-wide, not per-file; only run it on a full sweep.
    if not args.paths and (root / "docs").is_dir():
        index_rep = FileReport(root / "docs")
        check_index(root / "docs", index_rep)
        if index_rep.findings:
            reports.append(index_rep)

    print(f"Checked {len(targets)} file(s) under {root}")
    errors, total = render(reports, root, args.verbose)

    if args.strict and errors:
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
