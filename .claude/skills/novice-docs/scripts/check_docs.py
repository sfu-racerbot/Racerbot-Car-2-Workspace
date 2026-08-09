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

PROSE_LINE_LIMIT = 200

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
]

# Commands where "did that work?" is a real question for a beginner.
RUNNY_COMMAND = re.compile(r"^\s*(ros2\s+(launch|run)|colcon\s+build)\b")

TERMINAL_LABEL = re.compile(r"terminal\s*\d|\*\*terminal\b", re.I)
SUCCESS_SIGNAL = re.compile(
    r"working when|you should see|you'll see|you will see|success|"
    r"expect to see|output looks like|it worked",
    re.I,
)

LINK_RE = re.compile(r"\[([^\]]*)\]\(([^)]+)\)")

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


def check_long_paragraphs(lines: list[str], kinds: list[str], rep: FileReport) -> None:
    """Dense multi-idea paragraphs are this repo's characteristic problem."""
    for i, (line, kind) in enumerate(zip(lines, kinds)):
        if kind != "prose":
            continue
        length = visible_length(line)
        if length > PROSE_LINE_LIMIT:
            rep.add(i + 1, "warn", "dense",
                    f"Paragraph is {length} characters. Split it into one idea "
                    f"per paragraph (standard.md#2-one-idea-per-paragraph).")


def check_jargon(path: Path, lines: list[str], kinds: list[str],
                 rep: FileReport) -> None:
    """Flag robotics/ROS terms used before they're explained."""
    if path.name in {"glossary.md", "glossary-seed.md"}:
        return

    prose_idx = [i for i, k in enumerate(kinds) if k in ("prose", "table", "heading")]

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
        if not any(RUNNY_COMMAND.match(b) for b in body):
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
