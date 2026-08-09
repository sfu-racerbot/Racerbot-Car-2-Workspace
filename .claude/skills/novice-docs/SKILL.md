---
name: novice-docs
description: Write, rewrite, audit, or reorganize documentation in this ROS2/F1TENTH workspace so that someone brand new to robotics can actually follow it. Use this whenever touching any Markdown under docs/, the top-level README.md, or any src/<package>/README.md — writing a new doc, editing or extending an existing one, reviewing a doc for clarity, adding a section to one, updating the docs index, or documenting a new package or node. Also use it whenever someone says the docs are confusing, dense, badly organized, hard to follow, assume too much, or "written for people who already know ROS" — and when a doc change is a side effect of a code change, since a stale or expert-only doc is the same failure as a wrong one.
---

# Writing docs a robotics novice can follow

## Who is actually reading this

Write for a **first-year undergrad on their first week in the club**. They can program — Python, some C++, basic git. They have never used ROS2, never run `colcon`, never seen a launch file, and have never worked on a robot that moves under its own power.

That reader is not a hypothetical. They are the single most important audience this workspace has, for a reason that is easy to miss:

> **In this repo, "novice-readable" and "safe" are the same goal.** This car is a physical machine that can hurt someone. The person most likely to make it do something dangerous is the one who read a doc and thought they understood it. Every paragraph a beginner skims past because it looked like jargon is a paragraph that failed at its actual job.

So clarity here is not politeness or polish. A doc that only an expert can parse has not documented anything — it has recorded something.

## The one rule that shapes everything else

**Add a beginner layer on top; never delete the depth underneath.**

This workspace's docs contain real hard-won knowledge — the friction-circle math in `racing-autonomy.md`, the postmortems about latched states, the "why not MPC" reasoning, the exact servo constant that looks like a bug and isn't. Somebody paid for that knowledge with their time and probably with a crashed car. It stays.

The problem was never that the depth exists. It is that a beginner opening the file hits the depth immediately with no way to tell what they need now, what they need later, and what they may never need. So the work is almost always **restructuring and signposting, not cutting**:

- Lead with the plain-language version, then go deep.
- Mark the deep parts as skippable, explicitly, so skipping feels sanctioned rather than like giving up.
- Move nothing between files unless asked. Filenames are referenced from code, commit messages, and CLAUDE.md; renaming them breaks links for no reader benefit.

If you genuinely believe something should be deleted, say so to the user and let them decide. Do not quietly drop paragraphs while "simplifying" — that is how a repo loses the one sentence that explains a two-day debugging session.

## What's in scope

| In scope | Out of scope |
|---|---|
| `docs/*.md` | `CLAUDE.md` (written for an agent, different audience) |
| top-level `README.md` | source code comments and docstrings |
| `src/<package>/README.md` | vendored/upstream docs under `src/f1tenth_system/`, submodules |

## Pick your mode

**Writing a new doc** → build it to the standard from the start. Read `references/standard.md`, then use `assets/doc-template.md` as the skeleton.

**Editing or extending an existing doc** → match the standard for what you touch, and bring the doc's header block up to date if it lacks one. Don't silently rewrite the whole file because you were asked to add a section — but do mention to the user if the surrounding doc needs work.

**Auditing / rewriting a doc that's already there** → follow the procedure in `references/rewrite-playbook.md`. It has before/after pairs taken from this repo's actual docs, which is the fastest way to calibrate.

**Working on the index or reading order** → read `references/structure.md`. The canonical index is `docs/README.md`; the top-level `README.md` links to it rather than duplicating the table, so the two can't drift.

Always finish by running the checker (below).

## The eight things that make the difference

Full detail and worked examples for each live in `references/standard.md`. In brief:

**1. Open with a header block that answers "is this for me, right now?"**
Every doc starts with who it's for, what to read first, what you'll be able to do afterward, and roughly how long it takes. A beginner's worst experience is reading 400 lines before discovering they needed a different doc.

**2. One idea per paragraph.**
This repo's characteristic failure is the 300-to-550-character paragraph that chains four ideas with em-dashes and semicolons. An expert reads it as one dense thought. A beginner reads it three times and gives up. Split on the joints.

**3. Gloss every piece of jargon at first use, and link the glossary.**
"Node", "topic", "mux", "deadman", "TF", "odom", "occupancy grid", "lookahead" — all of these are used in this repo before they're defined. First use gets a short parenthetical or a following sentence in plain language, plus a link to `docs/glossary.md`.

**4. Concrete before abstract.**
Say what the reader will do and see, then explain the mechanism behind it. Most of these docs currently open with mechanism, which only works if you already know why you'd care.

**5. Fold depth into collapsible blocks instead of hiding it or cutting it.**
Math, derivations, wire formats and postmortems go inside a `<details>` block that GitHub renders folded, with a `<summary>` saying what's inside and whether skipping is safe. The page reads short; the expert clicks once and gets everything. Nothing is lost, and the beginner is told explicitly that moving on is allowed. Safety content is never collapsed.

**6. Every command block says which terminal it belongs to and how you know it worked.**
This is the biggest single win for robotics beginners. They run a launch file, get 40 lines of log, and have no idea whether that was success. Multi-terminal workflows compound it. Label the terminal, then state the success signal and the most likely failure.

**7. Safety content gets clearer, never softer.**
Simplifying prose must never dilute the LB deadman policy, the wheels-off-the-ground test order, or the `racerbot_sim` hardware interlock. A beginner is precisely the reader those rules exist to protect, so they get the *plainest* possible statement of the rule and the consequence — plus, in a sentence, why the rule exists. "Because a rule says so" is what people talk themselves out of at 11pm before a race.

**8. Anything you can run gets a Highlights block.**
Every doc for a usable thing — dashboard, simulators, diagnostics, cameras, calibration, each driving node, each package README — opens with four to eight bullets on what it does and what's genuinely good about it, then a short "Why it exists". Write it for an outsider: a judge, another team, someone deciding whether this project is worth their attention. Concrete claims with numbers, every one checkable against this repo, limits included. No marketing adjectives — they read as evasion.

## Never do these

- **Never soften, condense, or drop a safety rule** in the name of readability. If a safety section is hard to read, make it shorter and blunter, not gentler.
- **Never change a documented command, parameter, path, or value** to make prose flow better. If a command looks wrong, verify it against the code and tell the user — a doc that reads beautifully and gives a wrong command is worse than the dense version.
- **Never invent behavior to fill a gap.** If you can't tell what something does, read the source; if it's still unclear, write the section with an explicit `TODO(verify):` note and flag it, rather than guessing plausibly. Confident wrong documentation is expensive precisely because beginners trust it.
- **Never rename or move doc files** unless the user asked for it.

## Verify with the checker

`scripts/check_docs.py` catches the mechanical problems objectively, so your attention stays on the parts that need judgment. Stdlib only, no install:

```bash
# check everything in scope
python3 .claude/skills/novice-docs/scripts/check_docs.py

# check just the files you touched
python3 .claude/skills/novice-docs/scripts/check_docs.py docs/operations.md README.md

# see every finding rather than the top few per file
python3 .claude/skills/novice-docs/scripts/check_docs.py --verbose
```

It flags: overlong prose paragraphs, missing or incomplete header blocks, jargon used before it's glossed, broken relative links, command blocks with no terminal label or success signal, and docs missing from the index.

Treat the output as a to-do list, not a score. Some findings are wrong for a given file — a reference table legitimately has long lines. Use judgment, and say which findings you're deliberately leaving when you report back.

**The checker cannot see the thing that matters most**: whether a beginner would actually understand the explanation. It measures shape, not sense. A doc that passes clean can still be incomprehensible, so read what you wrote as if you'd never seen a robot before, and be honest about what you'd stumble on.

## Reference files

| File | Read it when |
|---|---|
| `references/standard.md` | Writing or rewriting anything — the full house style with worked examples |
| `references/rewrite-playbook.md` | Auditing/rewriting an existing doc — the step-by-step procedure and before/after pairs from this repo |
| `references/structure.md` | Touching the index, reading order, or a doc's place in the overall set |
| `assets/doc-template.md` | Starting a new doc from scratch |
| `assets/glossary-seed.md` | Building or extending `docs/glossary.md` |
