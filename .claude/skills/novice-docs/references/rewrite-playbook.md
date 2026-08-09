# The rewrite playbook

The procedure for improving a doc that already exists. Follow it in order — the early steps make the later ones cheaper, and doing paragraph surgery before you understand the doc's shape wastes effort on text that should have moved anyway.

## Before you touch anything

**Read the whole doc.** All of it, even 1,200 lines. You cannot judge what's skippable, what's duplicated, or what a beginner needs first until you've seen the whole thing.

**Skim the code it documents.** Enough to know whether the doc is still true. You are not primarily fact-checking, but you will find drift, and a doc that reads beautifully while describing a parameter that was renamed is a net loss. Note anything suspicious for the user rather than fixing it silently.

**Decide the doc's job.** Most docs here are a mix, and the mix is usually the problem:

| Kind | The reader wants | Example in this repo |
|---|---|---|
| Orientation | To understand what this even is | `concepts.md` |
| Procedure | To do a specific thing right now | `operations.md` |
| Reference | To look one fact up | `hardware-reference.md` |
| Explanation | To understand why it works this way | the "why not MPC" part of `racing-autonomy.md` |
| Safety | To not get hurt | the deadman policy sections |

A doc that mixes four of these without labeling which is which is the core complaint about this docs set. You're usually not splitting the file — you're making the boundaries visible.

## The passes

Work in this order. Each pass is cheap to do well once the previous one is done.

### Pass 1 — Frame it

Add or fix the header block (`references/standard.md#1-the-header-block`). To fill in "Read first", look at what the doc assumes in its first 20 lines; that assumption *is* the prerequisite.

Then read the opening paragraphs. Do they tell a beginner what this is and why they'd care, or do they start mid-mechanism? Most of these docs start mid-mechanism. Write the two or three sentences of orientation that are missing.

### Pass 2 — Restructure, don't rewrite

Now that you know the doc's shape, fix the order and the signposting:

- Does the beginner path come before the deep path? If the math is above the "how do I run it", move the math down. **Moving a section within a file is fine and encouraged; deleting it is not.**
- Mark deep sections with `Deep dive:` and a skip note.
- Are there sections that are really a different kind of content (a postmortem buried in a procedure)? Give them their own heading so they can be skipped as a unit.
- Add a contents list if the doc is over ~200 lines.

This pass produces the biggest readability gain per edit, and it's the one people skip because it feels like it isn't "writing".

### Pass 3 — Paragraph surgery

Now split the dense paragraphs (`references/standard.md#2-one-idea-per-paragraph`). Do this after restructuring, so you're not carefully rewriting a paragraph that was about to move or get a skip marker anyway.

Run the checker first to get the list of offenders rather than hunting by eye.

### Pass 4 — Gloss the jargon

Walk the doc top to bottom and gloss each term at its first appearance. Add anything new to `docs/glossary.md`.

Watch for the trap: after pass 2 moved sections around, "first use" may have moved too. Do this pass after restructuring, not before.

### Pass 5 — Command blocks

Add terminal labels and success signals (`references/standard.md#6-command-blocks`).

The success signal is the part that requires actual knowledge. If you don't know what a working run looks like, don't invent it — check the code, check `troubleshooting.md`, or leave `TODO(verify): what does success look like here?` and tell the user. A made-up success signal is worse than none, because it teaches a beginner to distrust their own eyes.

### Pass 6 — Safety re-read

Go back through every safety-related passage and check that your edits made it *sharper*, not smoother. Specifically:

- Is every rule still stated as a rule, not as advice?
- Did any warning drift below the thing it warns about?
- Did any consequence get softened, shortened, or turned into a parenthetical?
- Does each rule still say why it exists?

If you changed nothing in a safety section, that's a perfectly good outcome. Report it as deliberate.

### Pass 7 — Check and report

Run the checker, fix what's worth fixing, then report to the user:

- What you changed, by pass.
- What you deliberately left alone, and why.
- Anything you found that looks factually wrong or stale (do not fix silently).
- Any `TODO(verify):` markers you left, and what question each one is asking.

## Not losing content

The main risk in this work is quiet deletion — a paragraph disappears during a rewrite and nobody notices for a year.

Two habits that prevent it:

**Diff every rewrite before you finish.** `git diff` on the file, and read the removed lines specifically. For each removed line, you should be able to say either "that moved to line N" or "that was genuinely redundant with X". If you can't, put it back.

**Rewrites shrink prose, not facts.** Splitting a 519-character paragraph into four usually makes the file *longer*, not shorter. If a section got dramatically shorter, something left that shouldn't have. A shrinking line count is a signal to look harder, not a win.

## Large docs

For anything over ~400 lines (`racing-autonomy.md` at 1,272, `web_dashboard.md` at 704, `sim-fidelity-audit.md` at 522), don't attempt every pass over the whole file in one go. You'll run out of care somewhere around line 600 and the second half will be worse than the first.

Instead:

1. Do passes 1 and 2 over the **whole** file first. Signposting is what a beginner needs most from a giant doc, it's the cheapest pass, and it makes the file usable even if you stop there.
2. Then do passes 3–5 **section by section**, and say which sections you completed.

Stopping halfway through a large doc is fine as long as you say where you stopped. Silently doing a worse job on the back half is not.

## When the doc is fine

Sometimes the answer is "this doc is already good". `ros-simulator.md`, `simulator.md`, and `realsense-camera.md` have no paragraphs over 200 characters and are much closer to the standard than the older files.

Say so. Add the header block if it's missing, note that the rest meets the standard, and move on. Manufacturing changes to look productive is how a good doc gets worse.
