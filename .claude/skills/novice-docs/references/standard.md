# The house standard

The full version of the techniques in SKILL.md. Every "after" example below is a rewrite of text that is actually in this repo.

## Contents

1. [The header block](#1-the-header-block)
2. [One idea per paragraph](#2-one-idea-per-paragraph)
3. [Glossing jargon](#3-glossing-jargon)
4. [Concrete before abstract](#4-concrete-before-abstract)
5. [Marking depth as skippable](#5-marking-depth-as-skippable)
6. [Command blocks](#6-command-blocks)
7. [Safety writing](#7-safety-writing)
8. [The Highlights block](#8-the-highlights-block)
9. [Smaller habits](#9-smaller-habits)

---

## 1. The header block

Every doc opens with this, immediately after the `# Title`, before anything else:

```markdown
> **Who this is for:** anyone who wants to drive the car by hand for the first time.
> **Read first:** [concepts.md](concepts.md) — you need to know what a launch file is.
> **You'll be able to:** bring the hardware up and drive the car with the gamepad.
> **Time:** about 20 minutes, plus setup.
```

All four lines earn their place:

- **Who this is for** lets a reader bail out in five seconds instead of four hundred lines.
- **Read first** is the prerequisite chain. This is the single highest-value line in the block for a beginner, because the most common way to fail at these docs is to start in the middle of the dependency graph and conclude you're not smart enough.
- **You'll be able to** states the payoff, which is what gets someone through a dense section.
- **Time** sets expectations. "This takes 20 minutes" and "this takes an afternoon" call for different moods.

For a pure reference doc (`hardware-reference.md`, a topic table), swap **You'll be able to** for **What's in it**, and drop **Time** — nobody reads a lookup table end to end.

## 2. One idea per paragraph

The characteristic failure mode in this repo. Real example from `docs/architecture.md` — one paragraph, 519 characters, five distinct facts:

**Before:**

> `ros2 launch f1tenth_stack bringup_launch.py` is the shared **foundation layer**: hardware drivers plus arbitration, and nothing that can drive the car on its own. It starts `joy_node`, the full VESC chain, the LiDAR, and `ackermann_mux` — but deliberately no control layer — so running it by itself brings the hardware up and then just sits there; nothing publishes to `/teleop` or `/drive` until you launch something on top of it. See "Control layers" below for what runs on top, and [operations.md](operations.md) for exact commands.

**After:**

> `ros2 launch f1tenth_stack bringup_launch.py` starts the **foundation layer** — the hardware drivers, plus the referee that decides which commands reach the motor.
>
> It starts four things: `joy_node` (reads the gamepad), the VESC chain (talks to the motor controller), the LiDAR, and `ackermann_mux` (the referee).
>
> **It cannot move the car by itself, and that's deliberate.** Run it alone and the hardware wakes up, then nothing happens. No driving command exists yet, because nothing is publishing to `/teleop` or `/drive`.
>
> To actually drive, you launch a second thing on top of it — see [Control layers](#control-layers) below, or [operations.md](operations.md) for the exact commands.

What changed, and why each change matters:

| Change | Why |
|---|---|
| Four paragraphs instead of one | Each holds one fact the reader can absorb and check off |
| `joy_node`, VESC, `ackermann_mux` each glossed inline | A beginner has no idea what a "VESC chain" is; three words fixes it |
| "arbitration" → "the referee that decides which commands reach the motor" | Same meaning, no lookup required |
| The can't-move fact got its own bolded paragraph | It's the surprising part, and the part people misread |
| "just sits there" → "the hardware wakes up, then nothing happens" | Tells them what they'll *see*, so they don't think it's broken |

**How to find these:** if a sentence contains an em-dash *and* a semicolon, or runs past about 40 words, it's usually two or three sentences wearing a trenchcoat. The checker flags paragraphs over 200 characters as a starting point, but the real test is reading it aloud — if you run out of breath, split it.

**What not to do:** don't split a paragraph into choppy fragments to game the line length. The goal is one complete idea per paragraph, not short paragraphs.

## 3. Glossing jargon

First use of a term in each doc gets a plain-language gloss. Not the third use, not a link on its own — the first use, in place, because a beginner who has to leave the page to understand a sentence often doesn't come back.

Three ways to do it, roughly in order of preference:

**Inline parenthetical** — best for short glosses:

> `ackermann_mux` (the "mux", short for multiplexer — it picks which of several incoming drive commands actually reaches the motor)

**Following sentence** — best when the gloss needs more room:

> The car localizes with a **particle filter**. That means it keeps a few thousand guesses about where it might be on the map, scores each guess against what the LiDAR currently sees, and keeps the guesses that match.

**Glossary link** — for terms used across many docs, in addition to one of the above:

> ...the **deadman** ([glossary](glossary.md#deadman)) button...

Terms in this workspace that consistently need glossing on first use: node, topic, publish/subscribe, launch file, package, workspace, sourcing, mux/arbitration, deadman, odometry/odom, TF/frame, SLAM, localization, occupancy grid, particle filter, scan, Ackermann steering, lookahead, curvature, latch, watchdog, teleop.

Keep `docs/glossary.md` as the single definition of record and link to it, so the definitions don't drift across twenty files. `assets/glossary-seed.md` has starting definitions for all of the above.

## 4. Concrete before abstract

Lead with what the reader does and sees. Explain the mechanism after they have something to attach it to.

**Before** (mechanism first — reads fine if you already know why you'd want this):

> Phase 4 generates a velocity profile by computing curvature from three consecutive points, deriving cornering speed from a simplified friction circle, then applying forward/backward smoothing passes.

**After:**

> Phase 4 decides **how fast to drive at every point on the lap**.
>
> The idea is the one you'd use yourself: slow for the tight corners, fast on the straights, and start braking *before* the corner rather than at it.
>
> It works that out in three steps — measure how sharp each part of the track is, convert sharpness into a safe speed, then smooth the result so the braking starts early enough to be physically possible.
>
> Each step is detailed below.

Same information. The beginner now has a mental model to hang the math on, and the expert lost nothing but four seconds.

## 5. Marking depth as skippable

The device that lets one file serve both readers. **Depth goes inside a collapsed
`<details>` block**, so the page reads short and expands only for the reader who
wants it:

```markdown
### Cornering speed

The car slows down for corners. The tighter the corner, the slower it goes.

<details>
<summary><b>The math behind this</b> — click to expand. Skip it unless you're
tuning <code>a_lat_max</code>; nothing later in this doc depends on it.</summary>

[...the existing derivation, untouched...]

</details>
```

GitHub renders this folded. The beginner sees a one-line summary and moves on;
the expert clicks once and gets everything that was always there.

**The mechanics matter, and they are easy to get wrong:**

- **Blank line after `</summary>`, and before `</details>`.** Without them GitHub
  will not render the Markdown inside — your tables and code fences come out as
  raw text. This is the single most common mistake with this pattern.
- **No blank line between `<details>` and `<summary>`.** That one breaks the fold.
- Use `<b>` and `<code>` inside `<summary>`, not `**` and backticks. Markdown is
  not processed inside the summary line itself.
- Keep the summary to one or two lines. It is a signpost, not an abstract.

**What the summary has to say.** Always both of these:

1. What's inside, concretely — "the math behind this", "why we didn't use MPC",
   "the full wire format".
2. Whether skipping is safe — and say it as permission, not warning. A beginner
   who hits unexplained math assumes the problem is them and closes the file.
   Being told that skipping is the correct move keeps them reading.

If a later section genuinely does depend on the contents, say so instead:
`<summary><b>How the velocity profile is computed</b> — you'll need this to
follow the tuning section below.</summary>`

Rules for applying it:

- **The content inside does not change.** This is signposting, not editing.
  Resist the urge to "clean up" the math while you're in there.
- Good candidates: derivations, postmortems, "why we didn't do X instead",
  performance measurements, protocol and wire-format details, historical context,
  exhaustive parameter tables.
- **Bad candidates: anything about safety**, anything needed to run a command
  correctly, and anything a reader hits before the doc's main payoff. A folded
  safety rule is a safety rule nobody reads. Safety content is never collapsed.
- Don't nest `<details>` inside `<details>`. If you want to, the section needs
  restructuring instead.
- For a very long doc, a `# Part 2 — the technical detail` split can carry the
  same load. Use whichever leaves the top of the file readable.

## 6. Command blocks

The highest-value fix for a robotics beginner, because a terminal full of ROS log output looks identical whether things went right or wrong.

Every command block gets, at minimum, a terminal label and a success signal.

**Before** — a bare block, which tells a beginner nothing about what happens next:

    ```bash
    ros2 launch f1tenth_stack bringup_launch.py
    ```

**After:**

> **Terminal 1** — the hardware. Leave this running.
>
> ```bash
> ros2 launch f1tenth_stack bringup_launch.py
> ```
>
> **Working when:** the output stops scrolling and settles, and you see no repeating red `ERROR` lines. The car will sit still and do nothing — that's correct, not a failure.
>
> **If it doesn't:** a permission error on `/dev/sensors/vesc` usually means this terminal was opened before your user was added to the `dialout` group — open a fresh one. See [troubleshooting.md](troubleshooting.md).

For multi-terminal workflows — which is most of the interesting ones here — number the terminals consistently across the whole doc and repeat the label on every block. A beginner following a mapping procedure has three terminals open and no memory of which is which.

If a procedure has an order that matters, say so plainly at the top: **"Run these in order; terminal 1 must be up before terminal 2."** Ordering constraints that live only in the sequence of headings get missed.

## 7. Safety writing

Different rules apply. Everywhere else, the goal is to be easy to read. Here, the goal is to be **impossible to misread**, which sometimes means being blunter and uglier than the surrounding prose.

The pattern, in this order:

1. **The rule**, in one short sentence, in bold.
2. **The consequence** of breaking it, concretely.
3. **Why it exists** — one sentence.
4. Only then, any nuance or detail.

**Example:**

> **Hold LB on the gamepad the entire time the car is moving. Let go and it stops.**
>
> If you set `enable_deadman: false` to avoid holding the button, the car can drive off under its own power with no way to stop it short of physically catching it or killing the terminal.
>
> The rule exists because a driving node with a bug does not know it has a bug. LB is the one stop that doesn't depend on the code being correct.
>
> This applies to every node that can move the car, including one you wrote five minutes ago and are "just testing".

Point 3 matters more than it looks. A rule without a reason gets rationalized away by exactly the person under time pressure. A rule *with* a reason survives contact with "it'll be fine, it's just one lap".

Things to hold to when editing near safety content:

- Never trade an unambiguous sentence for a smoother one.
- Never move a safety warning below the command it's warning about.
- Never let a warning turn into a parenthetical or a footnote.
- If a safety rule is repeated in five docs, that repetition is deliberate — leave it.

## 8. The Highlights block

Every doc for a thing you can *run or use* — the dashboard, both simulators, the
diagnostics recorder, the cameras, the calibration wizard, each driving node,
each package README — carries a short block near the top saying what it is and
what's genuinely good about it.

This is **written to be read by an outsider**: someone at a competition, another
team, a judge, or somebody deciding whether this project is worth their attention.
It goes after the header block and the one-paragraph description, before the
setup instructions.

```markdown
## Highlights

- **Streams map updates as deltas, not frames.** A full occupancy grid is ~4 MB;
  after the first send, updates are typically under 2 KB.
- **Live parameter tuning with no rebuild.** Change a driving node's parameters
  from the browser and the running node picks them up immediately.
- **Read-only by construction.** Subscribes to topics, publishes to none, so it
  cannot move the car and is safe to leave running during a race.
- **No install on the viewing machine.** Any browser on the network.
```

**What makes one of these good:**

- **Lead with the concrete, specific thing.** "Streams map updates as deltas"
  beats "efficient networking". A number beats an adjective every time.
- **Every claim must be true and checkable against this repo.** These are the
  lines most likely to be quoted somewhere embarrassing. If you can't point at
  the code or a measurement that backs a bullet, cut it.
- **Bold the claim, then explain it in one sentence.** Skimmable at two speeds.
- **Four to eight bullets.** A list of fifteen is not highlights, it's an index.
- **Don't hide the limits.** If something only works on a saved map, or only at
  low speed, that belongs here too — an outsider who discovers a caveat later
  trusts the rest of the document less.

Follow it, where it helps, with a short **"Why it exists"** paragraph in the same
outward-facing register: what the situation was without this thing, and what it
changed. Two or three sentences. That paragraph is what somebody actually quotes
when they explain the project to another person.

**Do not write marketing.** No "powerful", "seamless", "robust", "cutting-edge",
"state-of-the-art". The reader this is aimed at discounts those words to zero and
starts wondering what's being covered up. Specifics are the only thing that reads
as confident.

## 9. Smaller habits

**Second person, active voice.** "You'll see the map appear", not "the map will be observed to appear".

**Say what things are for before what they're called.** "the referee that picks which drive command wins (`ackermann_mux`)" beats "`ackermann_mux` arbitrates".

**Number anything sequential.** If it's steps, it's a numbered list, even when it's two steps.

**Tables for anything with more than three parallel items.** Parameter lists, topic lists, and package lists are unreadable as prose and this repo already knows it — keep that up.

**Link with meaningful text.** "see [the safety model](architecture.md#the-safety-model)", not "see [here](architecture.md)". Beginners navigate by scanning link text.

**Don't apologize for complexity or hedge.** "This is a bit tricky but bear with me" adds anxiety and no information. Just explain it.

**Prefer a diagram or a small table to a long prose description of a structure.** The topic graph in `architecture.md` is worth more than any paragraph describing it.

**Keep the reading level plain, not childish.** The reader is a capable engineer who lacks domain context, not someone who needs simple words. Explain the robotics; don't dumb down the writing.
