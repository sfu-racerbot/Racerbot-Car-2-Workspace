# Structure, index, and where things live

How the docs set is organized, and the rules for keeping it that way. Read this when you're touching the index, adding a doc, or working out where something belongs.

## The one structural decision

**Filenames stay where they are.** No `tutorials/`/`how-to/`/`reference/` subfolders, no numbered prefixes. The paths are referenced from `CLAUDE.md`, from commit messages, from code comments, and from every existing cross-link, and moving them buys a beginner nothing that a good index doesn't buy more cheaply.

What replaces the reshuffle is **an index that states a reading order**, plus **per-doc header blocks that state prerequisites**. Together those give a beginner the thing a folder hierarchy was supposed to give them — "where am I, and what do I read next" — without breaking anything.

## `docs/README.md` is the canonical index

One file is the index of record. It has three parts, in this order:

### Part 1 — Start here

An explicitly numbered path for someone brand new, with a sentence on why each step comes where it does. Not the full list — the *path*. Something like:

```markdown
## Brand new? Read these in order.

1. [glossary.md](glossary.md) — the vocabulary. Skim it now, come back when a word bites.
2. [concepts.md](concepts.md) — what ROS2, a node, a topic, and `colcon build` actually are.
3. [operations.md](operations.md#manual-driving-teleop) — drive the car by hand. Do this before anything autonomous.
4. [architecture.md](architecture.md) — what talks to what, and the safety model. Required before you write driving code.
5. [adding-your-own-code.md](adding-your-own-code.md) — where your code goes and what it must have.
```

Five to seven steps, no more. The point is a ladder, not a syllabus.

### Part 2 — The full list, grouped by what you're trying to do

Group by the reader's intent, not by package or by age:

- **Learn how it works** — concepts, architecture, glossary
- **Do a thing** — operations, run-diagnostics, git-setup
- **Write code** — adding-your-own-code, writing-your-own-node, drive-intent
- **Go deep on a subsystem** — racing-autonomy, simulator, ros-simulator, web-dashboard, sim-fidelity-audit
- **Look something up** — hardware-reference, troubleshooting, odom-calibration
- **Hardware add-ons** — realsense-camera, usb-camera-livestream

Each entry gets one line saying what's in it *and* who it's for. The existing top-level README table is a good model for the "what's in it" half; it's missing the "who for" half.

### Part 3 — The map of everything else

Short pointers to `src/<package>/README.md` files, `CLAUDE.md`, and `CHANGELOG.md`, so a reader knows those exist.

## The top-level `README.md` links to the index; it does not duplicate it

Right now the full doc table lives in the top-level README. Once `docs/README.md` exists, the top-level README keeps:

- What this repo is, in a few lines
- Code provenance
- **A short pointer to `docs/README.md`** — "Full documentation index and a reading order for newcomers: [docs/README.md](docs/README.md)"
- The `src/` package table (this belongs here — it's about the repo layout, not the docs)
- Quick start
- The notes/gotchas section

**Two copies of the same table will drift, and the stale one will be the one a beginner reads.** One index, one place.

## Per-doc header blocks are the prerequisite graph

Each doc's `Read first:` line is one edge in the dependency graph. Kept honest across the set, they mean a reader who lands in the middle of the docs from a search result can always walk backward to solid ground.

When you add or edit a header block, check that the `Read first` target actually covers what this doc assumes. A `Read first` pointing at a doc that doesn't explain the needed concept is worse than none.

## Adding a new doc

1. Write it to the standard, starting from `assets/doc-template.md`.
2. Add it to `docs/README.md` — in the grouped list, and in the "Start here" path only if a newcomer genuinely needs it early.
3. Add a `Read first:` line pointing at its prerequisite.
4. If it documents a package, cross-link with that package's own README rather than duplicating it (see below).
5. Name it `kebab-case.md`, describing the subject rather than the package (`racing-autonomy.md`, not `pure-pursuit-docs.md`).

The checker flags docs missing from the index, so step 2 doesn't rely on memory.

## `docs/<topic>.md` vs `src/<package>/README.md`

The split this repo already uses, worth keeping deliberate:

| | `docs/<topic>.md` | `src/<package>/README.md` |
|---|---|---|
| Audience | Someone using or operating the system | Someone reading or modifying that package's code |
| Contains | Workflow, concepts, why it works this way, how to run it | Module layout, the algorithm as implemented, parameters, how to test it |
| Beginner needs | Yes — this is the on-ramp | Later, once they're editing code |

Both get header blocks. Both are in scope for this standard — a package README is often the first code-level doc a new member reads, and "you already know the architecture" is exactly the assumption to avoid there.

A package README should open by saying **what the package does in one sentence and who should care**, then link to the `docs/` topic doc for the workflow rather than restating it. Duplication between the two is the main source of staleness here.

## What a package README should have

```markdown
# <package_name>

> **Who this is for:** someone about to read or change this package's code.
> **Read first:** [docs/architecture.md](../../docs/architecture.md)
> **What's in it:** how the node is put together, its parameters, and how to test it.

One sentence: what this package does.

## What it does
Two or three paragraphs, plain language, concrete before abstract.

## Files
A table: each module, one line on its job.

## Parameters
A table: name, default, units, what it does, and what happens if you get it wrong.

## Running it
Command blocks with terminal labels and success signals.

## Testing
How to test it without the car, if that's possible for this package.

## Safety notes
For any package that can move the car — the deadman requirement, stated plainly.
```

Packages that can move the car (`gap_follow`, `pure_pursuit`, anything new that publishes `/drive`) must carry the safety section. That's not documentation garnish — it's where someone reading the code learns the contract they're bound by.
