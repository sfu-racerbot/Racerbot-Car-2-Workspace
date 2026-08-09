# <Title: the thing this doc is about, in plain words>

> **Who this is for:** <the reader, concretely — "someone about to run autonomy for the first time">
> **Read first:** [<prerequisite doc>](<link>) — <the one concept they need from it>
> **You'll be able to:** <the payoff, as a capability>
> **Time:** <rough estimate, or delete this line for reference docs>

<One or two sentences: what this is and why anyone would care. No mechanism yet.>

## Contents

<Only if the doc is over ~200 lines. Otherwise delete this section.>

## What this actually is

<The plain-language orientation. Assume they've never seen a robot. Concrete before
abstract: what it does for them, then how it works. This is the section a beginner
reads twice, so it's worth the effort.>

## Before you start

<Prerequisites as a checklist: what must be running, plugged in, built, or sourced.
If there aren't any, delete this section rather than writing "none".>

- [ ] <thing>
- [ ] <thing>

## <Doing the thing>

**Run these in order.** <Delete if order doesn't matter.>

**Terminal 1** — <what this terminal is for>. Leave it running.

```bash
<command>
```

**Working when:** <what they see on success — be concrete about the output>

**If it doesn't:** <the most likely failure and its fix, or a link to troubleshooting.md>

**Terminal 2** — <what this one is for>.

```bash
<command>
```

**Working when:** <success signal>

## Deep dive: <the part most readers can skip>

> **Skip this unless** <the specific reason someone would need it>. Nothing later
> in this doc depends on it.

<The math, the derivation, the postmortem, the rejected alternatives. Full depth —
this is where the real knowledge goes, and it is not abridged just because the
doc has a beginner on-ramp.>

## Safety

<Only for docs about anything that can move the car. Use the pattern from
references/standard.md#7-safety-writing: rule, consequence, why, then nuance.
Delete this section entirely if nothing here can move the car — an empty safety
section teaches people to skip real ones.>

**<The rule, one short bold sentence.>**

<What happens if you break it, concretely.>

<Why the rule exists, one sentence.>

## Troubleshooting

<Problems specific to this doc's workflow. General ones go in troubleshooting.md
and get linked, not copied.>

| Symptom | Likely cause | Fix |
|---|---|---|
| | | |

## See also

- [<related doc>](<link>) — <why you'd go there next>
