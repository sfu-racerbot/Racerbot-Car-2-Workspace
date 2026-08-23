# Test quality audit — 2026-08-22

Audit of every test in the eight first-party packages against
[TEST_QUALITY_STANDARDS.md](TEST_QUALITY_STANDARDS.md).
`tools/f1tenth_sim/` is in the standard's scope but contains no pytest functions.

**Scope:** 804 test functions in 42 files (900 collected — pytest expands parametrised cases).
Baseline: all 900 pass, sourced, in 31s.

## How each test was checked

1. **Static pass** — every test function parsed with `ast`; assertions, docstrings, decorators
   and surrounding prose classified against the A1–A9 checklist. Detector output was calibrated
   by reading the code: the first A3 pass flagged 306 tests, of which 286 were explained by
   oracle prose the standard explicitly accepts, leaving 20.
2. **Coverage-guided mutation** — `coverage.py` with per-test contexts built a map of
   *which tests execute which source function* (384 functions with at least one covering test).
   Each function then had its body replaced with `raise NotImplementedError`, and **only its
   covering tests** were run. 384 mutants, ~2h.
3. **Targeted `return <constant>` mutants** — §0 case 2, the shape a raising stub cannot catch,
   applied by hand to the units behind every test the static pass flagged as weak.

Mutation ran in an isolated git worktree with `PYTHONPATH` forced ahead of the installed
symlink tree, because several test files import their package rather than inserting `sys.path`
— without that, mutations would have silently applied to nothing and every test would have
looked like a false positive. Every mutation was reverted; `git status` is clean.

## Result

| Verdict | Count | Meaning |
|---|---|---|
| KEEP | 556 | no anti-pattern; failed under mutation as expected |
| KEEP (nit) | 186 | real but low-severity style finding, mostly bare `pytest.approx` |
| KEEP (unverified) | 52 | assert on config/spec/assets, so no function mutant reaches them; not hollow |
| **REWRITE** | **10** | falsely passed a plausible wrong implementation |

**No test in the suite is hollow by the §0 stub swap: all 752 mutant-reachable tests failed
under at least one mutant.** The 10 rewrites all share one shape — they survive an
implementation that returns a *constant*. In every case a sibling test in the same file does
catch it, so no unit is unguarded; the weakness is in those assertions specifically.

### Nit breakdown

| Flag | Count |
|---|---|
| A5 — bare `pytest.approx`, no explicit tolerance/unit | 132 |
| A2 — asserts only shape or `None`-ness | 48 |
| A3 — numeric literal with no oracle stated nearby | 20 |
| A5 — lone `is not None` as the only assertion | 8 |
| A5 — assertion reachable only via a filter that may never match | 3 |
| A5 — no assertion in the test body | 3 |
| Non-negotiable — `time.sleep` used to sequence events | 2 |

### Per package

| Package | KEEP | nit | unverified | REWRITE |
|---|---|---|---|---|
| drive_intent | 54 | 23 | 0 | 1 |
| gap_follow | 138 | 56 | 7 | 0 |
| odom_calibration | 7 | 6 | 0 | 0 |
| pure_pursuit | 188 | 60 | 7 | 2 |
| race_diagnostics | 10 | 2 | 0 | 0 |
| racerbot_sim | 21 | 2 | 4 | 0 |
| usb_cam_stream | 15 | 3 | 0 | 0 |
| web_dashboard | 123 | 34 | 34 | 7 |

## Findings that are not about an individual test

These are gaps in *coverage*, which a per-test audit cannot express as a row.

1. **`pure_pursuit_node` and `auto_map_race_node` have no deadman deny-path test at all** —
   a §3 hard fail for a node that publishes `/drive`. `PurePursuitNode.joy_callback` is never
   executed by any test; in `_deadman_status` only the `enable_deadman=false` short-circuit runs,
   and all four deny branches are unexecuted. `test_opponent_integration.py:15` describes the gate
   as "already-covered" — the only coverage is `gap_follow`'s, a separately implemented gate in a
   different file. `gap_follow` covers 3 of 4 states; `deadman_button_missing` is unexecuted there.
2. **`racerbot_sim`'s independent collision detection is untested.** `sim_bridge.any_collision`,
   `body_contact` and `agent_state` have zero covering tests — the A7 remedy that exists precisely
   because gym's own flag never fires.
3. **`GapFollowNode._deadman_engaged` is dead code** — zero callers, zero coverage; the live gate
   is `_deadman_status()`. `CLAUDE.md` names `_deadman_engaged` as the copy-paste template for new
   driving nodes.
4. **The unsourced pytest command hides 177 of 900 tests** (gap_follow 70, pure_pursuit 89,
   usb_cam_stream all 18), including every deadman and watchdog test. Fixed in `CLAUDE.md`.
5. **The standard's own "three mock usages" claim is a false positive.** There are **zero** mocks;
   its A1 grep matches `patch(` inside test *names* such as `..._forces_a_keyframe_not_a_patch():`.
   The 11-of-11 `enable_deadman:=false` ↔ `drive_topic` pairing claim does hold (re-verified).
6. **The `/drive_intent` §4 contract is fully met** in both packages, ordering included.

## The 10 rewrites

Each falsely passed the mutant named. All are `KEEP`-able as-is in the sense that a sibling
catches the bug — but each of these assertions, alone, cannot fail for the reason it claims.

| test | file:line | mutant it survived | fix |
|---|---|---|---|
| `test_velocity_profile_never_exceeds_braking_capability` | pure_pursuit/test_racing_math.py:244 | `compute_velocity_profile -> full(v_max)` | assert at least one braking segment was examined |
| `test_friction_ellipse_profile_still_respects_braking_capability` | pure_pursuit/test_racing_math.py:634 | same | same |
| `test_a_payload_from_a_newer_schema_is_refused_not_half_drawn` | web_dashboard/test_intent_protocol.py:101 | `validate -> 'constant rejection'` | assert *which* reason |
| `test_a_missing_severity_is_refused` | web_dashboard/test_intent_protocol.py:105 | same | assert *which* reason |
| `test_a_path_of_the_wrong_shape_is_refused` | web_dashboard/test_intent_protocol.py:111 | same | assert *which* reason |
| `test_a_nan_from_a_printf_style_publisher_is_refused` | web_dashboard/test_intent_protocol.py:117 | same | assert *which* reason |
| `test_an_oversized_path_is_refused_before_it_reaches_a_phone` | web_dashboard/test_intent_protocol.py:126 | same | assert *which* reason |
| `test_validate_rejects_a_non_object` | drive_intent/test_schema.py:220 | same | assert *which* reason |
| `test_process_state_message_is_json_serialisable` | web_dashboard/test_proccontrol.py:447 | `process_state_message -> None` | assert the round-tripped payload, not just that `json.dumps` ran |
| `test_thinning_tolerates_a_missing_or_empty_path` | web_dashboard/test_protocol_encoding.py:199 | `thin_intent_payload -> {}` | assert the returned payload's contents |

## Resolution — 2026-08-23

All 10 were rewritten. Each was then re-run against the exact mutant it had previously
survived and **confirmed to fail**, per Hard rule 1 of the new `CLAUDE.md` section:

| Mutant | Tests that now fail against it |
|---|---|
| `compute_velocity_profile -> np.full(n, v_max)` | both braking-capability tests |
| `schema.validate -> 'constant rejection'` | all 6 refusal tests |
| `process_state_message -> None` | the JSON round-trip test |
| `thin_intent_payload -> {}` | the missing/empty-path test |

The fixes, by shape:

- **Braking invariants** — count the braking segments and assert `>= 2` after the loop, so
  the bound can no longer go unchecked against a constant profile. The threshold is the
  stadium's geometry (two corners), not the measured count of 9, which would be an A3
  change detector.
- **The six refusals** — assert *which* reason `validate` returned, built from the module's
  own constants (`SCHEMA_VERSION`, `SEVERITIES`, `MAX_PATH_POINTS`) rather than pasted text.
- **JSON round-trip** — decode the encoded message and assert its contents; a bare
  `json.dumps()` is satisfied by `None` and `{}` alike.
- **Thinning pass-through** — assert `is payload`, which is the documented contract
  ("returns the payload unchanged (not a copy)") and pins both halves of it.

Nothing was weakened: every line removed was an `is not None` or a bare `json.dumps`, each
replaced by an exact-value assertion. Suite after the rewrites: **900 passed**.

The verdict column in the table below is the state *at audit time*; these 10 rows are now
resolved.

## Full per-test table

| # | test | file:line | verifies | anti-patterns | mutation check | verdict |
|---|---|---|---|---|---|---|
| 1 | `test_zero_steering_is_a_straight_line_along_the_heading` | drive_intent/test_predict.py:24 | Zero steering is a straight line along the heading | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 2 | `test_straight_line_respects_a_non_zero_starting_heading` | drive_intent/test_predict.py:31 | Straight line respects a non zero starting heading | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 3 | `test_quarter_circle_lands_exactly_where_geometry_says` | drive_intent/test_predict.py:38 | Kappa = tan(steer)/L = 0.5 -> radius 2m. A quarter turn from the | — | failed as expected (4/4 mutants) | **KEEP** |
| 4 | `test_right_turn_is_the_mirror_image_of_a_left_turn` | drive_intent/test_predict.py:57 | Right turn is the mirror image of a left turn | A5 (bare approx, no tolerance) | failed as expected (4/4 mutants) | **KEEP (nit)** |
| 5 | `test_curvature_from_steering_matches_the_bicycle_model` | drive_intent/test_predict.py:66 | Curvature from steering matches the bicycle model | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 6 | `test_a_nonsense_wheelbase_is_rejected_not_silently_used` | drive_intent/test_predict.py:72 | A nonsense wheelbase is rejected not silently used | — | failed as expected (1/1 mutants) | **KEEP** |
| 7 | `test_non_finite_steering_is_rejected` | drive_intent/test_predict.py:77 | Non finite steering is rejected | — | failed as expected (1/1 mutants) | **KEEP** |
| 8 | `test_first_sample_is_the_starting_pose` | drive_intent/test_predict.py:86 | First sample is the starting pose | A5 (bare approx, no tolerance) | failed as expected (4/4 mutants) | **KEEP (nit)** |
| 9 | `test_sample_count_and_speed_are_carried_through` | drive_intent/test_predict.py:91 | Sample count and speed are carried through | A5 (bare approx, no tolerance) | failed as expected (4/4 mutants) | **KEEP (nit)** |
| 10 | `test_a_stopped_car_intends_to_stay_exactly_where_it_is` | drive_intent/test_predict.py:97 | The arrow length is the point of the feature: zero speed must | A5 (bare approx, no tolerance) | failed as expected (5/5 mutants) | **KEEP (nit)** |
| 11 | `test_length_scales_with_speed_so_a_faster_plan_draws_a_longer_arrow` | drive_intent/test_predict.py:107 | Length scales with speed so a faster plan draws a longer arrow | A5 (bare approx, no tolerance) | failed as expected (5/5 mutants) | **KEEP (nit)** |
| 12 | `test_max_length_truncates_a_very_fast_plan` | drive_intent/test_predict.py:114 | Max length truncates a very fast plan | — | failed as expected (5/5 mutants) | **KEEP** |
| 13 | `test_max_length_leaves_a_short_plan_untouched` | drive_intent/test_predict.py:124 | Max length leaves a short plan untouched | — | failed as expected (4/4 mutants) | **KEEP** |
| 14 | `test_callbacks_receive_the_evolving_pose` | drive_intent/test_predict.py:129 | Pure_pursuit's arrow bends because it re-asks its steering law at | A5 (bare approx, no tolerance) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 15 | `test_a_steering_callback_that_corrects_cross_track_error_converges` | drive_intent/test_predict.py:144 | A crude proportional controller aiming back at y=0 should bring the | — | failed as expected (3/3 mutants) | **KEEP** |
| 16 | `test_fewer_than_two_samples_is_not_a_path` | drive_intent/test_predict.py:157 | Fewer than two samples is not a path | — | failed as expected (2/2 mutants) | **KEEP** |
| 17 | `test_a_nonsense_horizon_is_rejected` | drive_intent/test_predict.py:163 | A nonsense horizon is rejected | — | failed as expected (2/2 mutants) | **KEEP** |
| 18 | `test_a_non_finite_speed_from_a_callback_is_rejected` | drive_intent/test_predict.py:168 | A non finite speed from a callback is rejected | — | failed as expected (1/1 mutants) | **KEEP** |
| 19 | `test_to_body_rotates_and_translates_into_the_cars_own_frame` | drive_intent/test_predict.py:177 | To body rotates and translates into the cars own frame | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 20 | `test_to_body_of_the_origin_pose_is_the_origin` | drive_intent/test_predict.py:187 | To body of the origin pose is the origin | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 21 | `test_polar_to_body_adds_the_lidar_mounting_offset` | drive_intent/test_predict.py:192 | 0.33m is the real offset on this car; a gap target drawn without it | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 22 | `test_path_length_of_a_single_point_is_zero` | drive_intent/test_predict.py:204 | Path length of a single point is zero | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 23 | `test_a_built_payload_validates` | drive_intent/test_schema.py:28 | A built payload validates | A2 (shape/None only) | failed as expected (8/8 mutants) | **KEEP (nit)** |
| 24 | `test_payload_carries_the_schema_version_so_consumers_can_refuse_it` | drive_intent/test_schema.py:36 | Payload carries the schema version so consumers can refuse it | — | failed as expected (6/6 mutants) | **KEEP** |
| 25 | `test_path_points_are_reduced_to_the_three_fields_the_browser_draws` | drive_intent/test_schema.py:41 | Path points are reduced to the three fields the browser draws | A3 (unsourced number) | failed as expected (6/6 mutants) | **KEEP (nit)** |
| 26 | `test_a_three_tuple_path_treats_the_last_element_as_speed` | drive_intent/test_schema.py:46 | A three tuple path treats the last element as speed | A3 (unsourced number) | failed as expected (6/6 mutants) | **KEEP (nit)** |
| 27 | `test_reason_is_omitted_entirely_when_not_supplied` | drive_intent/test_schema.py:51 | The publisher attaches a reason only on state changes and on the | — | failed as expected (8/8 mutants) | **KEEP** |
| 28 | `test_a_reason_thunk_is_resolved_at_build_time` | drive_intent/test_schema.py:60 | A reason thunk is resolved at build time | — | failed as expected (6/6 mutants) | **KEEP** |
| 29 | `test_an_enormous_reason_is_truncated_rather_than_broadcast` | drive_intent/test_schema.py:65 | An enormous reason is truncated rather than broadcast | — | failed as expected (6/6 mutants) | **KEEP** |
| 30 | `test_an_absurd_path_is_truncated_at_build_time` | drive_intent/test_schema.py:70 | An absurd path is truncated at build time | — | failed as expected (6/6 mutants) | **KEEP** |
| 31 | `test_a_non_finite_number_raises_instead_of_emitting_invalid_json` | drive_intent/test_schema.py:76 | Json.dumps would happily write a bare NaN, which JSON.parse rejects | — | failed as expected (5/5 mutants) | **KEEP** |
| 32 | `test_encode_refuses_non_finite_values_even_if_one_slips_through` | drive_intent/test_schema.py:86 | Encode refuses non finite values even if one slips through | — | failed as expected (1/1 mutants) | **KEEP** |
| 33 | `test_encode_decode_round_trips` | drive_intent/test_schema.py:91 | Encode decode round trips | — | failed as expected (9/9 mutants) | **KEEP** |
| 34 | `test_encoding_is_compact_enough_for_a_20hz_stream` | drive_intent/test_schema.py:99 | Encoding is compact enough for a 20hz stream | — | failed as expected (8/8 mutants) | **KEEP** |
| 35 | `test_bind_min_marks_the_lowest_speed_ceiling` | drive_intent/test_schema.py:111 | Bind min marks the lowest speed ceiling | — | failed as expected (5/5 mutants) | **KEEP** |
| 36 | `test_bind_min_marks_every_member_of_a_tie` | drive_intent/test_schema.py:121 | Naming one of two equal limits as 'the' reason would be a lie of | — | failed as expected (4/4 mutants) | **KEEP** |
| 37 | `test_bind_min_does_not_mutate_its_input` | drive_intent/test_schema.py:129 | Bind min does not mutate its input | — | failed as expected (4/4 mutants) | **KEEP** |
| 38 | `test_bind_min_of_nothing_is_nothing` | drive_intent/test_schema.py:135 | Bind min of nothing is nothing | — | failed as expected (1/1 mutants) | **KEEP** |
| 39 | `test_binding_factor_is_none_when_no_constraint_is_active` | drive_intent/test_schema.py:139 | Binding factor is none when no constraint is active | A2 (shape/None only) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 40 | `test_a_zero_speed_command_is_always_a_stop` | drive_intent/test_schema.py:148 | A zero speed command is always a stop | A3 (unsourced number) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 41 | `test_ordinary_driving_states_are_not_flagged` | drive_intent/test_schema.py:153 | Ordinary driving states are not flagged | A3 (unsourced number) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 42 | `test_anything_out_of_the_ordinary_draws_the_eye` | drive_intent/test_schema.py:158 | Anything out of the ordinary draws the eye | A3 (unsourced number) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 43 | `test_severity_is_derived_from_the_commanded_speed_not_the_desired_one` | drive_intent/test_schema.py:163 | A safety override zeroes the command while the plan still wants | — | failed as expected (6/6 mutants) | **KEEP** |
| 44 | `test_an_explicit_severity_overrides_the_derived_one` | drive_intent/test_schema.py:170 | An explicit severity overrides the derived one | — | failed as expected (5/5 mutants) | **KEEP** |
| 45 | `test_an_expensive_reason_thunk_runs_at_most_once` | drive_intent/test_schema.py:180 | An expensive reason thunk runs at most once | — | failed as expected (2/2 mutants) | **KEEP** |
| 46 | `test_memoize_passes_plain_strings_straight_through` | drive_intent/test_schema.py:193 | Memoize passes plain strings straight through | — | failed as expected (1/1 mutants) | **KEEP** |
| 47 | `test_resolve_reason_of_none_is_none` | drive_intent/test_schema.py:197 | Resolve reason of none is none | A2 (shape/None only) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 48 | `test_a_future_schema_version_is_refused_rather_than_half_rendered` | drive_intent/test_schema.py:205 | A future schema version is refused rather than half rendered | — | failed as expected (7/7 mutants) | **KEEP** |
| 49 | `test_decode_rejects_garbage` | drive_intent/test_schema.py:211 | Decode rejects garbage | — | failed as expected (1/1 mutants) | **KEEP** |
| 50 | `test_validate_rejects_a_non_object` | drive_intent/test_schema.py:220 | Validate rejects a non object | A2 (shape/None only); A5 (lone `is not None`) | FALSELY PASSED on `return <const>` (stub killed it; sibling catches it) | **REWRITE** |
| 51 | `test_validate_requires_the_identifying_strings` | drive_intent/test_schema.py:225 | Validate requires the identifying strings | — | failed as expected (7/7 mutants) | **KEEP** |
| 52 | `test_validate_rejects_an_unknown_severity` | drive_intent/test_schema.py:231 | Validate rejects an unknown severity | — | failed as expected (7/7 mutants) | **KEEP** |
| 53 | `test_validate_requires_finite_numbers` | drive_intent/test_schema.py:240 | Validate requires finite numbers | — | failed as expected (7/7 mutants) | **KEEP** |
| 54 | `test_validate_rejects_a_nan_that_arrived_over_the_wire` | drive_intent/test_schema.py:246 | Python's json.loads accepts the non-standard bare NaN even though | — | failed as expected (8/8 mutants) | **KEEP** |
| 55 | `test_validate_rejects_a_malformed_path` | drive_intent/test_schema.py:255 | Validate rejects a malformed path | — | failed as expected (8/8 mutants) | **KEEP** |
| 56 | `test_validate_rejects_an_oversized_path_from_the_wire` | drive_intent/test_schema.py:269 | Validate rejects an oversized path from the wire | — | failed as expected (8/8 mutants) | **KEEP** |
| 57 | `test_validate_rejects_malformed_factors` | drive_intent/test_schema.py:275 | Validate rejects malformed factors | — | failed as expected (8/8 mutants) | **KEEP** |
| 58 | `test_validate_rejects_a_non_string_reason` | drive_intent/test_schema.py:285 | Validate rejects a non string reason | — | failed as expected (8/8 mutants) | **KEEP** |
| 59 | `test_target_is_a_labelled_body_frame_point` | drive_intent/test_schema.py:295 | Target is a labelled body frame point | — | failed as expected (3/3 mutants) | **KEEP** |
| 60 | `test_wedge_is_carried_through_and_validates` | drive_intent/test_schema.py:300 | Wedge is carried through and validates | A5 (bare approx, no tolerance) | failed as expected (8/8 mutants) | **KEEP (nit)** |
| 61 | `test_no_wedge_means_no_key` | drive_intent/test_schema.py:307 | No wedge means no key | — | failed as expected (6/6 mutants) | **KEEP** |
| 62 | `test_frame_defaults_to_base_link_so_the_arrow_renders_without_a_pose` | drive_intent/test_schema.py:311 | Robot-centric mode has no map and no localization at all; publishing | — | failed as expected (6/6 mutants) | **KEEP** |
| 63 | `test_stamp_can_be_supplied_for_deterministic_tests` | drive_intent/test_schema.py:318 | Stamp can be supplied for deterministic tests | A5 (bare approx, no tolerance) | failed as expected (6/6 mutants) | **KEEP (nit)** |
| 64 | `test_stamp_defaults_to_now` | drive_intent/test_schema.py:322 | Stamp defaults to now | — | failed as expected (6/6 mutants) | **KEEP** |
| 65 | `test_the_first_call_always_publishes` | drive_intent/test_throttle.py:25 | The first call always publishes | — | failed as expected (2/2 mutants) | **KEEP** |
| 66 | `test_publishing_is_capped_at_the_configured_rate` | drive_intent/test_throttle.py:29 | Gap_follow decides at scan rate (~40Hz) and pure_pursuit at | — | failed as expected (3/3 mutants) | **KEEP** |
| 67 | `test_a_refused_tick_does_not_reset_the_clock` | drive_intent/test_throttle.py:40 | A refused tick does not reset the clock | — | failed as expected (3/3 mutants) | **KEEP** |
| 68 | `test_a_non_positive_rate_means_every_tick_not_silence` | drive_intent/test_throttle.py:48 | A rate of 0 that silently published nothing would be a trap; the | — | failed as expected (3/3 mutants) | **KEEP** |
| 69 | `test_every_state_transition_carries_its_reason` | drive_intent/test_throttle.py:61 | The transition is the diagnostic event -- it is exactly the moment | — | failed as expected (2/2 mutants) | **KEEP** |
| 70 | `test_a_steady_state_repeats_its_reason_only_on_the_slow_period` | drive_intent/test_throttle.py:70 | A steady state repeats its reason only on the slow period | — | failed as expected (2/2 mutants) | **KEEP** |
| 71 | `test_a_zero_reason_period_means_transitions_only` | drive_intent/test_throttle.py:79 | A zero reason period means transitions only | — | failed as expected (2/2 mutants) | **KEEP** |
| 72 | `test_reset_makes_the_next_tick_publish_and_explain` | drive_intent/test_throttle.py:86 | A browser that just connected has no context at all; making it wait | — | failed as expected (4/4 mutants) | **KEEP** |
| 73 | `test_publish_and_reason_clocks_are_independent` | drive_intent/test_throttle.py:97 | Publish and reason clocks are independent | — | failed as expected (4/4 mutants) | **KEEP** |
| 74 | `test_a_single_failure_is_tolerated` | drive_intent/test_throttle.py:109 | A single failure is tolerated | — | failed as expected (2/2 mutants) | **KEEP** |
| 75 | `test_sustained_failure_trips_the_latch_exactly_once` | drive_intent/test_throttle.py:115 | Sustained failure trips the latch exactly once | — | failed as expected (2/2 mutants) | **KEEP** |
| 76 | `test_success_clears_the_count_so_one_bad_scan_never_accumulates` | drive_intent/test_throttle.py:125 | Success clears the count so one bad scan never accumulates | — | failed as expected (3/3 mutants) | **KEEP** |
| 77 | `test_success_after_the_latch_trips_does_not_re_enable_it` | drive_intent/test_throttle.py:135 | Once intent generation has proven broken for this configuration it | — | failed as expected (3/3 mutants) | **KEEP** |
| 78 | `test_a_latch_that_could_never_tolerate_anything_is_rejected` | drive_intent/test_throttle.py:147 | A latch that could never tolerate anything is rejected | — | failed as expected (1/1 mutants) | **KEEP** |
| 79 | `test_a_driving_tick_publishes_a_valid_intent` | gap_follow/test_gap_follow_intent.py:100 | A driving tick publishes a valid intent | — | failed as expected (64/65 mutants) | **KEEP** |
| 80 | `test_the_intent_reports_the_command_that_was_actually_published` | gap_follow/test_gap_follow_intent.py:112 | If these two could drift apart the arrow would be fiction, and the | — | failed as expected (62/63 mutants) | **KEEP** |
| 81 | `test_the_arrow_length_is_distance_covered_over_the_horizon` | gap_follow/test_gap_follow_intent.py:131 | The arrow's length is the feature: it has to mean 'this far, this | — | failed as expected (62/63 mutants) | **KEEP** |
| 82 | `test_the_intent_arrow_shows_the_plan_not_the_acceleration_ramp` | gap_follow/test_gap_follow_intent.py:142 | This is the distinction the whole feature rests on. On a clear | A5 (bare approx, no tolerance) | failed as expected (63/66 mutants) | **KEEP (nit)** |
| 83 | `test_the_ghost_path_is_what_grows_with_the_acceleration_ramp` | gap_follow/test_gap_follow_intent.py:158 | ...and the gap between the two is exactly the command shaping, | — | failed as expected (63/66 mutants) | **KEEP** |
| 84 | `test_the_path_speed_profile_is_what_drives_the_arrow_width` | gap_follow/test_gap_follow_intent.py:172 | The path speed profile is what drives the arrow width | — | failed as expected (62/63 mutants) | **KEEP** |
| 85 | `test_a_clear_corridor_names_a_gap_target_ahead_of_the_lidar` | gap_follow/test_gap_follow_intent.py:182 | A clear corridor names a gap target ahead of the lidar | — | failed as expected (62/63 mutants) | **KEEP** |
| 86 | `test_the_gap_wedge_spans_the_selected_gap` | gap_follow/test_gap_follow_intent.py:193 | The gap wedge spans the selected gap | A5 (bare approx, no tolerance) | failed as expected (62/63 mutants) | **KEEP (nit)** |
| 87 | `test_every_speed_ceiling_is_reported_and_exactly_one_group_binds` | gap_follow/test_gap_follow_intent.py:203 | Every speed ceiling is reported and exactly one group binds | A5 (bare approx, no tolerance) | failed as expected (62/63 mutants) | **KEEP (nit)** |
| 88 | `test_the_binding_factor_matches_the_speed_actually_commanded` | gap_follow/test_gap_follow_intent.py:216 | The whole diagnostic claim of the panel is 'this is the limit in | — | failed as expected (62/63 mutants) | **KEEP** |
| 89 | `test_a_deadman_release_publishes_a_stop_intent` | gap_follow/test_gap_follow_intent.py:231 | A deadman release publishes a stop intent | A5 (bare approx, no tolerance) | failed as expected (32/33 mutants) | **KEEP (nit)** |
| 90 | `test_a_stop_predicts_no_movement_at_all` | gap_follow/test_gap_follow_intent.py:243 | A stopped car intends to stay where it is; drawing a stub arrow | A5 (bare approx, no tolerance) | failed as expected (43/44 mutants) | **KEEP (nit)** |
| 91 | `test_a_stop_still_reports_where_the_rack_is_held` | gap_follow/test_gap_follow_intent.py:255 | Gap_follow deliberately holds the steering rack through a stop | — | failed as expected (65/66 mutants) | **KEEP** |
| 92 | `test_an_emergency_stop_names_its_state_not_just_a_stop` | gap_follow/test_gap_follow_intent.py:267 | An emergency stop names its state not just a stop | — | failed as expected (43/44 mutants) | **KEEP** |
| 93 | `test_a_state_transition_always_carries_its_reason` | gap_follow/test_gap_follow_intent.py:279 | The transition is the diagnostic event -- it is the moment someone | — | failed as expected (65/66 mutants) | **KEEP** |
| 94 | `test_a_steady_state_does_not_repeat_its_reason_every_tick` | gap_follow/test_gap_follow_intent.py:290 | The reason string can be expensive to build; it is attached on | — | failed as expected (63/64 mutants) | **KEEP** |
| 95 | `test_the_intent_reason_is_the_same_text_the_terminal_logs` | gap_follow/test_gap_follow_intent.py:301 | The intent reason is the same text the terminal logs | — | failed as expected (62/63 mutants) | **KEEP** |
| 96 | `test_a_broken_intent_builder_does_not_stop_the_car_driving` | gap_follow/test_gap_follow_intent.py:315 | Rule 2. If this ever regresses, a bad drawing takes the steering | — | failed as expected (52/62 mutants) | **KEEP** |
| 97 | `test_sustained_intent_failure_switches_intent_off_not_the_node` | gap_follow/test_gap_follow_intent.py:333 | Sustained intent failure switches intent off not the node | — | failed as expected (53/63 mutants) | **KEEP** |
| 98 | `test_the_drive_command_is_published_before_the_intent` | gap_follow/test_gap_follow_intent.py:348 | Rule 1. Nothing in the intent path may sit in front of a command, | — | failed as expected (64/65 mutants) | **KEEP** |
| 99 | `test_an_expensive_stop_reason_is_computed_at_most_once_per_tick` | gap_follow/test_gap_follow_intent.py:366 | Gap_follow's TTC stop reason re-runs the whole gap pipeline. The | — | failed as expected (44/57 mutants) | **KEEP** |
| 100 | `test_publish_intent_false_creates_no_publisher_at_all` | gap_follow/test_gap_follow_intent.py:382 | Publish intent false creates no publisher at all | — | failed as expected (50/50 mutants) | **KEEP** |
| 101 | `test_the_publish_rate_is_honoured` | gap_follow/test_gap_follow_intent.py:395 | The publish rate is honoured | — | failed as expected (63/64 mutants) | **KEEP** |
| 102 | `test_the_payload_stays_small_enough_to_stream` | gap_follow/test_gap_follow_intent.py:409 | ~1KB at 20Hz per client is fine over the LAN; an unbounded reason | — | failed as expected (61/62 mutants) | **KEEP** |
| 103 | `test_the_wire_format_is_strict_json` | gap_follow/test_gap_follow_intent.py:420 | Not just 'json.loads accepts it' -- Python tolerates bare NaN and | — | failed as expected (51/62 mutants) | **KEEP** |
| 104 | `test_a_value_inside_its_bounds_is_accepted` | gap_follow/test_gap_follow_live_tuning.py:60 | A value inside its bounds is accepted | — | failed as expected (6/6 mutants) | **KEEP** |
| 105 | `test_bounds_are_inclusive` | gap_follow/test_gap_follow_live_tuning.py:66 | Bounds are inclusive | — | failed as expected (6/6 mutants) | **KEEP** |
| 106 | `test_integers_are_accepted_as_floats` | gap_follow/test_gap_follow_live_tuning.py:77 | A slider landing exactly on 2 must not fail where 2.1 works | A2 (shape/None only) | failed as expected (6/6 mutants) | **KEEP (nit)** |
| 107 | `test_passthrough_names_are_ignored_not_refused` | gap_follow/test_gap_follow_live_tuning.py:83 | Passthrough names are ignored not refused | A2 (shape/None only) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 108 | `test_a_bool_tunable_round_trips` | gap_follow/test_gap_follow_live_tuning.py:88 | A bool tunable round trips | A2 (shape/None only) | failed as expected (6/6 mutants) | **KEEP (nit)** |
| 109 | `test_out_of_range_is_refused` | gap_follow/test_gap_follow_live_tuning.py:98 | Out of range is refused | — | failed as expected (2/2 mutants) | **KEEP** |
| 110 | `test_an_unknown_parameter_is_refused_rather_than_ignored` | gap_follow/test_gap_follow_live_tuning.py:104 | The whole reason this module exists -- a silently accepted change | — | failed as expected (1/1 mutants) | **KEEP** |
| 111 | `test_the_deadman_cannot_be_switched_off_at_runtime` | gap_follow/test_gap_follow_live_tuning.py:112 | The deadman cannot be switched off at runtime | — | failed as expected (1/1 mutants) | **KEEP** |
| 112 | `test_non_finite_is_refused` | gap_follow/test_gap_follow_live_tuning.py:119 | Non finite is refused | — | failed as expected (2/2 mutants) | **KEEP** |
| 113 | `test_a_bool_for_a_float_is_refused` | gap_follow/test_gap_follow_live_tuning.py:124 | A bool for a float is refused | — | failed as expected (2/2 mutants) | **KEEP** |
| 114 | `test_a_number_for_a_bool_is_refused` | gap_follow/test_gap_follow_live_tuning.py:129 | A number for a bool is refused | — | failed as expected (2/2 mutants) | **KEEP** |
| 115 | `test_a_string_is_refused` | gap_follow/test_gap_follow_live_tuning.py:134 | A string is refused | — | failed as expected (2/2 mutants) | **KEEP** |
| 116 | `test_one_bad_value_rejects_the_whole_batch` | gap_follow/test_gap_follow_live_tuning.py:143 | Half a speed change landing is its own hazard | — | failed as expected (2/2 mutants) | **KEEP** |
| 117 | `test_min_speed_cannot_exceed_max_speed` | gap_follow/test_gap_follow_live_tuning.py:150 | Min speed cannot exceed max speed | — | failed as expected (3/3 mutants) | **KEEP** |
| 118 | `test_a_batch_that_satisfies_the_invariant_together_is_accepted` | gap_follow/test_gap_follow_live_tuning.py:156 | Min_speed alone would be illegal; raising max_speed with it is not, | — | failed as expected (6/6 mutants) | **KEEP** |
| 119 | `test_the_forward_reserve_cannot_sit_inside_the_contact_floor` | gap_follow/test_gap_follow_live_tuning.py:164 | The forward reserve cannot sit inside the contact floor | — | failed as expected (4/4 mutants) | **KEEP** |
| 120 | `test_the_escape_creep_cannot_become_a_drive_speed` | gap_follow/test_gap_follow_live_tuning.py:171 | The creep is the one speed permitted inside the forward reserve, so | — | failed as expected (5/5 mutants) | **KEEP** |
| 121 | `test_the_stop_cone_cannot_be_wider_than_the_scan_window` | gap_follow/test_gap_follow_live_tuning.py:180 | The stop cone cannot be wider than the scan window | — | failed as expected (6/6 mutants) | **KEEP** |
| 122 | `test_the_shipped_config_is_inside_every_bound` | gap_follow/test_gap_follow_live_tuning.py:191 | A default outside its own tunable range would mean the panel opens | — | no mutant reached it | **KEEP (unverified)** |
| 123 | `test_the_shipped_config_satisfies_its_own_invariants` | gap_follow/test_gap_follow_live_tuning.py:204 | The shipped config satisfies its own invariants | A2 (shape/None only) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 124 | `test_parameters_that_must_never_be_browser_tunable` | gap_follow/test_gap_follow_live_tuning.py:215 | Parameters that must never be browser tunable | — | no mutant reached it | **KEEP (unverified)** |
| 125 | `test_every_tunable_is_declared_by_the_node` | gap_follow/test_gap_follow_live_tuning.py:219 | Every tunable is declared by the node | — | no mutant reached it | **KEEP (unverified)** |
| 126 | `test_every_tunable_attr_is_actually_assigned_by_the_node` | gap_follow/test_gap_follow_live_tuning.py:226 | Catches a typo in `attr`, which would otherwise make setattr() | — | failed as expected (1/1 mutants) | **KEEP** |
| 127 | `test_every_tunable_is_documented_and_bounded` | gap_follow/test_gap_follow_live_tuning.py:237 | Every tunable is documented and bounded | — | no mutant reached it | **KEEP (unverified)** |
| 128 | `test_safety_flagged_parameters_are_the_expected_ones` | gap_follow/test_gap_follow_live_tuning.py:244 | A change here should be deliberate: the flag drives the warning | — | no mutant reached it | **KEEP (unverified)** |
| 129 | `test_spec_json_describes_every_tunable` | gap_follow/test_gap_follow_live_tuning.py:260 | Spec json describes every tunable | — | failed as expected (2/2 mutants) | **KEEP** |
| 130 | `test_spec_is_parseable_by_the_dashboard` | gap_follow/test_gap_follow_live_tuning.py:272 | The producer and the consumer live in different packages and can | — | failed as expected (3/3 mutants) | **KEEP** |
| 131 | `test_the_generic_half_matches_pure_pursuits_copy` | gap_follow/test_gap_follow_live_tuning.py:292 | This workspace duplicates rather than imports across packages (see | — | no mutant reached it | **KEEP (unverified)** |
| 132 | `test_refuses_to_drive_before_any_joy_message` | gap_follow/test_gap_follow_node.py:93 | Refuses to drive before any joy message | — | failed as expected (18/31 mutants) | **KEEP** |
| 133 | `test_refuses_to_drive_when_the_deadman_is_released` | gap_follow/test_gap_follow_node.py:101 | Refuses to drive when the deadman is released | — | failed as expected (19/32 mutants) | **KEEP** |
| 134 | `test_stops_when_the_joy_stream_goes_stale` | gap_follow/test_gap_follow_node.py:110 | Stops when the joy stream goes stale | — | failed as expected (19/32 mutants) | **KEEP** |
| 135 | `test_drives_forward_down_an_open_corridor` | gap_follow/test_gap_follow_node.py:124 | Drives forward down an open corridor | — | failed as expected (51/62 mutants) | **KEEP** |
| 136 | `test_every_command_stays_inside_the_physical_envelope` | gap_follow/test_gap_follow_node.py:134 | Every command stays inside the physical envelope | — | failed as expected (52/63 mutants) | **KEEP** |
| 137 | `test_steers_away_from_an_obstacle_on_one_side` | gap_follow/test_gap_follow_node.py:167 | Steers away from an obstacle on one side | — | failed as expected (54/65 mutants) | **KEEP** |
| 138 | `test_steering_slews_at_the_configured_rate_and_no_faster` | gap_follow/test_gap_follow_node.py:188 | Steering slews at the configured rate and no faster | — | failed as expected (52/63 mutants) | **KEEP** |
| 139 | `test_a_transient_stop_does_not_cost_the_car_its_steering` | gap_follow/test_gap_follow_node.py:210 | Regression: the 2026-07-27 wall collision | — | failed as expected (56/67 mutants) | **KEEP** |
| 140 | `test_braking_mid_run_holds_the_rack_where_it_is` | gap_follow/test_gap_follow_node.py:241 | Braking is not a reason to throw away the turn the car is mid-way | A5 (bare approx, no tolerance) | failed as expected (56/67 mutants) | **KEEP (nit)** |
| 141 | `test_recovery_never_depends_on_cycling_the_deadman` | gap_follow/test_gap_follow_node.py:260 | Nothing in the stop path may need an operator to notice the car and | — | failed as expected (57/68 mutants) | **KEEP** |
| 142 | `test_a_blocked_forward_cone_crawls_out_instead_of_latching` | gap_follow/test_gap_follow_node.py:294 | Regression: the deadlock that stranded the car on 2026-07-27 | — | failed as expected (50/61 mutants) | **KEEP** |
| 143 | `test_the_crawl_never_exceeds_its_stopping_distance` | gap_follow/test_gap_follow_node.py:316 | The creep spends at most half the forward reserve, so it still has | — | failed as expected (50/61 mutants) | **KEEP** |
| 144 | `test_a_dead_end_still_stops_the_car_dead` | gap_follow/test_gap_follow_node.py:333 | The creep must not become a licence to drive into a wall. With no gap | — | failed as expected (42/55 mutants) | **KEEP** |
| 145 | `test_close_obstacle_stops_immediately_without_ramping_down` | gap_follow/test_gap_follow_node.py:350 | Close obstacle stops immediately without ramping down | — | failed as expected (56/67 mutants) | **KEEP** |
| 146 | `test_stops_when_odometry_goes_stale_while_ttc_is_enabled` | gap_follow/test_gap_follow_node.py:364 | Stops when odometry goes stale while ttc is enabled | — | failed as expected (21/34 mutants) | **KEEP** |
| 147 | `test_ttc_brakes_at_speed_but_not_at_a_crawl` | gap_follow/test_gap_follow_node.py:385 | The TTC brake is armed only above ttc_min_brake_speed | — | failed as expected (55/66 mutants) | **KEEP** |
| 148 | `test_the_crawl_case_is_the_gate_and_not_the_geometry` | gap_follow/test_gap_follow_node.py:413 | Guard against the test above passing for an unrelated reason | — | failed as expected (44/57 mutants) | **KEEP** |
| 149 | `test_stops_on_a_malformed_scan_rather_than_guessing` | gap_follow/test_gap_follow_node.py:427 | Stops on a malformed scan rather than guessing | — | failed as expected (21/34 mutants) | **KEEP** |
| 150 | `test_invalid_returns_do_not_fake_an_emergency_stop` | gap_follow/test_gap_follow_node.py:437 | Invalid returns do not fake an emergency stop | — | failed as expected (51/62 mutants) | **KEEP** |
| 151 | `test_speed_ramp_starts_from_measured_speed_not_the_last_command` | gap_follow/test_gap_follow_node.py:454 | Speed ramp starts from measured speed not the last command | — | failed as expected (53/64 mutants) | **KEEP** |
| 152 | `test_a_wild_odometry_reading_cannot_exceed_the_speed_ceiling` | gap_follow/test_gap_follow_node.py:466 | A wild odometry reading cannot exceed the speed ceiling | — | failed as expected (44/57 mutants) | **KEEP** |
| 153 | `test_centering_steers_off_a_wall_it_is_running_parallel_to` | gap_follow/test_gap_follow_node.py:499 | The behaviour being fixed. In a corridor wide enough that the free | — | failed as expected (53/64 mutants) | **KEEP** |
| 154 | `test_centering_is_symmetric` | gap_follow/test_gap_follow_node.py:512 | Centering is symmetric | — | failed as expected (53/64 mutants) | **KEEP** |
| 155 | `test_centering_is_silent_when_already_centred` | gap_follow/test_gap_follow_node.py:519 | Centering is silent when already centred | — | failed as expected (53/64 mutants) | **KEEP** |
| 156 | `test_centering_never_exceeds_its_authority_bound` | gap_follow/test_gap_follow_node.py:526 | Hard against one wall, the bias is still only a bias | — | failed as expected (53/64 mutants) | **KEEP** |
| 157 | `test_without_centering_the_car_holds_its_offset` | gap_follow/test_gap_follow_node.py:534 | Same scan with the term switched off: bearing steering sees a target | — | failed as expected (52/63 mutants) | **KEEP** |
| 158 | `test_narrow_corridor_still_uses_the_midpoint_aim` | gap_follow/test_gap_follow_node.py:544 | Where both gap edges are real obstacles the midpoint already steers off | — | failed as expected (53/64 mutants) | **KEEP** |
| 159 | `test_centering_does_not_disturb_the_open_road` | gap_follow/test_gap_follow_node.py:553 | No walls in range is not a corridor; the car must go straight | — | failed as expected (53/64 mutants) | **KEEP** |
| 160 | `test_centering_refuses_to_start_with_too_much_authority` | gap_follow/test_gap_follow_node.py:561 | A bias able to cancel the chosen gap is a second driving policy | — | failed as expected (3/3 mutants) | **KEEP** |
| 161 | `test_adaptive_width_margin_is_a_no_op_by_default_even_on_a_narrow_straight` | gap_follow/test_gap_follow_node.py:579 | Shipped behaviour (config/gap_follow.yaml): min_safety_margin | A5 (bare approx, no tolerance) | failed as expected (53/64 mutants) | **KEEP (nit)** |
| 162 | `test_adaptive_width_leaves_the_margin_alone_on_a_sensed_wide_corridor` | gap_follow/test_gap_follow_node.py:593 | Adaptive width leaves the margin alone on a sensed wide corridor | A5 (bare approx, no tolerance) | failed as expected (53/64 mutants) | **KEEP (nit)** |
| 163 | `test_adaptive_width_margin_still_scales_linearly_when_configured_to_shrink` | gap_follow/test_gap_follow_node.py:600 | The interpolation itself is still correct if a future per-course | — | failed as expected (53/64 mutants) | **KEEP** |
| 164 | `test_disabling_adaptive_width_holds_the_static_corner_speed_on_a_wide_bend` | gap_follow/test_gap_follow_node.py:612 | Margin is unaffected by enable_adaptive_width either way under the | A5 (bare approx, no tolerance) | failed as expected (51/62 mutants) | **KEEP (nit)** |
| 165 | `test_adaptive_width_raises_the_corner_speed_ceiling_on_a_wide_bend` | gap_follow/test_gap_follow_node.py:625 | Corner_speed itself is the floor -- today's tuned narrow-corner | A5 (bare approx, no tolerance) | failed as expected (13/13 mutants) | **KEEP (nit)** |
| 166 | `test_adaptive_width_ignores_an_open_side_and_holds_the_static_defaults` | gap_follow/test_gap_follow_node.py:636 | One side unbounded is not a corridor to measure -- treated as "wide" | A5 (bare approx, no tolerance) | failed as expected (12/12 mutants) | **KEEP (nit)** |
| 167 | `test_smaller_effective_margin_recovers_a_gap_a_static_one_would_miss` | gap_follow/test_gap_follow_node.py:649 | The mechanism the ASB 10000-level course exercised (see | — | failed as expected (17/17 mutants) | **KEEP** |
| 168 | `test_adaptive_width_refuses_a_floor_above_the_static_margin` | gap_follow/test_gap_follow_node.py:681 | Min_safety_margin is a floor the margin may shrink to, not a value | — | failed as expected (4/4 mutants) | **KEEP** |
| 169 | `test_adaptive_width_refuses_a_narrow_reference_that_does_not_fit` | gap_follow/test_gap_follow_node.py:695 | Adaptive width refuses a narrow reference that does not fit | — | failed as expected (4/4 mutants) | **KEEP** |
| 170 | `test_packaged_config_plus_mapping_overrides_actually_starts` | gap_follow/test_gap_follow_node.py:706 | The real regression guard for 2026-08-19 | — | failed as expected (12/12 mutants) | **KEEP** |
| 171 | `test_lowering_only_max_speed_is_still_rejected` | gap_follow/test_gap_follow_node.py:734 | The launch-side fix must not have quietly relaxed the check itself: | — | failed as expected (4/4 mutants) | **KEEP** |
| 172 | `test_adaptive_width_refuses_a_corner_speed_ceiling_below_its_floor` | gap_follow/test_gap_follow_node.py:747 | Adaptive width refuses a corner speed ceiling below its floor | — | failed as expected (4/4 mutants) | **KEEP** |
| 173 | `test_select_gap_hysteresis_keeps_the_previous_gap_through_a_small_edge` | gap_follow/test_gap_follow_node.py:765 | Same shape as gap_logic's own hysteresis tests, run through the real | — | failed as expected (17/17 mutants) | **KEEP** |
| 174 | `test_select_gap_hysteresis_is_a_no_op_with_no_previous_target` | gap_follow/test_gap_follow_node.py:788 | Select gap hysteresis is a no op with no previous target | — | failed as expected (17/17 mutants) | **KEEP** |
| 175 | `test_cornering_anticipation_caps_speed_beyond_ordinary_curvature` | gap_follow/test_gap_follow_node.py:805 | A gap whose near portion (<=anticipation_near_depth) points one way | — | failed as expected (18/18 mutants) | **KEEP** |
| 176 | `test_cornering_anticipation_near_depth_must_exceed_zero` | gap_follow/test_gap_follow_node.py:852 | Cornering anticipation near depth must exceed zero | — | failed as expected (2/2 mutants) | **KEEP** |
| 177 | `test_sanitize_keeps_normal_readings_valid` | gap_follow/test_gap_logic.py:23 | Sanitize keeps normal readings valid | — | failed as expected (1/1 mutants) | **KEEP** |
| 178 | `test_sanitize_nan_is_invalid_and_non_free` | gap_follow/test_gap_logic.py:29 | Sanitize nan is invalid and non free | — | failed as expected (1/1 mutants) | **KEEP** |
| 179 | `test_sanitize_sub_range_min_is_invalid` | gap_follow/test_gap_logic.py:39 | Sanitize sub range min is invalid | — | failed as expected (1/1 mutants) | **KEEP** |
| 180 | `test_sanitize_posinf_is_free_space_not_invalid` | gap_follow/test_gap_logic.py:48 | Sanitize posinf is free space not invalid | — | failed as expected (1/1 mutants) | **KEEP** |
| 181 | `test_sanitize_clips_to_max_range` | gap_follow/test_gap_logic.py:56 | Sanitize clips to max range | — | failed as expected (1/1 mutants) | **KEEP** |
| 182 | `test_closest_valid_ignores_invalid_beams` | gap_follow/test_gap_logic.py:65 | Closest valid ignores invalid beams | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 183 | `test_closest_valid_with_no_valid_beams_reports_nothing` | gap_follow/test_gap_logic.py:73 | Closest valid with no valid beams reports nothing | — | failed as expected (2/2 mutants) | **KEEP** |
| 184 | `test_noisy_scan_does_not_emergency_stop` | gap_follow/test_gap_logic.py:81 | Noisy scan does not emergency stop | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 185 | `test_disparity_extend_widens_the_far_side_of_an_edge` | gap_follow/test_gap_logic.py:103 | Disparity extend widens the far side of an edge | — | failed as expected (1/1 mutants) | **KEEP** |
| 186 | `test_disparity_extend_never_raises_a_range` | gap_follow/test_gap_logic.py:116 | Disparity extend never raises a range | — | failed as expected (1/1 mutants) | **KEEP** |
| 187 | `test_disparity_extend_reaches_further_for_closer_edges` | gap_follow/test_gap_logic.py:123 | Disparity extend reaches further for closer edges | — | failed as expected (1/1 mutants) | **KEEP** |
| 188 | `test_disparity_extend_handles_an_edge_in_the_other_direction` | gap_follow/test_gap_logic.py:134 | Disparity extend handles an edge in the other direction | — | failed as expected (1/1 mutants) | **KEEP** |
| 189 | `test_curvature_speed_limit_is_fast_straight_and_slow_in_turn` | gap_follow/test_gap_logic.py:143 | Curvature speed limit is fast straight and slow in turn | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 190 | `test_braking_speed_limit_reserves_stopping_clearance` | gap_follow/test_gap_logic.py:149 | Braking speed limit reserves stopping clearance | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 191 | `test_slew_rate_limit_has_asymmetric_rise_and_fall` | gap_follow/test_gap_logic.py:158 | Slew rate limit has asymmetric rise and fall | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 192 | `test_disparity_extend_ignores_smooth_walls` | gap_follow/test_gap_logic.py:164 | Disparity extend ignores smooth walls | — | failed as expected (1/1 mutants) | **KEEP** |
| 193 | `test_disparity_extend_skips_invalid_zero_edges` | gap_follow/test_gap_logic.py:172 | Disparity extend skips invalid zero edges | — | failed as expected (1/1 mutants) | **KEEP** |
| 194 | `test_safety_bubble_zeroes_around_the_closest_point` | gap_follow/test_gap_logic.py:185 | Safety bubble zeroes around the closest point | — | failed as expected (1/1 mutants) | **KEEP** |
| 195 | `test_safety_bubble_is_wider_for_closer_obstacles` | gap_follow/test_gap_logic.py:196 | Safety bubble is wider for closer obstacles | — | failed as expected (1/1 mutants) | **KEEP** |
| 196 | `test_best_gap_prefers_deep_corridor_over_shallow_alcove` | gap_follow/test_gap_logic.py:207 | Best gap prefers deep corridor over shallow alcove | — | failed as expected (1/1 mutants) | **KEEP** |
| 197 | `test_best_gap_rejects_gaps_narrower_than_the_car` | gap_follow/test_gap_logic.py:215 | Best gap rejects gaps narrower than the car | A2 (shape/None only) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 198 | `test_best_gap_accepts_gaps_wider_than_the_car` | gap_follow/test_gap_logic.py:227 | Best gap accepts gaps wider than the car | — | failed as expected (1/1 mutants) | **KEEP** |
| 199 | `test_best_gap_returns_none_when_boxed_in` | gap_follow/test_gap_logic.py:236 | Best gap returns none when boxed in | A2 (shape/None only) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 200 | `test_vehicle_boundary_matches_padded_traxxas_rectangle` | gap_follow/test_gap_logic.py:256 | Vehicle boundary matches padded traxxas rectangle | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 201 | `test_vehicle_boundary_rejects_lidar_outside_footprint` | gap_follow/test_gap_logic.py:265 | Vehicle boundary rejects lidar outside footprint | — | failed as expected (1/1 mutants) | **KEEP** |
| 202 | `test_minimum_clearance_is_measured_from_body_not_lidar` | gap_follow/test_gap_logic.py:271 | Minimum clearance is measured from body not lidar | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 203 | `test_ttc_projects_odometry_speed_and_subtracts_footprint` | gap_follow/test_gap_logic.py:279 | Ttc projects odometry speed and subtracts footprint | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 204 | `test_ttc_ignores_invalid_beams_and_stationary_vehicle` | gap_follow/test_gap_logic.py:295 | Ttc ignores invalid beams and stationary vehicle | — | failed as expected (3/3 mutants) | **KEEP** |
| 205 | `test_forward_clearance_cone_ignores_close_side_obstacles` | gap_follow/test_gap_logic.py:306 | Forward clearance cone ignores close side obstacles | A5 (bare approx, no tolerance) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 206 | `test_forward_clearance_cone_reports_frontal_obstacle` | gap_follow/test_gap_logic.py:320 | Forward clearance cone reports frontal obstacle | A5 (bare approx, no tolerance) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 207 | `test_conservative_ttc_speed_uses_recent_positive_command` | gap_follow/test_gap_logic.py:334 | Conservative ttc speed uses recent positive command | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 208 | `test_conservative_ttc_speed_keeps_higher_measured_speed` | gap_follow/test_gap_logic.py:343 | Conservative ttc speed keeps higher measured speed | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 209 | `test_conservative_ttc_speed_trusts_meaningful_fresh_odom` | gap_follow/test_gap_logic.py:352 | Conservative ttc speed trusts meaningful fresh odom | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 210 | `test_conservative_ttc_speed_ignores_stale_command` | gap_follow/test_gap_logic.py:362 | Conservative ttc speed ignores stale command | — | failed as expected (1/1 mutants) | **KEEP** |
| 211 | `test_conservative_ttc_speed_uses_reverse_motion_magnitude` | gap_follow/test_gap_logic.py:371 | A stale command must not hide real motion just because it reads | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 212 | `test_conservative_ttc_speed_survives_inverted_odometry_sign` | gap_follow/test_gap_logic.py:382 | Regression: the 2026-07-27 collision | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 213 | `test_conservative_ttc_speed_rejects_invalid_age` | gap_follow/test_gap_logic.py:401 | Conservative ttc speed rejects invalid age | — | failed as expected (1/1 mutants) | **KEEP** |
| 214 | `test_gap_fallback_accepts_passable_corner_hidden_inside_two_metres` | gap_follow/test_gap_logic.py:411 | Gap fallback accepts passable corner hidden inside two metres | — | failed as expected (2/2 mutants) | **KEEP** |
| 215 | `test_gap_fallback_keeps_preferred_deep_gap_when_available` | gap_follow/test_gap_logic.py:425 | Gap fallback keeps preferred deep gap when available | — | failed as expected (2/2 mutants) | **KEEP** |
| 216 | `test_gap_fallback_still_rejects_boxed_in_scene` | gap_follow/test_gap_logic.py:434 | Gap fallback still rejects boxed in scene | — | failed as expected (2/2 mutants) | **KEEP** |
| 217 | `test_hysteresis_is_a_no_op_with_no_previous_target` | gap_follow/test_gap_logic.py:456 | Hysteresis is a no op with no previous target | — | failed as expected (1/1 mutants) | **KEEP** |
| 218 | `test_hysteresis_is_a_no_op_at_the_default_switch_margin` | gap_follow/test_gap_logic.py:461 | Hysteresis is a no op at the default switch margin | — | failed as expected (1/1 mutants) | **KEEP** |
| 219 | `test_hysteresis_keeps_the_previous_gap_through_a_small_score_edge` | gap_follow/test_gap_logic.py:468 | B (score 42) beats A (score 40) by 5% -- under the 20% margin, so | — | failed as expected (1/1 mutants) | **KEEP** |
| 220 | `test_hysteresis_still_switches_for_a_real_improvement` | gap_follow/test_gap_logic.py:478 | B now scores 50% better than A -- past the 20% margin, so the car | — | failed as expected (1/1 mutants) | **KEEP** |
| 221 | `test_hysteresis_falls_through_when_the_previous_target_has_no_gap_left` | gap_follow/test_gap_logic.py:488 | Hysteresis falls through when the previous target has no gap left | — | failed as expected (1/1 mutants) | **KEEP** |
| 222 | `test_hysteresis_never_picks_a_candidate_that_failed_the_width_filter` | gap_follow/test_gap_logic.py:496 | The sticky candidate still has to have passed min_gap_width_m -- | — | failed as expected (1/1 mutants) | **KEEP** |
| 223 | `test_find_gap_with_fallback_applies_hysteresis_within_the_preferred_tier` | gap_follow/test_gap_logic.py:509 | Find gap with fallback applies hysteresis within the preferred tier | — | failed as expected (2/2 mutants) | **KEEP** |
| 224 | `test_near_gap_bearing_uses_only_the_close_beams` | gap_follow/test_gap_logic.py:523 | 10 near beams (1.0m) around bearing ~0.045rad, 10 far beams (5.0m) | — | failed as expected (1/1 mutants) | **KEEP** |
| 225 | `test_near_gap_bearing_is_none_with_no_near_beams` | gap_follow/test_gap_logic.py:535 | Nothing to compare against -- there is no early look here at all, | A2 (shape/None only) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 226 | `test_near_gap_bearing_is_none_when_the_whole_gap_is_near` | gap_follow/test_gap_logic.py:543 | A small room, not a corridor with something beyond the near view | A2 (shape/None only) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 227 | `test_near_gap_bearing_is_none_without_a_gap` | gap_follow/test_gap_logic.py:550 | Near gap bearing is none without a gap | A2 (shape/None only) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 228 | `test_near_gap_bearing_rejects_a_non_positive_near_depth` | gap_follow/test_gap_logic.py:554 | Near gap bearing rejects a non positive near depth | — | failed as expected (1/1 mutants) | **KEEP** |
| 229 | `test_post_inflation_gap_does_not_require_second_full_car_width` | gap_follow/test_gap_logic.py:562 | Post inflation gap does not require second full car width | — | failed as expected (1/1 mutants) | **KEEP** |
| 230 | `test_ttc_ignores_a_wall_the_car_drives_past` | gap_follow/test_gap_logic.py:583 | Regression: the 1m-course crawl | — | failed as expected (2/2 mutants) | **KEEP** |
| 231 | `test_ttc_still_brakes_for_an_obstacle_dead_ahead` | gap_follow/test_gap_logic.py:600 | Ttc still brakes for an obstacle dead ahead | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 232 | `test_ttc_still_brakes_for_an_obstacle_inside_the_swept_width` | gap_follow/test_gap_logic.py:606 | Ttc still brakes for an obstacle inside the swept width | — | failed as expected (2/2 mutants) | **KEEP** |
| 233 | `test_ttc_swept_corridor_follows_the_turn` | gap_follow/test_gap_logic.py:613 | A car at full lock curves around the outside of its own corner. Judging | — | failed as expected (2/2 mutants) | **KEEP** |
| 234 | `test_side_wall_distance_recovers_the_perpendicular_distance` | gap_follow/test_gap_logic.py:656 | Side wall distance recovers the perpendicular distance | — | failed as expected (1/1 mutants) | **KEEP** |
| 235 | `test_side_wall_distance_is_yaw_tolerant_inside_the_window` | gap_follow/test_gap_logic.py:664 | A beam hits a straight wall at d/cos(theta), minimised at the | — | failed as expected (1/1 mutants) | **KEEP** |
| 236 | `test_side_wall_distance_survives_a_doorway_mid_window` | gap_follow/test_gap_logic.py:675 | A hole in the wall must not read as 'acres of room on this side' -- | — | failed as expected (1/1 mutants) | **KEEP** |
| 237 | `test_side_wall_distance_is_infinite_with_no_valid_beam` | gap_follow/test_gap_logic.py:686 | Side wall distance is infinite with no valid beam | — | failed as expected (1/1 mutants) | **KEEP** |
| 238 | `test_side_wall_distance_ignores_invalid_beams` | gap_follow/test_gap_logic.py:692 | Side wall distance ignores invalid beams | — | failed as expected (1/1 mutants) | **KEEP** |
| 239 | `test_centering_is_silent_in_the_middle_of_the_corridor` | gap_follow/test_gap_logic.py:719 | Centering is silent in the middle of the corridor | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 240 | `test_centering_steers_right_when_hugging_the_left_wall` | gap_follow/test_gap_logic.py:725 | Centering steers right when hugging the left wall | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 241 | `test_centering_steers_left_when_hugging_the_right_wall` | gap_follow/test_gap_logic.py:731 | Centering steers left when hugging the right wall | — | failed as expected (2/2 mutants) | **KEEP** |
| 242 | `test_centering_is_proportional_to_the_offset` | gap_follow/test_gap_logic.py:736 | Centering is proportional to the offset | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 243 | `test_centering_is_clamped_to_its_authority_bound` | gap_follow/test_gap_logic.py:743 | The clamp is the safety property: this term can never outvote the gap | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 244 | `test_centering_fades_out_while_turning` | gap_follow/test_gap_logic.py:752 | Centering fades out while turning | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 245 | `test_centering_fade_is_a_ramp_not_a_switch` | gap_follow/test_gap_logic.py:758 | A hard on/off on a steering term is what produced the scan-rate | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 246 | `test_centering_ignores_a_side_that_is_not_a_wall` | gap_follow/test_gap_logic.py:769 | An opening on one side is not something to centre against -- otherwise | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 247 | `test_centering_stops_before_a_corner` | gap_follow/test_gap_logic.py:777 | Centering stops before a corner | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 248 | `test_centering_can_be_disabled_by_zero_gain` | gap_follow/test_gap_logic.py:782 | Centering can be disabled by zero gain | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 249 | `test_centering_rejects_a_negative_authority_bound` | gap_follow/test_gap_logic.py:786 | Centering rejects a negative authority bound | — | failed as expected (1/1 mutants) | **KEEP** |
| 250 | `test_centering_converges_to_the_middle_of_a_straight` | gap_follow/test_gap_logic.py:822 | Centering converges to the middle of a straight | — | failed as expected (2/2 mutants) | **KEEP** |
| 251 | `test_centering_converges_from_either_side` | gap_follow/test_gap_logic.py:828 | Centering converges from either side | — | failed as expected (2/2 mutants) | **KEEP** |
| 252 | `test_centering_does_not_weave` | gap_follow/test_gap_logic.py:834 | The bearing term is the damping: as the car turns toward the middle its | — | failed as expected (2/2 mutants) | **KEEP** |
| 253 | `test_without_centering_the_car_holds_its_offset_forever` | gap_follow/test_gap_logic.py:846 | The behaviour being fixed: bearing steering alone is parallel to the | — | no mutant reached it | **KEEP (unverified)** |
| 254 | `test_centering_never_leaves_the_corridor` | gap_follow/test_gap_logic.py:853 | Centering never leaves the corridor | — | failed as expected (2/2 mutants) | **KEEP** |
| 255 | `test_width_factor_is_one_at_or_above_the_reference_width` | gap_follow/test_gap_logic.py:866 | Width factor is one at or above the reference width | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 256 | `test_width_factor_is_zero_at_or_below_the_narrow_width` | gap_follow/test_gap_logic.py:871 | Width factor is zero at or below the narrow width | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 257 | `test_width_factor_is_linear_between_them` | gap_follow/test_gap_logic.py:876 | Width factor is linear between them | A3 (unsourced number); A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 258 | `test_width_factor_ignores_a_side_that_is_not_a_wall` | gap_follow/test_gap_logic.py:881 | The safe default when a side is unbounded: assume the wide, | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 259 | `test_width_factor_rejects_a_narrow_width_that_does_not_fit` | gap_follow/test_gap_logic.py:889 | Width factor rejects a narrow width that does not fit | — | failed as expected (1/1 mutants) | **KEEP** |
| 260 | `test_scale_between_endpoints` | gap_follow/test_gap_logic.py:896 | Scale between endpoints | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 261 | `test_scale_between_works_growing_or_shrinking` | gap_follow/test_gap_logic.py:902 | Scale between works growing or shrinking | A3 (unsourced number); A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 262 | `test_scale_between_rejects_a_width_factor_outside_zero_one` | gap_follow/test_gap_logic.py:908 | Scale between rejects a width factor outside zero one | — | failed as expected (1/1 mutants) | **KEEP** |
| 263 | `test_packaged_config_alone_is_self_consistent` | gap_follow/test_speed_overrides.py:41 | Packaged config alone is self consistent | A5 (no assertion); cleared: asserts through a helper | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 264 | `test_every_mapping_speed_yields_a_startable_set` | gap_follow/test_speed_overrides.py:48 | Every mapping speed yields a startable set | A5 (no assertion); cleared: asserts through a helper | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 265 | `test_the_exact_arguments_that_killed_the_node` | gap_follow/test_speed_overrides.py:53 | The exact arguments that killed the node | — | failed as expected (3/3 mutants) | **KEEP** |
| 266 | `test_no_cap_is_the_default_and_overrides_nothing` | gap_follow/test_speed_overrides.py:63 | The launch files now pass empty strings unless a cap is asked for | — | failed as expected (2/2 mutants) | **KEEP** |
| 267 | `test_a_floor_without_a_cap_passes_through_alone` | gap_follow/test_speed_overrides.py:76 | A floor without a cap passes through alone | — | failed as expected (2/2 mutants) | **KEEP** |
| 268 | `test_a_cap_without_a_floor_uses_the_configured_floor` | gap_follow/test_speed_overrides.py:82 | A cap without a floor uses the configured floor | — | failed as expected (3/3 mutants) | **KEEP** |
| 269 | `test_a_cap_below_the_configured_floor_still_starts` | gap_follow/test_speed_overrides.py:92 | A cap below the configured floor still starts | — | failed as expected (3/3 mutants) | **KEEP** |
| 270 | `test_coupled_caps_scale_with_max_speed_not_just_clamp` | gap_follow/test_speed_overrides.py:104 | Coupled caps scale with max speed not just clamp | A5 (bare approx, no tolerance) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 271 | `test_raising_max_speed_raises_the_caps_with_it` | gap_follow/test_speed_overrides.py:116 | Raising max speed raises the caps with it | A5 (bare approx, no tolerance) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 272 | `test_caps_never_scale_past_the_new_max_speed` | gap_follow/test_speed_overrides.py:126 | Caps never scale past the new max speed | — | failed as expected (3/3 mutants) | **KEEP** |
| 273 | `test_min_speed_is_clamped_rather_than_left_above_max` | gap_follow/test_speed_overrides.py:139 | Min speed is clamped rather than left above max | — | failed as expected (3/3 mutants) | **KEEP** |
| 274 | `test_string_arguments_from_launch_configurations_are_accepted` | gap_follow/test_speed_overrides.py:151 | String arguments from launch configurations are accepted | — | failed as expected (3/3 mutants) | **KEEP** |
| 275 | `test_absent_coupled_parameter_is_left_to_the_node_default` | gap_follow/test_speed_overrides.py:158 | Absent coupled parameter is left to the node default | — | failed as expected (3/3 mutants) | **KEEP** |
| 276 | `test_config_without_a_max_speed_still_overrides_the_asked_for_speeds` | gap_follow/test_speed_overrides.py:164 | Config without a max speed still overrides the asked for speeds | — | failed as expected (3/3 mutants) | **KEEP** |
| 277 | `test_every_coupled_name_exists_in_the_packaged_config` | gap_follow/test_speed_overrides.py:170 | Every coupled name exists in the packaged config | — | failed as expected (1/1 mutants) | **KEEP** |
| 278 | `test_scaled_caps_are_floored_at_min_speed` | gap_follow/test_speed_overrides.py:178 | Scaled caps are floored at min speed | — | failed as expected (3/3 mutants) | **KEEP** |
| 279 | `test_ordering_survives_both_caps_landing_on_the_floor` | gap_follow/test_speed_overrides.py:191 | Ordering survives both caps landing on the floor | A5 (bare approx, no tolerance) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 280 | `test_integrator_skips_large_telemetry_gap` | odom_calibration/test_calibration_math.py:52 | Integrator skips large telemetry gap | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 281 | `test_capture_summary_preserves_reverse_signs` | odom_calibration/test_calibration_math.py:66 | Capture summary preserves reverse signs | A5 (bare approx, no tolerance) | failed as expected (7/7 mutants) | **KEEP (nit)** |
| 282 | `test_forward_and_reverse_trials_produce_positive_gain` | odom_calibration/test_calibration_math.py:92 | Forward and reverse trials produce positive gain | A5 (bare approx, no tolerance) | failed as expected (5/5 mutants) | **KEEP (nit)** |
| 283 | `test_negative_gain_is_reported_not_absolute_valued` | odom_calibration/test_calibration_math.py:118 | Negative gain is reported not absolute valued | — | failed as expected (4/4 mutants) | **KEEP** |
| 284 | `test_movement_falls_back_to_current_odom_scale_without_raw_erpm` | odom_calibration/test_calibration_math.py:137 | Movement falls back to current odom scale without raw erpm | A5 (bare approx, no tolerance) | failed as expected (5/5 mutants) | **KEEP (nit)** |
| 285 | `test_robust_gain_rejects_outlier_even_when_good_values_match_exactly` | odom_calibration/test_calibration_math.py:156 | Robust gain rejects outlier even when good values match exactly | A5 (bare approx, no tolerance) | failed as expected (5/5 mutants) | **KEEP (nit)** |
| 286 | `test_left_right_and_center_fit_steering_parameters` | odom_calibration/test_calibration_math.py:187 | Left right and center fit steering parameters | A5 (bare approx, no tolerance) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 287 | `test_report_omits_steering_for_movement_only_mode` | odom_calibration/test_calibration_math.py:216 | Report omits steering for movement only mode | — | failed as expected (3/3 mutants) | **KEEP** |
| 288 | `test_markdown_never_writes_none_as_parameter_value` | odom_calibration/test_calibration_math.py:230 | Markdown never writes none as parameter value | — | failed as expected (6/6 mutants) | **KEEP** |
| 289 | `test_session_round_trip` | odom_calibration/test_session_store.py:21 | Session round trip | — | failed as expected (6/6 mutants) | **KEEP** |
| 290 | `test_restart_marks_active_capture_interrupted` | odom_calibration/test_session_store.py:31 | Restart marks active capture interrupted | — | failed as expected (6/6 mutants) | **KEEP** |
| 291 | `test_deliberate_replacement_archive_is_valid_json` | odom_calibration/test_session_store.py:41 | Deliberate replacement archive is valid json | — | failed as expected (4/4 mutants) | **KEEP** |
| 292 | `test_report_archive_writes_json_and_markdown` | odom_calibration/test_session_store.py:51 | Report archive writes json and markdown | — | failed as expected (4/4 mutants) | **KEEP** |
| 293 | `test_angle_difference_wraps_at_pi` | pure_pursuit/test_auto_map_race.py:16 | Angle difference wraps at pi | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 294 | `test_lap_recorder_requires_departure_distance_and_heading` | pure_pursuit/test_auto_map_race.py:20 | Lap recorder requires departure distance and heading | — | failed as expected (5/5 mutants) | **KEEP** |
| 295 | `test_lap_recorder_does_not_close_before_departing` | pure_pursuit/test_auto_map_race.py:49 | Lap recorder does not close before departing | — | failed as expected (4/4 mutants) | **KEEP** |
| 296 | `test_profile_parameter_response_enables_racing_transition` | pure_pursuit/test_auto_map_race.py:75 | Profile parameter response enables racing transition | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 297 | `test_supervisor_reports_missing_then_forwards_fresh_mapping_command` | pure_pursuit/test_auto_map_race.py:96 | Supervisor reports missing then forwards fresh mapping command | A5 (bare approx, no tolerance) | failed as expected (13/13 mutants) | **KEEP (nit)** |
| 298 | `test_profile_handover_waits_for_the_blocking_slam_save` | pure_pursuit/test_auto_map_race.py:150 | Regression test for the 2026-07-27 collision | — | failed as expected (14/14 mutants) | **KEEP** |
| 299 | `test_handover_proceeds_after_the_save_times_out` | pure_pursuit/test_auto_map_race.py:190 | A wedged save must not strand the car forever: the racing line is | — | failed as expected (14/14 mutants) | **KEEP** |
| 300 | `test_a_failed_save_is_retried_before_the_gate_moves_on` | pure_pursuit/test_auto_map_race.py:219 | Slam_toolbox's SaveMap runs nav2's map_saver inline, and map_saver | — | failed as expected (5/5 mutants) | **KEEP** |
| 301 | `test_a_failed_save_settles_once_the_retries_are_used_up` | pure_pursuit/test_auto_map_race.py:242 | The gate cares whether slam_toolbox is still blocked, not whether | — | failed as expected (4/4 mutants) | **KEEP** |
| 302 | `test_a_successful_save_is_not_retried` | pure_pursuit/test_auto_map_race.py:258 | A successful save is not retried | — | failed as expected (5/5 mutants) | **KEEP** |
| 303 | `test_a_failed_pose_graph_save_is_never_retried` | pure_pursuit/test_auto_map_race.py:276 | Only the occupancy map has the map_saver race. Retrying the pose | — | failed as expected (4/4 mutants) | **KEEP** |
| 304 | `test_recorder_absorbs_a_slam_correction_instead_of_recording_it` | pure_pursuit/test_auto_map_race.py:315 | A pose that teleports is slam_toolbox re-optimising, not the car | — | failed as expected (5/5 mutants) | **KEEP** |
| 305 | `test_recorder_reanchors_rotation_as_well_as_translation` | pure_pursuit/test_auto_map_race.py:345 | A correction can rotate the map, not just slide it | — | failed as expected (5/5 mutants) | **KEEP** |
| 306 | `test_recorder_without_jump_detection_records_the_correction` | pure_pursuit/test_auto_map_race.py:363 | Max_pose_jump_m: 0.0 keeps the old behaviour, for comparison runs | — | failed as expected (5/5 mutants) | **KEEP** |
| 307 | `test_a_lap_shorter_than_minimum_lap_distance_never_closes` | pure_pursuit/test_auto_map_race.py:377 | The defect behind every two-revolution recording this car made | — | failed as expected (4/4 mutants) | **KEEP** |
| 308 | `test_turn_gate_closes_a_lap_the_distance_gate_would_miss` | pure_pursuit/test_auto_map_race.py:394 | Accumulated yaw does not need to be told how big the course is: one | — | failed as expected (5/5 mutants) | **KEEP** |
| 309 | `test_turn_gate_rejects_a_there_and_back_again_run` | pure_pursuit/test_auto_map_race.py:409 | Driving out and reversing back to the start passes every distance | — | failed as expected (4/4 mutants) | **KEEP** |
| 310 | `test_odom_turn_survives_corrections_that_map_turn_loses` | pure_pursuit/test_auto_map_race.py:430 | The turn gate must not depend on how busy the pose graph was | — | failed as expected (6/6 mutants) | **KEEP** |
| 311 | `test_odom_turn_ignores_a_correction_that_only_rotates_the_map` | pure_pursuit/test_auto_map_race.py:468 | A pure map rotation is not the car turning, whatever the map says | — | failed as expected (5/5 mutants) | **KEEP** |
| 312 | `test_missing_odom_falls_back_to_the_previous_behaviour` | pure_pursuit/test_auto_map_race.py:483 | Odometry not up yet is not a reason to stop counting turns | — | failed as expected (5/5 mutants) | **KEEP** |
| 313 | `test_a_lap_that_misses_a_fixed_gate_still_closes_once_widened` | pure_pursuit/test_auto_map_race.py:495 | The failure mode that costs a whole revolution per miss | — | failed as expected (6/6 mutants) | **KEEP** |
| 314 | `test_widening_is_off_when_either_parameter_is_zero` | pure_pursuit/test_auto_map_race.py:528 | Widening is off when either parameter is zero | — | failed as expected (5/5 mutants) | **KEEP** |
| 315 | `test_lap_points_trims_a_multi_revolution_recording_to_one_lap` | pure_pursuit/test_auto_map_race.py:541 | Two overlapping laps are not a racing line | — | failed as expected (7/7 mutants) | **KEEP** |
| 316 | `test_lap_points_returns_a_single_revolution_untouched` | pure_pursuit/test_auto_map_race.py:567 | The normal case must not be trimmed at all | — | failed as expected (7/7 mutants) | **KEEP** |
| 317 | `test_lap_points_trims_a_clockwise_recording_too` | pure_pursuit/test_auto_map_race.py:580 | Turning is signed, and half the courses go the other way | — | failed as expected (7/7 mutants) | **KEEP** |
| 318 | `test_closest_approach_only_counts_after_departing` | pure_pursuit/test_auto_map_race.py:597 | Sitting at the start is not a near miss | — | failed as expected (5/5 mutants) | **KEEP** |
| 319 | `test_optimizer_straightens_the_line_and_keeps_it_off_the_walls` | pure_pursuit/test_auto_map_race.py:654 | Optimizer straightens the line and keeps it off the walls | — | failed as expected (38/38 mutants) | **KEEP** |
| 320 | `test_optimizer_is_skipped_when_turned_off_or_mapless` | pure_pursuit/test_auto_map_race.py:683 | Optimizer is skipped when turned off or mapless | A2 (shape/None only) | failed as expected (7/7 mutants) | **KEEP (nit)** |
| 321 | `test_optimized_line_too_close_to_a_wall_is_refused` | pure_pursuit/test_auto_map_race.py:699 | Failing the clearance check must fall back, not race the line | A2 (shape/None only) | failed as expected (12/36 mutants) | **KEEP (nit)** |
| 322 | `test_particle_filter_is_only_trusted_after_it_settles` | pure_pursuit/test_auto_map_race.py:745 | Particle filter is only trusted after it settles | — | failed as expected (8/8 mutants) | **KEEP** |
| 323 | `test_a_silent_particle_filter_falls_back_to_slam_for_good` | pure_pursuit/test_auto_map_race.py:766 | A silent particle filter falls back to slam for good | — | failed as expected (8/8 mutants) | **KEEP** |
| 324 | `test_slow_particle_filter_is_given_up_on_at_the_timeout` | pure_pursuit/test_auto_map_race.py:792 | Slow particle filter is given up on at the timeout | — | failed as expected (7/7 mutants) | **KEEP** |
| 325 | `test_the_published_pose_follows_whichever_source_is_trusted` | pure_pursuit/test_auto_map_race.py:807 | Pure_pursuit reads one topic throughout; only the source changes | A5 (bare approx, no tolerance) | failed as expected (7/7 mutants) | **KEEP (nit)** |
| 326 | `test_no_saved_map_means_no_handover_attempt` | pure_pursuit/test_auto_map_race.py:838 | No saved map means no handover attempt | — | failed as expected (5/5 mutants) | **KEEP** |
| 327 | `test_a_slower_optimized_line_is_refused` | pure_pursuit/test_auto_map_race.py:850 | The justification for optimizing is lap time, so lap time decides | A2 (shape/None only) | failed as expected (10/34 mutants) | **KEEP (nit)** |
| 328 | `test_over_steering_is_tolerated_only_when_no_worse_than_the_recording` | pure_pursuit/test_auto_map_race.py:876 | `prepare` accepts a recording that exceeds the rack and warns about | — | failed as expected (38/38 mutants) | **KEEP** |
| 329 | `test_frozen_pose_stops_the_car_although_messages_keep_arriving` | pure_pursuit/test_localization_watchdogs.py:116 | The 2026-07-27 signature: pose republished punctually but never | NP (time.sleep sequencing) | failed as expected (59/72 mutants) | **KEEP (nit)** |
| 330 | `test_frozen_watchdog_recovers_when_localization_resumes` | pure_pursuit/test_localization_watchdogs.py:148 | A stall must stop the car, not latch it off forever -- once the | NP (time.sleep sequencing) | failed as expected (55/68 mutants) | **KEEP (nit)** |
| 331 | `test_a_parked_car_does_not_trip_the_frozen_pose_watchdog` | pure_pursuit/test_localization_watchdogs.py:175 | The pose is equally motionless when the car is simply stopped -- | — | failed as expected (53/66 mutants) | **KEEP** |
| 332 | `test_a_moving_car_with_tracking_localization_is_left_alone` | pure_pursuit/test_localization_watchdogs.py:189 | The watchdog must not fire while localization is doing its job | — | failed as expected (53/66 mutants) | **KEEP** |
| 333 | `test_stale_header_stamp_is_caught_even_when_the_message_just_arrived` | pure_pursuit/test_localization_watchdogs.py:203 | Second, independent defence: the message is brand new, but the pose | — | failed as expected (29/43 mutants) | **KEEP** |
| 334 | `test_unstamped_poses_still_drive` | pure_pursuit/test_localization_watchdogs.py:217 | Publishers that leave header.stamp at zero (the default-constructed | — | failed as expected (51/64 mutants) | **KEEP** |
| 335 | `test_rear_beams_hitting_the_cars_own_chassis_do_not_pin_it` | pure_pursuit/test_localization_watchdogs.py:242 | Regression test for the 2026-07-27 standstill | — | failed as expected (52/65 mutants) | **KEEP** |
| 336 | `test_a_wall_on_the_flank_is_still_caught_inside_the_window` | pure_pursuit/test_localization_watchdogs.py:266 | The narrowed window must not cost the lateral coverage that is the | — | failed as expected (46/58 mutants) | **KEEP** |
| 337 | `test_a_single_stray_cell_in_clear_space_is_removed` | pure_pursuit/test_map_despeckle.py:39 | A single stray cell in clear space is removed | — | failed as expected (1/1 mutants) | **KEEP** |
| 338 | `test_the_walls_are_never_touched` | pure_pursuit/test_map_despeckle.py:47 | The walls are never touched | — | failed as expected (1/1 mutants) | **KEEP** |
| 339 | `test_a_blob_that_casts_a_shadow_is_kept` | pure_pursuit/test_map_despeckle.py:59 | The safety case: a real object occludes, so unknown cells sit behind | — | failed as expected (1/1 mutants) | **KEEP** |
| 340 | `test_the_halo_is_exactly_one_cell_wide_by_default` | pure_pursuit/test_map_despeckle.py:71 | A blob touching unobserved space is kept; one clear cell of margin | — | failed as expected (1/1 mutants) | **KEEP** |
| 341 | `test_a_blob_larger_than_the_threshold_is_kept` | pure_pursuit/test_map_despeckle.py:88 | A blob larger than the threshold is kept | — | failed as expected (1/1 mutants) | **KEEP** |
| 342 | `test_the_threshold_is_inclusive_and_counts_diagonals` | pure_pursuit/test_map_despeckle.py:96 | The threshold is inclusive and counts diagonals | — | failed as expected (1/1 mutants) | **KEEP** |
| 343 | `test_max_cells_zero_disables_the_filter_entirely` | pure_pursuit/test_map_despeckle.py:105 | Max cells zero disables the filter entirely | — | failed as expected (1/1 mutants) | **KEEP** |
| 344 | `test_the_input_grid_is_never_modified` | pure_pursuit/test_map_despeckle.py:113 | The input grid is never modified | — | failed as expected (1/1 mutants) | **KEEP** |
| 345 | `test_only_occupied_cells_ever_change_and_only_to_free` | pure_pursuit/test_map_despeckle.py:121 | Only occupied cells ever change and only to free | — | failed as expected (1/1 mutants) | **KEEP** |
| 346 | `test_unknown_cells_are_left_alone` | pure_pursuit/test_map_despeckle.py:131 | Unknown cells are left alone | — | failed as expected (1/1 mutants) | **KEEP** |
| 347 | `test_a_clean_map_is_returned_unchanged` | pure_pursuit/test_map_despeckle.py:139 | A clean map is returned unchanged | — | failed as expected (1/1 mutants) | **KEEP** |
| 348 | `test_the_default_threshold_is_a_10cm_patch` | pure_pursuit/test_map_despeckle.py:146 | The default threshold is a 10cm patch | — | no mutant reached it | **KEEP (unverified)** |
| 349 | `test_a_despeckled_map_no_longer_blocks_the_cell` | pure_pursuit/test_map_despeckle.py:151 | The point of all this: the phantom stops costing clearance | — | failed as expected (6/6 mutants) | **KEEP** |
| 350 | `test_despeckle_map_file_cleans_the_image_in_place` | pure_pursuit/test_map_despeckle.py:179 | Despeckle map file cleans the image in place | — | failed as expected (4/4 mutants) | **KEEP** |
| 351 | `test_despeckle_map_file_leaves_a_clean_map_byte_identical` | pure_pursuit/test_map_despeckle.py:199 | Despeckle map file leaves a clean map byte identical | — | failed as expected (2/2 mutants) | **KEEP** |
| 352 | `test_despeckle_map_file_keeps_an_object_with_a_shadow` | pure_pursuit/test_map_despeckle.py:208 | Despeckle map file keeps an object with a shadow | — | failed as expected (4/4 mutants) | **KEEP** |
| 353 | `test_from_grid_message_despeckles_only_when_asked` | pure_pursuit/test_map_despeckle.py:217 | Off by default, because most callers read /map to avoid things | — | failed as expected (3/3 mutants) | **KEEP** |
| 354 | `test_the_real_saved_map_loses_only_phantoms_in_clear_track` | pure_pursuit/test_map_despeckle.py:235 | Regression against this car's actual map, if it is still on disk | — | failed as expected (3/3 mutants) | **KEEP** |
| 355 | `test_drives_normally_on_a_clear_track` | pure_pursuit/test_opponent_integration.py:152 | Drives normally on a clear track | A5 (bare approx, no tolerance) | failed as expected (55/68 mutants) | **KEEP (nit)** |
| 356 | `test_shaped_speed_ramps_up_to_the_profiled_speed` | pure_pursuit/test_opponent_integration.py:170 | Shaped speed ramps up to the profiled speed | A5 (bare approx, no tolerance) | failed as expected (52/65 mutants) | **KEEP (nit)** |
| 357 | `test_missing_lidar_stop_has_a_diagnostic_reason` | pure_pursuit/test_opponent_integration.py:192 | Missing lidar stop has a diagnostic reason | — | failed as expected (38/50 mutants) | **KEEP** |
| 358 | `test_overtakes_toward_the_more_open_side` | pure_pursuit/test_opponent_integration.py:211 | Overtakes toward the more open side | — | failed as expected (63/75 mutants) | **KEEP** |
| 359 | `test_overtakes_toward_the_right_when_thats_more_open` | pure_pursuit/test_opponent_integration.py:220 | Overtakes toward the right when thats more open | — | failed as expected (63/75 mutants) | **KEEP** |
| 360 | `test_the_chosen_pass_side_actually_moves_the_steering_that_way` | pure_pursuit/test_opponent_integration.py:229 | Passing right must steer further right than passing left, from the | — | failed as expected (63/75 mutants) | **KEEP** |
| 361 | `test_acceleration_ramp_starts_from_measured_speed_not_the_last_command` | pure_pursuit/test_opponent_integration.py:267 | Acceleration ramp starts from measured speed not the last command | A5 (bare approx, no tolerance) | failed as expected (53/66 mutants) | **KEEP (nit)** |
| 362 | `test_ramp_basis_never_exceeds_the_configured_speed_ceiling` | pure_pursuit/test_opponent_integration.py:296 | Ramp basis never exceeds the configured speed ceiling | — | failed as expected (52/65 mutants) | **KEEP** |
| 363 | `test_overtake_aims_further_ahead_than_the_normal_steering_target` | pure_pursuit/test_opponent_integration.py:307 | Overtake aims further ahead than the normal steering target | — | failed as expected (63/75 mutants) | **KEEP** |
| 364 | `test_overtake_preview_shorter_than_the_lookahead_is_rejected` | pure_pursuit/test_opponent_integration.py:333 | Overtake preview shorter than the lookahead is rejected | — | failed as expected (2/2 mutants) | **KEEP** |
| 365 | `test_hard_stop_overrides_an_active_overtake` | pure_pursuit/test_opponent_integration.py:348 | Hard stop overrides an active overtake | — | failed as expected (63/75 mutants) | **KEEP** |
| 366 | `test_forward_cone_obstacle_crawls_out_rather_than_latching` | pure_pursuit/test_opponent_integration.py:368 | An obstacle inside the forward cone with an open track beside it is | A5 (bare approx, no tolerance) | failed as expected (54/66 mutants) | **KEEP (nit)** |
| 367 | `test_forward_cone_hard_stop_still_fires_with_nowhere_to_go` | pure_pursuit/test_opponent_integration.py:393 | With no opening wide enough to crawl toward, the hard stop stands | — | failed as expected (56/68 mutants) | **KEEP** |
| 368 | `test_emergency_escape_yields_to_the_body_contact_tier` | pure_pursuit/test_opponent_integration.py:412 | Nearly touching means stop, not nudge -- whatever the cone shows | — | failed as expected (46/58 mutants) | **KEEP** |
| 369 | `test_wall_alongside_the_car_stops_it_although_the_forward_cone_is_clear` | pure_pursuit/test_opponent_integration.py:432 | Regression test for the 2026-07-27 collision | — | failed as expected (46/58 mutants) | **KEEP** |
| 370 | `test_overtake_resolves_once_ego_is_past_the_opponent` | pure_pursuit/test_opponent_integration.py:460 | Overtake resolves once ego is past the opponent | — | failed as expected (64/76 mutants) | **KEEP** |
| 371 | `test_overtake_stays_active_while_the_cars_still_overlap` | pure_pursuit/test_opponent_integration.py:488 | The completion-test defect, at the node level | — | failed as expected (63/75 mutants) | **KEEP** |
| 372 | `test_recovers_after_a_localization_jump` | pure_pursuit/test_opponent_integration.py:516 | Recovers after a localization jump | — | failed as expected (52/65 mutants) | **KEEP** |
| 373 | `test_stays_stopped_when_genuinely_lost` | pure_pursuit/test_opponent_integration.py:536 | Stays stopped when genuinely lost | — | failed as expected (31/45 mutants) | **KEEP** |
| 374 | `test_overtake_survives_losing_sight_of_the_opponent_mid_pass` | pure_pursuit/test_opponent_integration.py:547 | Overtake survives losing sight of the opponent mid pass | — | failed as expected (63/75 mutants) | **KEEP** |
| 375 | `test_overtake_aborts_after_too_long_blind` | pure_pursuit/test_opponent_integration.py:568 | Overtake aborts after too long blind | — | failed as expected (64/76 mutants) | **KEEP** |
| 376 | `test_map_mode_falls_back_to_heuristic_until_a_map_arrives` | pure_pursuit/test_opponent_integration.py:625 | Map mode falls back to heuristic until a map arrives | — | failed as expected (63/75 mutants) | **KEEP** |
| 377 | `test_map_mode_overtakes_using_map_subtraction` | pure_pursuit/test_opponent_integration.py:639 | Map mode overtakes using map subtraction | — | failed as expected (67/79 mutants) | **KEEP** |
| 378 | `test_map_subtraction_ignores_what_the_map_explains` | pure_pursuit/test_opponent_integration.py:657 | Map subtraction ignores what the map explains | A2 (shape/None only) | failed as expected (21/22 mutants) | **KEEP (nit)** |
| 379 | `test_map_subtraction_detects_an_unmapped_car_directly` | pure_pursuit/test_opponent_integration.py:673 | Map subtraction detects an unmapped car directly | — | failed as expected (22/23 mutants) | **KEEP** |
| 380 | `test_opponent_progress_rate_wraps_cleanly_at_start_finish` | pure_pursuit/test_opponent_integration.py:692 | Opponent progress rate wraps cleanly at start finish | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 381 | `test_waiting_node_loads_generated_profile_at_runtime` | pure_pursuit/test_opponent_integration.py:699 | Waiting node loads generated profile at runtime | — | failed as expected (27/41 mutants) | **KEEP** |
| 382 | `test_a_racing_tick_publishes_a_valid_intent` | pure_pursuit/test_pure_pursuit_intent.py:131 | A racing tick publishes a valid intent | — | failed as expected (71/72 mutants) | **KEEP** |
| 383 | `test_the_intent_reports_the_command_that_was_actually_published` | pure_pursuit/test_pure_pursuit_intent.py:141 | The intent reports the command that was actually published | — | failed as expected (65/66 mutants) | **KEEP** |
| 384 | `test_the_predicted_path_starts_at_the_car` | pure_pursuit/test_pure_pursuit_intent.py:151 | Published in base_link, so the first sample is the origin by | — | failed as expected (65/66 mutants) | **KEEP** |
| 385 | `test_the_speed_profile_varies_along_the_predicted_path` | pure_pursuit/test_pure_pursuit_intent.py:162 | Pure_pursuit's plan is not one speed held for a horizon: it is the | — | failed as expected (65/66 mutants) | **KEEP** |
| 386 | `test_the_steering_target_is_reported_ahead_of_the_car` | pure_pursuit/test_pure_pursuit_intent.py:172 | The steering target is reported ahead of the car | — | failed as expected (65/66 mutants) | **KEEP** |
| 387 | `test_the_speed_ceilings_are_named_and_the_lowest_binds` | pure_pursuit/test_pure_pursuit_intent.py:180 | The speed ceilings are named and the lowest binds | A5 (bare approx, no tolerance) | failed as expected (65/66 mutants) | **KEEP (nit)** |
| 388 | `test_the_arrow_follows_the_racing_line_round_a_corner` | pure_pursuit/test_pure_pursuit_intent.py:194 | Placed at a corner of the stadium, the predicted path must bend | — | failed as expected (66/67 mutants) | **KEEP** |
| 389 | `test_the_predicted_path_tracks_the_line_rather_than_the_current_heading` | pure_pursuit/test_pure_pursuit_intent.py:212 | Deliberately point the car off the line and check the prediction | — | failed as expected (66/67 mutants) | **KEEP** |
| 390 | `test_a_reactive_override_falls_back_to_a_single_arc` | pure_pursuit/test_pure_pursuit_intent.py:233 | When the reactive net takes over, following the line is no longer | — | failed as expected (69/70 mutants) | **KEEP** |
| 391 | `test_waiting_for_a_pose_publishes_a_stop_intent` | pure_pursuit/test_pure_pursuit_intent.py:253 | Waiting for a pose publishes a stop intent | A5 (bare approx, no tolerance) | failed as expected (40/41 mutants) | **KEEP (nit)** |
| 392 | `test_a_transition_carries_its_reason` | pure_pursuit/test_pure_pursuit_intent.py:262 | A transition carries its reason | — | failed as expected (68/69 mutants) | **KEEP** |
| 393 | `test_a_steady_state_does_not_repeat_its_reason_every_tick` | pure_pursuit/test_pure_pursuit_intent.py:272 | A steady state does not repeat its reason every tick | — | failed as expected (66/67 mutants) | **KEEP** |
| 394 | `test_the_drive_command_is_published_before_the_intent` | pure_pursuit/test_pure_pursuit_intent.py:283 | The drive command is published before the intent | — | failed as expected (64/65 mutants) | **KEEP** |
| 395 | `test_a_broken_intent_builder_does_not_stop_the_car_driving` | pure_pursuit/test_pure_pursuit_intent.py:295 | A broken intent builder does not stop the car driving | — | failed as expected (53/65 mutants) | **KEEP** |
| 396 | `test_sustained_intent_failure_switches_intent_off_not_the_node` | pure_pursuit/test_pure_pursuit_intent.py:308 | Sustained intent failure switches intent off not the node | — | failed as expected (54/66 mutants) | **KEEP** |
| 397 | `test_a_broken_path_predictor_does_not_stop_the_car_driving` | pure_pursuit/test_pure_pursuit_intent.py:322 | The prediction touches more of the node's state than the encoder | — | failed as expected (53/54 mutants) | **KEEP** |
| 398 | `test_publish_intent_false_creates_no_publisher_at_all` | pure_pursuit/test_pure_pursuit_intent.py:339 | Publish intent false creates no publisher at all | — | failed as expected (51/51 mutants) | **KEEP** |
| 399 | `test_a_value_inside_its_bounds_is_accepted` | pure_pursuit/test_pure_pursuit_live_tuning.py:60 | A value inside its bounds is accepted | — | failed as expected (5/5 mutants) | **KEEP** |
| 400 | `test_bounds_are_inclusive` | pure_pursuit/test_pure_pursuit_live_tuning.py:66 | Bounds are inclusive | — | failed as expected (5/5 mutants) | **KEEP** |
| 401 | `test_integers_are_accepted_as_floats` | pure_pursuit/test_pure_pursuit_live_tuning.py:77 | A slider landing exactly on 2 must not fail where 2.1 works | A2 (shape/None only) | failed as expected (5/5 mutants) | **KEEP (nit)** |
| 402 | `test_passthrough_names_are_ignored_not_refused` | pure_pursuit/test_pure_pursuit_live_tuning.py:83 | Passthrough names are ignored not refused | A2 (shape/None only) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 403 | `test_a_bool_tunable_round_trips` | pure_pursuit/test_pure_pursuit_live_tuning.py:88 | A bool tunable round trips | A2 (shape/None only) | failed as expected (5/5 mutants) | **KEEP (nit)** |
| 404 | `test_out_of_range_is_refused` | pure_pursuit/test_pure_pursuit_live_tuning.py:98 | Out of range is refused | — | failed as expected (2/2 mutants) | **KEEP** |
| 405 | `test_an_unknown_parameter_is_refused_rather_than_ignored` | pure_pursuit/test_pure_pursuit_live_tuning.py:104 | The whole reason this module exists -- a silently accepted change | — | failed as expected (1/1 mutants) | **KEEP** |
| 406 | `test_the_deadman_cannot_be_switched_off_at_runtime` | pure_pursuit/test_pure_pursuit_live_tuning.py:112 | The deadman cannot be switched off at runtime | — | failed as expected (1/1 mutants) | **KEEP** |
| 407 | `test_the_reactive_safety_net_cannot_be_switched_off_at_runtime` | pure_pursuit/test_pure_pursuit_live_tuning.py:118 | The reactive safety net cannot be switched off at runtime | — | failed as expected (1/1 mutants) | **KEEP** |
| 408 | `test_non_finite_is_refused` | pure_pursuit/test_pure_pursuit_live_tuning.py:125 | Non finite is refused | — | failed as expected (2/2 mutants) | **KEEP** |
| 409 | `test_a_bool_for_a_float_is_refused` | pure_pursuit/test_pure_pursuit_live_tuning.py:130 | A bool for a float is refused | — | failed as expected (2/2 mutants) | **KEEP** |
| 410 | `test_a_number_for_a_bool_is_refused` | pure_pursuit/test_pure_pursuit_live_tuning.py:135 | A number for a bool is refused | — | failed as expected (2/2 mutants) | **KEEP** |
| 411 | `test_a_string_is_refused` | pure_pursuit/test_pure_pursuit_live_tuning.py:140 | A string is refused | — | failed as expected (2/2 mutants) | **KEEP** |
| 412 | `test_one_bad_value_rejects_the_whole_batch` | pure_pursuit/test_pure_pursuit_live_tuning.py:149 | Half a speed change landing is its own hazard | — | failed as expected (2/2 mutants) | **KEEP** |
| 413 | `test_min_speed_cannot_exceed_max_speed` | pure_pursuit/test_pure_pursuit_live_tuning.py:156 | Min speed cannot exceed max speed | — | failed as expected (3/3 mutants) | **KEEP** |
| 414 | `test_a_batch_that_satisfies_the_invariant_together_is_accepted` | pure_pursuit/test_pure_pursuit_live_tuning.py:162 | Min_speed alone would be illegal; raising max_speed with it is not, | — | failed as expected (5/5 mutants) | **KEEP** |
| 415 | `test_min_lookahead_cannot_exceed_max_lookahead` | pure_pursuit/test_pure_pursuit_live_tuning.py:170 | Min lookahead cannot exceed max lookahead | — | failed as expected (4/4 mutants) | **KEEP** |
| 416 | `test_max_lookahead_cannot_exceed_the_overtake_preview` | pure_pursuit/test_pure_pursuit_live_tuning.py:176 | Max lookahead cannot exceed the overtake preview | — | failed as expected (5/5 mutants) | **KEEP** |
| 417 | `test_the_shipped_config_is_inside_every_bound` | pure_pursuit/test_pure_pursuit_live_tuning.py:188 | A default outside its own tunable range would mean the panel opens | — | no mutant reached it | **KEEP (unverified)** |
| 418 | `test_the_shipped_config_satisfies_its_own_invariants` | pure_pursuit/test_pure_pursuit_live_tuning.py:201 | The shipped config satisfies its own invariants | A2 (shape/None only) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 419 | `test_parameters_that_must_never_be_browser_tunable` | pure_pursuit/test_pure_pursuit_live_tuning.py:211 | Parameters that must never be browser tunable | — | no mutant reached it | **KEEP (unverified)** |
| 420 | `test_every_tunable_is_declared_by_the_node` | pure_pursuit/test_pure_pursuit_live_tuning.py:215 | Every tunable is declared by the node | — | no mutant reached it | **KEEP (unverified)** |
| 421 | `test_every_tunable_attr_is_actually_assigned_by_the_node` | pure_pursuit/test_pure_pursuit_live_tuning.py:222 | Catches a typo in `attr`, which would otherwise make setattr() | — | failed as expected (1/1 mutants) | **KEEP** |
| 422 | `test_every_tunable_is_documented_and_bounded` | pure_pursuit/test_pure_pursuit_live_tuning.py:233 | Every tunable is documented and bounded | — | no mutant reached it | **KEEP (unverified)** |
| 423 | `test_safety_flagged_parameters_are_the_expected_ones` | pure_pursuit/test_pure_pursuit_live_tuning.py:240 | A change here should be deliberate: the flag drives the warning | — | no mutant reached it | **KEEP (unverified)** |
| 424 | `test_spec_json_describes_every_tunable` | pure_pursuit/test_pure_pursuit_live_tuning.py:254 | Spec json describes every tunable | — | failed as expected (2/2 mutants) | **KEEP** |
| 425 | `test_spec_is_parseable_by_the_dashboard` | pure_pursuit/test_pure_pursuit_live_tuning.py:266 | The producer and the consumer live in different packages and can | — | failed as expected (3/3 mutants) | **KEEP** |
| 426 | `test_the_generic_half_matches_gap_follows_copy` | pure_pursuit/test_pure_pursuit_live_tuning.py:286 | This workspace duplicates rather than imports across packages (see | — | no mutant reached it | **KEEP (unverified)** |
| 427 | `test_resample_gives_uniform_spacing` | pure_pursuit/test_raceline_optimizer.py:48 | Resample gives uniform spacing | — | failed as expected (1/1 mutants) | **KEEP** |
| 428 | `test_resample_preserves_the_shape` | pure_pursuit/test_raceline_optimizer.py:56 | Resample preserves the shape | — | failed as expected (1/1 mutants) | **KEEP** |
| 429 | `test_normals_point_left_of_travel` | pure_pursuit/test_raceline_optimizer.py:61 | Normals point left of travel | — | failed as expected (1/1 mutants) | **KEEP** |
| 430 | `test_signed_curvature_matches_the_radius_and_keeps_the_sign` | pure_pursuit/test_raceline_optimizer.py:67 | Signed curvature matches the radius and keeps the sign | — | failed as expected (1/1 mutants) | **KEEP** |
| 431 | `test_signed_curvature_is_zero_on_a_straight` | pure_pursuit/test_raceline_optimizer.py:74 | Signed curvature is zero on a straight | — | failed as expected (1/1 mutants) | **KEEP** |
| 432 | `test_curvature_limit_matches_the_bicycle_model` | pure_pursuit/test_raceline_optimizer.py:79 | Curvature limit matches the bicycle model | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 433 | `test_circular_track_takes_the_outer_wall` | pure_pursuit/test_raceline_optimizer.py:89 | The largest circle that fits inside an annulus is the outer one, and | — | failed as expected (14/14 mutants) | **KEEP** |
| 434 | `test_circular_track_answer_does_not_depend_on_lap_direction` | pure_pursuit/test_raceline_optimizer.py:101 | Circular track answer does not depend on lap direction | — | failed as expected (14/14 mutants) | **KEEP** |
| 435 | `test_the_optimizer_reduces_the_objective_it_claims_to` | pure_pursuit/test_raceline_optimizer.py:111 | The optimizer reduces the objective it claims to | — | failed as expected (14/14 mutants) | **KEEP** |
| 436 | `test_corners_are_opened_out` | pure_pursuit/test_raceline_optimizer.py:121 | On a closed oval the corners are the whole cost, so the line has to | — | failed as expected (14/14 mutants) | **KEEP** |
| 437 | `test_the_whole_corridor_gets_used` | pure_pursuit/test_raceline_optimizer.py:137 | A racing line is not a constant offset. It has to move across the | — | failed as expected (14/14 mutants) | **KEEP** |
| 438 | `test_the_answer_beats_every_constant_offset` | pure_pursuit/test_raceline_optimizer.py:151 | The cheapest way to fake this optimizer would be to shift the whole | — | failed as expected (14/14 mutants) | **KEEP** |
| 439 | `test_the_answer_is_a_local_minimum` | pure_pursuit/test_raceline_optimizer.py:166 | Probe the result with smooth random moves inside the corridor. If any | — | failed as expected (14/14 mutants) | **KEEP** |
| 440 | `test_the_line_stays_inside_the_corridor` | pure_pursuit/test_raceline_optimizer.py:198 | The line stays inside the corridor | — | failed as expected (14/14 mutants) | **KEEP** |
| 441 | `test_asymmetric_widths_are_respected` | pure_pursuit/test_raceline_optimizer.py:210 | Room on one side only: the line may use that side and not the other | — | failed as expected (14/14 mutants) | **KEEP** |
| 442 | `test_a_corridor_narrower_than_the_car_is_reported_not_hidden` | pure_pursuit/test_raceline_optimizer.py:224 | A corridor narrower than the car is reported not hidden | A5 (bare approx, no tolerance) | failed as expected (14/14 mutants) | **KEEP (nit)** |
| 443 | `test_rejects_a_non_positive_vehicle_width` | pure_pursuit/test_raceline_optimizer.py:232 | Rejects a non positive vehicle width | — | failed as expected (1/1 mutants) | **KEEP** |
| 444 | `test_rejects_a_non_positive_trust_region` | pure_pursuit/test_raceline_optimizer.py:239 | Rejects a non positive trust region | — | failed as expected (9/9 mutants) | **KEEP** |
| 445 | `test_the_trust_region_bounds_a_single_pass` | pure_pursuit/test_raceline_optimizer.py:247 | The trust region bounds a single pass | — | failed as expected (14/14 mutants) | **KEEP** |
| 446 | `test_world_to_cell_round_trips` | pure_pursuit/test_raceline_optimizer.py:272 | World to cell round trips | — | failed as expected (2/2 mutants) | **KEEP** |
| 447 | `test_free_and_blocked_agree_with_the_geometry` | pure_pursuit/test_raceline_optimizer.py:278 | Free and blocked agree with the geometry | — | failed as expected (5/5 mutants) | **KEEP** |
| 448 | `test_cast_ray_finds_the_wall` | pure_pursuit/test_raceline_optimizer.py:285 | Cast ray finds the wall | — | failed as expected (6/6 mutants) | **KEEP** |
| 449 | `test_cast_ray_reports_max_distance_when_nothing_is_hit` | pure_pursuit/test_raceline_optimizer.py:293 | Cast ray reports max distance when nothing is hit | A5 (bare approx, no tolerance) | failed as expected (6/6 mutants) | **KEEP (nit)** |
| 450 | `test_clearance_field_matches_the_corridor` | pure_pursuit/test_raceline_optimizer.py:298 | Clearance field matches the corridor | — | failed as expected (6/6 mutants) | **KEEP** |
| 451 | `test_rejects_a_rotated_map_origin` | pure_pursuit/test_raceline_optimizer.py:304 | Rejects a rotated map origin | — | failed as expected (1/1 mutants) | **KEEP** |
| 452 | `test_refine_centerline_recovers_the_middle_of_a_ring` | pure_pursuit/test_raceline_optimizer.py:316 | Refine centerline recovers the middle of a ring | — | failed as expected (13/13 mutants) | **KEEP** |
| 453 | `test_refine_centerline_then_optimize_stays_off_the_walls` | pure_pursuit/test_raceline_optimizer.py:328 | End to end on a synthetic map: extract, optimize, and check the result | — | failed as expected (27/27 mutants) | **KEEP** |
| 454 | `test_quaternion_to_yaw_identity` | pure_pursuit/test_racing_math.py:34 | Quaternion to yaw identity | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 455 | `test_quaternion_to_yaw_90_degrees` | pure_pursuit/test_racing_math.py:38 | Quaternion to yaw 90 degrees | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 456 | `test_world_to_body_straight_ahead` | pure_pursuit/test_racing_math.py:44 | World to body straight ahead | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 457 | `test_world_to_body_target_to_the_left` | pure_pursuit/test_racing_math.py:52 | World to body target to the left | — | failed as expected (1/1 mutants) | **KEEP** |
| 458 | `test_world_to_body_accounts_for_heading` | pure_pursuit/test_racing_math.py:57 | World to body accounts for heading | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 459 | `test_steering_arc_curvature_target_dead_ahead_is_straight` | pure_pursuit/test_racing_math.py:69 | Steering arc curvature target dead ahead is straight | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 460 | `test_steering_arc_curvature_sign_matches_left_right` | pure_pursuit/test_racing_math.py:74 | Steering arc curvature sign matches left right | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 461 | `test_steering_from_curvature_matches_bicycle_model` | pure_pursuit/test_racing_math.py:82 | Steering from curvature matches bicycle model | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 462 | `test_pure_pursuit_end_to_end_steers_left_for_a_left_target` | pure_pursuit/test_racing_math.py:89 | Pure pursuit end to end steers left for a left target | — | failed as expected (3/3 mutants) | **KEEP** |
| 463 | `test_adaptive_lookahead_clips_to_bounds` | pure_pursuit/test_racing_math.py:101 | Adaptive lookahead clips to bounds | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 464 | `test_online_curvature_speed_limit` | pure_pursuit/test_racing_math.py:110 | Online curvature speed limit | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 465 | `test_command_slew_rate_limit` | pure_pursuit/test_racing_math.py:117 | Command slew rate limit | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 466 | `test_segment_lengths_closed_vs_open` | pure_pursuit/test_racing_math.py:130 | Segment lengths closed vs open | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 467 | `test_find_nearest_index_and_cross_track_error` | pure_pursuit/test_racing_math.py:140 | Find nearest index and cross track error | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 468 | `test_find_nearest_index_search_window_restricts_candidates` | pure_pursuit/test_racing_math.py:147 | Find nearest index search window restricts candidates | — | failed as expected (1/1 mutants) | **KEEP** |
| 469 | `test_find_lookahead_index_walks_forward_expected_distance` | pure_pursuit/test_racing_math.py:158 | Find lookahead index walks forward expected distance | — | failed as expected (2/2 mutants) | **KEEP** |
| 470 | `test_find_lookahead_index_wraps_around_a_closed_loop` | pure_pursuit/test_racing_math.py:167 | Find lookahead index wraps around a closed loop | — | failed as expected (2/2 mutants) | **KEEP** |
| 471 | `test_curvature_of_a_circle_matches_1_over_r` | pure_pursuit/test_racing_math.py:189 | Curvature of a circle matches 1 over r | — | failed as expected (1/1 mutants) | **KEEP** |
| 472 | `test_curvature_of_a_straight_line_is_near_zero` | pure_pursuit/test_racing_math.py:196 | Curvature of a straight line is near zero | — | failed as expected (1/1 mutants) | **KEEP** |
| 473 | `test_velocity_profile_uniform_on_constant_curvature_loop` | pure_pursuit/test_racing_math.py:202 | Velocity profile uniform on constant curvature loop | — | failed as expected (3/3 mutants) | **KEEP** |
| 474 | `test_velocity_profile_brakes_before_a_tight_corner` | pure_pursuit/test_racing_math.py:226 | Velocity profile brakes before a tight corner | — | failed as expected (3/3 mutants) | **KEEP** |
| 475 | `test_velocity_profile_never_exceeds_braking_capability` | pure_pursuit/test_racing_math.py:244 | Velocity profile never exceeds braking capability | A5 (assert may never run) | FALSELY PASSED on `return <const>` (stub killed it; sibling catches it) | **REWRITE** |
| 476 | `test_velocity_profile_respects_v_max_and_v_min` | pure_pursuit/test_racing_math.py:261 | Velocity profile respects v max and v min | — | failed as expected (3/3 mutants) | **KEEP** |
| 477 | `test_estimate_lap_time_positive_and_matches_constant_speed_case` | pure_pursuit/test_racing_math.py:272 | Estimate lap time positive and matches constant speed case | — | failed as expected (2/2 mutants) | **KEEP** |
| 478 | `test_csv_round_trip` | pure_pursuit/test_racing_math.py:285 | Csv round trip | — | failed as expected (2/2 mutants) | **KEEP** |
| 479 | `test_load_profiled_csv_rejects_raw_file` | pure_pursuit/test_racing_math.py:295 | Load profiled csv rejects raw file | — | failed as expected (2/2 mutants) | **KEEP** |
| 480 | `test_cluster_scan_ranges_separates_close_object_from_open_track` | pure_pursuit/test_racing_math.py:307 | Cluster scan ranges separates close object from open track | — | failed as expected (1/1 mutants) | **KEEP** |
| 481 | `test_cluster_scan_ranges_splits_on_a_big_jump_between_two_close_objects` | pure_pursuit/test_racing_math.py:314 | Cluster scan ranges splits on a big jump between two close objects | — | failed as expected (1/1 mutants) | **KEEP** |
| 482 | `test_cluster_geometry_matches_known_chord_length` | pure_pursuit/test_racing_math.py:322 | Cluster geometry matches known chord length | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 483 | `test_detect_opponent_cluster_finds_car_sized_object_in_the_open` | pure_pursuit/test_racing_math.py:343 | Detect opponent cluster finds car sized object in the open | A5 (bare approx, no tolerance) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 484 | `test_detect_opponent_cluster_ignores_things_too_far_to_matter_yet` | pure_pursuit/test_racing_math.py:354 | Detect opponent cluster ignores things too far to matter yet | A2 (shape/None only) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 485 | `test_detect_opponent_cluster_rejects_wall_shaped_cluster` | pure_pursuit/test_racing_math.py:362 | Detect opponent cluster rejects wall shaped cluster | A2 (shape/None only) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 486 | `test_detect_opponent_cluster_rejects_when_flush_against_another_object` | pure_pursuit/test_racing_math.py:372 | Detect opponent cluster rejects when flush against another object | A2 (shape/None only) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 487 | `test_compute_cumulative_arc_length` | pure_pursuit/test_racing_math.py:385 | Compute cumulative arc length | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 488 | `test_track_progress_gap_simple_case` | pure_pursuit/test_racing_math.py:394 | Track progress gap simple case | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 489 | `test_track_progress_gap_wraps_around_finish_line` | pure_pursuit/test_racing_math.py:399 | Track progress gap wraps around finish line | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 490 | `test_track_lead_distance_is_negative_while_the_opponent_leads` | pure_pursuit/test_racing_math.py:406 | Track lead distance is negative while the opponent leads | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 491 | `test_track_lead_distance_is_positive_once_the_ego_is_past` | pure_pursuit/test_racing_math.py:412 | Track lead distance is positive once the ego is past | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 492 | `test_track_lead_distance_wraps_around_the_finish_line` | pure_pursuit/test_racing_math.py:418 | Track lead distance wraps around the finish line | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 493 | `test_overtake_is_not_complete_while_the_cars_are_still_alongside` | pure_pursuit/test_racing_math.py:426 | The bug this function exists to prevent | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 494 | `test_approaching_an_opponent_never_reads_as_a_finished_pass` | pure_pursuit/test_racing_math.py:448 | Approaching an opponent never reads as a finished pass | — | failed as expected (1/1 mutants) | **KEEP** |
| 495 | `test_side_clearance_recovers_the_perpendicular_wall_distance` | pure_pursuit/test_racing_math.py:471 | Side clearance recovers the perpendicular wall distance | — | failed as expected (1/1 mutants) | **KEEP** |
| 496 | `test_side_clearance_reports_infinity_when_nothing_is_in_range` | pure_pursuit/test_racing_math.py:483 | Side clearance reports infinity when nothing is in range | — | failed as expected (1/1 mutants) | **KEEP** |
| 497 | `test_overtake_declined_when_the_chosen_side_is_a_wall` | pure_pursuit/test_racing_math.py:492 | The defect this guards | — | failed as expected (2/2 mutants) | **KEEP** |
| 498 | `test_overtake_allowed_when_no_wall_is_seen_at_all` | pure_pursuit/test_racing_math.py:508 | Overtake allowed when no wall is seen at all | — | failed as expected (2/2 mutants) | **KEEP** |
| 499 | `test_braking_speed_limit_allows_stopping_short` | pure_pursuit/test_racing_math.py:518 | Braking speed limit allows stopping short | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 500 | `test_braking_speed_limit_leaves_top_speed_alone_when_nothing_is_seen` | pure_pursuit/test_racing_math.py:526 | Braking speed limit leaves top speed alone when nothing is seen | — | failed as expected (1/1 mutants) | **KEEP** |
| 501 | `test_lateral_offset_point_moves_left_on_a_straight_segment` | pure_pursuit/test_racing_math.py:530 | Lateral offset point moves left on a straight segment | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 502 | `test_lateral_offset_point_negative_offset_moves_right` | pure_pursuit/test_racing_math.py:537 | Lateral offset point negative offset moves right | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 503 | `test_pick_pass_side_prefers_the_more_open_side_left` | pure_pursuit/test_racing_math.py:543 | Pick pass side prefers the more open side left | — | failed as expected (1/1 mutants) | **KEEP** |
| 504 | `test_pick_pass_side_prefers_the_more_open_side_right` | pure_pursuit/test_racing_math.py:551 | Pick pass side prefers the more open side right | — | failed as expected (1/1 mutants) | **KEEP** |
| 505 | `test_smooth_path_zero_window_returns_input_unchanged` | pure_pursuit/test_racing_math.py:563 | Smooth path zero window returns input unchanged | — | failed as expected (1/1 mutants) | **KEEP** |
| 506 | `test_smooth_path_removes_jitter_curvature_from_a_circle` | pure_pursuit/test_racing_math.py:570 | Smooth path removes jitter curvature from a circle | — | failed as expected (2/2 mutants) | **KEEP** |
| 507 | `test_smooth_path_closed_loop_has_no_seam` | pure_pursuit/test_racing_math.py:593 | Smooth path closed loop has no seam | — | failed as expected (1/1 mutants) | **KEEP** |
| 508 | `test_smooth_path_open_path_keeps_a_straight_line_straight` | pure_pursuit/test_racing_math.py:604 | Smooth path open path keeps a straight line straight | — | failed as expected (2/2 mutants) | **KEEP** |
| 509 | `test_friction_ellipse_profile_is_never_faster_than_uncoupled` | pure_pursuit/test_racing_math.py:618 | Friction ellipse profile is never faster than uncoupled | — | failed as expected (3/3 mutants) | **KEEP** |
| 510 | `test_friction_ellipse_profile_still_respects_braking_capability` | pure_pursuit/test_racing_math.py:634 | Friction ellipse profile still respects braking capability | A5 (assert may never run) | FALSELY PASSED on `return <const>` (stub killed it; sibling catches it) | **REWRITE** |
| 511 | `test_friction_ellipse_changes_nothing_on_a_constant_curvature_loop` | pure_pursuit/test_racing_math.py:654 | Friction ellipse changes nothing on a constant curvature loop | — | failed as expected (3/3 mutants) | **KEEP** |
| 512 | `test_dynamic_beam_mask_wall_present_in_map_is_not_flagged` | pure_pursuit/test_racing_math.py:672 | Dynamic beam mask wall present in map is not flagged | — | failed as expected (1/1 mutants) | **KEEP** |
| 513 | `test_dynamic_beam_mask_flags_only_beams_shorter_than_the_map_predicts` | pure_pursuit/test_racing_math.py:682 | Dynamic beam mask flags only beams shorter than the map predicts | — | failed as expected (1/1 mutants) | **KEEP** |
| 514 | `test_dynamic_beam_mask_never_flags_invalid_beams` | pure_pursuit/test_racing_math.py:691 | Dynamic beam mask never flags invalid beams | — | failed as expected (1/1 mutants) | **KEEP** |
| 515 | `test_detect_dynamic_cluster_finds_a_car_in_front_of_a_wall` | pure_pursuit/test_racing_math.py:705 | Detect dynamic cluster finds a car in front of a wall | A5 (bare approx, no tolerance) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 516 | `test_detect_dynamic_cluster_ignores_a_scene_the_map_fully_explains` | pure_pursuit/test_racing_math.py:721 | Detect dynamic cluster ignores a scene the map fully explains | A2 (shape/None only) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 517 | `test_detect_dynamic_cluster_ignores_objects_beyond_engagement_range` | pure_pursuit/test_racing_math.py:729 | Detect dynamic cluster ignores objects beyond engagement range | A2 (shape/None only) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 518 | `test_detect_dynamic_cluster_rejects_wall_sized_residuals` | pure_pursuit/test_racing_math.py:738 | Detect dynamic cluster rejects wall sized residuals | A2 (shape/None only) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 519 | `test_detect_dynamic_cluster_returns_the_closest_of_two_and_splits_on_depth` | pure_pursuit/test_racing_math.py:749 | Detect dynamic cluster returns the closest of two and splits on depth | A5 (bare approx, no tolerance) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 520 | `test_curvature_limit_matches_the_bicycle_model` | pure_pursuit/test_recorded_path.py:41 | Curvature limit matches the bicycle model | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 521 | `test_curvature_limit_rejects_impossible_geometry` | pure_pursuit/test_recorded_path.py:49 | Curvature limit rejects impossible geometry | — | failed as expected (1/1 mutants) | **KEEP** |
| 522 | `test_lowpass_keeps_the_loop_the_same_size` | pure_pursuit/test_recorded_path.py:56 | Regression: repeated moving-average smoothing is curve-shortening | — | failed as expected (2/2 mutants) | **KEEP** |
| 523 | `test_lowpass_removes_jitter_but_keeps_the_shape` | pure_pursuit/test_recorded_path.py:71 | Lowpass removes jitter but keeps the shape | — | failed as expected (1/1 mutants) | **KEEP** |
| 524 | `test_lowpass_leaves_short_paths_alone` | pure_pursuit/test_recorded_path.py:81 | Lowpass leaves short paths alone | — | failed as expected (1/1 mutants) | **KEEP** |
| 525 | `test_drop_repeated_points_removes_duplicates_including_the_wrap` | pure_pursuit/test_recorded_path.py:88 | Drop repeated points removes duplicates including the wrap | — | failed as expected (1/1 mutants) | **KEEP** |
| 526 | `test_duplicate_points_leave_a_zero_length_segment_behind` | pure_pursuit/test_recorded_path.py:96 | A car recorded twice in the same spot is not a corner, but it is a | — | failed as expected (2/2 mutants) | **KEEP** |
| 527 | `test_resample_gives_uniform_spacing` | pure_pursuit/test_recorded_path.py:113 | The recorded lap is sampled by distance travelled but never evenly: | — | failed as expected (1/1 mutants) | **KEEP** |
| 528 | `test_resample_rejects_degenerate_input` | pure_pursuit/test_recorded_path.py:134 | Resample rejects degenerate input | — | failed as expected (1/1 mutants) | **KEEP** |
| 529 | `test_last_revolution_trims_a_two_lap_recording` | pure_pursuit/test_recorded_path.py:141 | The defect this exists for: `minimum_lap_distance: 20.0` is longer | — | failed as expected (1/1 mutants) | **KEEP** |
| 530 | `test_last_revolution_leaves_a_single_lap_untouched` | pure_pursuit/test_recorded_path.py:153 | Last revolution leaves a single lap untouched | — | failed as expected (1/1 mutants) | **KEEP** |
| 531 | `test_last_revolution_survives_a_jittering_pose` | pure_pursuit/test_recorded_path.py:158 | An earlier implementation integrated heading to count revolutions | — | failed as expected (1/1 mutants) | **KEEP** |
| 532 | `test_last_revolution_ignores_paths_too_short_to_judge` | pure_pursuit/test_recorded_path.py:174 | Last revolution ignores paths too short to judge | — | failed as expected (1/1 mutants) | **KEEP** |
| 533 | `test_seam_heading_error_is_zero_on_a_clean_loop` | pure_pursuit/test_recorded_path.py:181 | Seam heading error is zero on a clean loop | — | failed as expected (1/1 mutants) | **KEEP** |
| 534 | `test_seam_heading_error_finds_a_kink` | pure_pursuit/test_recorded_path.py:185 | Measured on the real recordings: 34.8, 38.6 and 110.1 degrees across | — | failed as expected (1/1 mutants) | **KEEP** |
| 535 | `test_prepare_makes_a_jittered_lap_drivable` | pure_pursuit/test_recorded_path.py:195 | Prepare makes a jittered lap drivable | — | failed as expected (13/13 mutants) | **KEEP** |
| 536 | `test_prepare_barely_touches_an_already_clean_lap` | pure_pursuit/test_recorded_path.py:210 | Prepare barely touches an already clean lap | — | failed as expected (10/10 mutants) | **KEEP** |
| 537 | `test_prepare_trims_and_cleans_a_two_lap_jittered_recording` | pure_pursuit/test_recorded_path.py:219 | The exact shape of every real recording: two revolutions of a | — | failed as expected (12/12 mutants) | **KEEP** |
| 538 | `test_prepare_refuses_a_course_tighter_than_the_car` | pure_pursuit/test_recorded_path.py:231 | A 0.4m-radius loop is inside the car's 1.22m turning circle. No | — | failed as expected (13/13 mutants) | **KEEP** |
| 539 | `test_prepare_bounds_how_far_it_moves_the_driven_line` | pure_pursuit/test_recorded_path.py:244 | Filtering harder always looks better by curvature and eventually | — | failed as expected (10/10 mutants) | **KEEP** |
| 540 | `test_prepare_reports_a_seam_it_could_not_fix` | pure_pursuit/test_recorded_path.py:255 | Prepare reports a seam it could not fix | — | failed as expected (11/11 mutants) | **KEEP** |
| 541 | `test_prepare_rejects_input_that_is_not_a_path` | pure_pursuit/test_recorded_path.py:263 | Prepare rejects input that is not a path | — | failed as expected (3/3 mutants) | **KEEP** |
| 542 | `test_grading_separates_slightly_wide_from_undrivable` | pure_pursuit/test_recorded_path.py:274 | `feasible` and `acceptable` are different questions. A line a little | — | failed as expected (3/3 mutants) | **KEEP** |
| 543 | `test_clearance_is_not_checked_without_a_map` | pure_pursuit/test_recorded_path.py:308 | Silence is not a pass. With no map the result says so rather than | — | failed as expected (12/12 mutants) | **KEEP** |
| 544 | `test_a_prepared_line_closer_than_required_does_not_fit` | pure_pursuit/test_recorded_path.py:319 | The failure this check exists for. Filtering rounds a corner | — | failed as expected (5/5 mutants) | **KEEP** |
| 545 | `test_the_requirement_is_capped_by_what_driving_achieved` | pure_pursuit/test_recorded_path.py:349 | The lap that just happened is proof that a line fits. Where the map | A5 (bare approx, no tolerance) | failed as expected (12/12 mutants) | **KEEP (nit)** |
| 546 | `test_a_line_with_room_around_it_is_accepted` | pure_pursuit/test_recorded_path.py:364 | A line with room around it is accepted | — | failed as expected (12/12 mutants) | **KEEP** |
| 547 | `test_staying_inside_the_track_outranks_curvature` | pure_pursuit/test_recorded_path.py:374 | The two constraints pull opposite ways -- filtering harder always | — | failed as expected (11/11 mutants) | **KEEP** |
| 548 | `test_a_map_with_other_cars_in_it_does_not_veto_the_lap_that_happened` | pure_pursuit/test_recorded_path.py:393 | Mapping with traffic paints the other cars into the grid, so the | A5 (bare approx, no tolerance) | failed as expected (14/14 mutants) | **KEEP (nit)** |
| 549 | `test_recorder_reports_pose_health_and_progress` | pure_pursuit/test_waypoint_recorder_integration.py:19 | Recorder reports pose health and progress | A5 (bare approx, no tolerance) | failed as expected (6/6 mutants) | **KEEP (nit)** |
| 550 | `test_real_lines_classify_as_expected` | race_diagnostics/test_run_events.py:33 | Real lines classify as expected | — | failed as expected (2/2 mutants) | **KEEP** |
| 551 | `test_ordinary_drive_chatter_is_dropped` | race_diagnostics/test_run_events.py:39 | Ordinary drive chatter is dropped | — | failed as expected (2/2 mutants) | **KEEP** |
| 552 | `test_noisy_categories_are_throttled_but_still_counted` | race_diagnostics/test_run_events.py:45 | Noisy categories are throttled but still counted | — | failed as expected (2/2 mutants) | **KEEP** |
| 553 | `test_a_watchdog_stop_outranks_the_generic_stop_category` | race_diagnostics/test_run_events.py:55 | STOP [pose_frozen] must report as the watchdog that fired, not as | — | failed as expected (2/2 mutants) | **KEEP** |
| 554 | `test_lap_progress_parses_every_gate` | race_diagnostics/test_run_events.py:72 | Lap progress parses every gate | — | failed as expected (1/1 mutants) | **KEEP** |
| 555 | `test_blocking_gate_names_the_first_failing_gate_in_order` | race_diagnostics/test_run_events.py:83 | Blocking gate names the first failing gate in order | — | failed as expected (2/2 mutants) | **KEEP** |
| 556 | `test_the_heading_gate_is_identified_when_it_alone_fails` | race_diagnostics/test_run_events.py:87 | The real 2026-07-27 case: everything passed except heading, by | A3 (unsourced number); A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 557 | `test_a_fully_satisfied_sample_has_no_blocking_gate` | race_diagnostics/test_run_events.py:98 | A fully satisfied sample has no blocking gate | — | failed as expected (2/2 mutants) | **KEEP** |
| 558 | `test_not_having_departed_outranks_the_other_gates` | race_diagnostics/test_run_events.py:104 | At the very start every distance is trivially small; reporting | — | failed as expected (2/2 mutants) | **KEEP** |
| 559 | `test_non_lap_lines_parse_to_none` | race_diagnostics/test_run_events.py:112 | Non lap lines parse to none | — | failed as expected (2/2 mutants) | **KEEP** |
| 560 | `test_timeline_records_phases_and_worst_pose_lag` | race_diagnostics/test_run_events.py:117 | Timeline records phases and worst pose lag | A5 (bare approx, no tolerance) | failed as expected (4/4 mutants) | **KEEP (nit)** |
| 561 | `test_classifier_rejects_a_negative_throttle` | race_diagnostics/test_run_events.py:134 | Classifier rejects a negative throttle | — | failed as expected (1/1 mutants) | **KEEP** |
| 562 | `test_no_hardware_means_no_conflict` | racerbot_sim/test_hardware_guard.py:45 | No hardware means no conflict | — | failed as expected (3/3 mutants) | **KEEP** |
| 563 | `test_every_bringup_driver_blocks_the_simulator` | racerbot_sim/test_hardware_guard.py:52 | Every bringup driver blocks the simulator | — | failed as expected (4/4 mutants) | **KEEP** |
| 564 | `test_the_real_joystick_counts_as_hardware` | racerbot_sim/test_hardware_guard.py:61 | Joy_node is in bringup precisely so autonomy can check LB. A | — | no mutant reached it | **KEEP (unverified)** |
| 565 | `test_hardware_appearing_mid_run_suppresses_output` | racerbot_sim/test_hardware_guard.py:67 | The check is repeated, not just made at startup: bringing the car up | — | failed as expected (3/3 mutants) | **KEEP** |
| 566 | `test_output_resumes_once_the_drivers_are_gone` | racerbot_sim/test_hardware_guard.py:77 | Output resumes once the drivers are gone | — | failed as expected (4/4 mutants) | **KEEP** |
| 567 | `test_a_graph_query_failure_never_takes_the_node_down` | racerbot_sim/test_hardware_guard.py:87 | A graph query failure never takes the node down | — | failed as expected (1/1 mutants) | **KEEP** |
| 568 | `test_opponent_starts_where_it_was_asked_to` | racerbot_sim/test_sim_bridge.py:29 | Opponent starts where it was asked to | — | failed as expected (6/6 mutants) | **KEEP** |
| 569 | `test_opponent_start_offset_moves_it_along_the_loop` | racerbot_sim/test_sim_bridge.py:38 | Opponent start offset moves it along the loop | — | failed as expected (6/6 mutants) | **KEEP** |
| 570 | `test_a_parked_opponent_commands_nothing` | racerbot_sim/test_sim_bridge.py:46 | Speed 0 is a static obstacle on the line -- the case that used to | — | failed as expected (4/4 mutants) | **KEEP** |
| 571 | `test_opponent_steers_toward_the_loop_from_beside_it` | racerbot_sim/test_sim_bridge.py:54 | Opponent steers toward the loop from beside it | A5 (bare approx, no tolerance) | failed as expected (6/6 mutants) | **KEEP (nit)** |
| 572 | `test_opponent_steering_stays_inside_a_plausible_rack` | racerbot_sim/test_sim_bridge.py:63 | Opponent steering stays inside a plausible rack | — | failed as expected (6/6 mutants) | **KEEP** |
| 573 | `test_lateral_offset_moves_the_opponents_line_sideways` | racerbot_sim/test_sim_bridge.py:72 | Lateral offset moves the opponents line sideways | — | failed as expected (6/6 mutants) | **KEEP** |
| 574 | `test_odometry_integrates_a_straight_run` | racerbot_sim/test_sim_bridge.py:90 | Odometry integrates a straight run | — | failed as expected (2/2 mutants) | **KEEP** |
| 575 | `test_odometry_turns_from_the_commanded_steering_angle` | racerbot_sim/test_sim_bridge.py:99 | Vesc_to_odom runs with use_servo_cmd_to_calc_angular_velocity, so the | — | failed as expected (2/2 mutants) | **KEEP** |
| 576 | `test_odometry_scale_error_shows_up_as_distance_error` | racerbot_sim/test_sim_bridge.py:111 | Odometry scale error shows up as distance error | — | failed as expected (2/2 mutants) | **KEEP** |
| 577 | `test_odometry_reports_what_it_integrated` | racerbot_sim/test_sim_bridge.py:118 | Odometry reports what it integrated | A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 578 | `test_odometry_yaw_stays_wrapped` | racerbot_sim/test_sim_bridge.py:125 | Odometry yaw stays wrapped | — | failed as expected (2/2 mutants) | **KEEP** |
| 579 | `test_lidar_geometry_matches_the_real_sensor` | racerbot_sim/test_sim_bridge.py:134 | Hokuyo UST-10LX: 1081 beams at 0.25deg over 270deg | A5 (bare approx, no tolerance) | no mutant reached it | **KEEP (unverified)** |
| 580 | `test_vehicle_geometry_matches_the_car_configs` | racerbot_sim/test_sim_bridge.py:144 | These must agree with gap_follow.yaml and pure_pursuit.yaml, or the | — | no mutant reached it | **KEEP (unverified)** |
| 581 | `test_centerline_is_a_closed_loop_of_the_right_size` | racerbot_sim/test_tracks.py:23 | Centerline is a closed loop of the right size | — | failed as expected (1/1 mutants) | **KEEP** |
| 582 | `test_centerline_starts_on_a_straight_facing_positive_x` | racerbot_sim/test_tracks.py:31 | A car spawned mid-corner starts its mapping lap already steering, | — | failed as expected (1/1 mutants) | **KEEP** |
| 583 | `test_centerline_is_evenly_sampled` | racerbot_sim/test_tracks.py:41 | Centerline is evenly sampled | — | failed as expected (1/1 mutants) | **KEEP** |
| 584 | `test_centerline_rejects_a_radius_that_does_not_fit` | racerbot_sim/test_tracks.py:49 | Centerline rejects a radius that does not fit | — | failed as expected (1/1 mutants) | **KEEP** |
| 585 | `test_occupancy_is_free_on_the_line_and_walled_beside_it` | racerbot_sim/test_tracks.py:54 | Occupancy is free on the line and walled beside it | — | failed as expected (2/2 mutants) | **KEEP** |
| 586 | `test_every_named_layout_builds_and_is_big_enough_to_close_a_lap` | racerbot_sim/test_tracks.py:72 | `minimum_lap_distance` and `minimum_lap_duration_sec` are real gates | — | failed as expected (6/6 mutants) | **KEEP** |
| 587 | `test_layout_corners_are_inside_the_cars_turning_circle` | racerbot_sim/test_tracks.py:85 | Tan(0.26)/0.324 is a 1.22m minimum turning radius. A track with a | — | no mutant reached it | **KEEP (unverified)** |
| 588 | `test_unknown_layout_is_refused` | racerbot_sim/test_tracks.py:93 | Unknown layout is refused | — | failed as expected (1/1 mutants) | **KEEP** |
| 589 | `test_topic_stream_reports_missing_live_and_stale_frames` | usb_cam_stream/test_camera_stream_node.py:10 | Topic stream reports missing live and stale frames | — | failed as expected (9/9 mutants) | **KEEP** |
| 590 | `test_a_plain_request_gets_the_cheap_preview` | usb_cam_stream/test_stream_tiers.py:59 | A plain request gets the cheap preview | — | failed as expected (3/3 mutants) | **KEEP** |
| 591 | `test_the_recording_view_can_ask_for_the_full_stream` | usb_cam_stream/test_stream_tiers.py:65 | The recording view can ask for the full stream | — | failed as expected (3/3 mutants) | **KEEP** |
| 592 | `test_the_legacy_full_flag_still_works` | usb_cam_stream/test_stream_tiers.py:69 | The legacy full flag still works | — | failed as expected (3/3 mutants) | **KEEP** |
| 593 | `test_an_unknown_tier_falls_back_to_preview_rather_than_failing` | usb_cam_stream/test_stream_tiers.py:75 | An unknown tier falls back to preview rather than failing | — | failed as expected (3/3 mutants) | **KEEP** |
| 594 | `test_no_viewers_means_no_encoding_at_all` | usb_cam_stream/test_stream_tiers.py:83 | No viewers means no encoding at all | — | failed as expected (8/8 mutants) | **KEEP** |
| 595 | `test_only_the_watched_tier_is_encoded` | usb_cam_stream/test_stream_tiers.py:92 | Only the watched tier is encoded | A2 (shape/None only) | failed as expected (11/11 mutants) | **KEEP (nit)** |
| 596 | `test_both_tiers_are_encoded_when_both_are_watched` | usb_cam_stream/test_stream_tiers.py:100 | Both tiers are encoded when both are watched | A2 (shape/None only) | failed as expected (11/11 mutants) | **KEEP (nit)** |
| 597 | `test_the_preview_is_downscaled_to_its_configured_width` | usb_cam_stream/test_stream_tiers.py:113 | The preview is downscaled to its configured width | — | failed as expected (11/11 mutants) | **KEEP** |
| 598 | `test_the_full_tier_keeps_the_cameras_own_size` | usb_cam_stream/test_stream_tiers.py:125 | The full tier keeps the cameras own size | — | failed as expected (11/11 mutants) | **KEEP** |
| 599 | `test_the_preview_is_dramatically_smaller_on_the_wire` | usb_cam_stream/test_stream_tiers.py:134 | The whole point, asserted rather than assumed | — | failed as expected (11/11 mutants) | **KEEP** |
| 600 | `test_a_frame_smaller_than_the_preview_width_is_not_upscaled` | usb_cam_stream/test_stream_tiers.py:145 | A frame smaller than the preview width is not upscaled | — | failed as expected (11/11 mutants) | **KEEP** |
| 601 | `test_a_camera_jpeg_reaches_the_full_tier_untouched` | usb_cam_stream/test_stream_tiers.py:158 | No decode, no re-encode, and so no second generation of loss | — | failed as expected (10/10 mutants) | **KEEP** |
| 602 | `test_a_camera_jpeg_is_decoded_only_when_the_preview_needs_scaling` | usb_cam_stream/test_stream_tiers.py:170 | A camera jpeg is decoded only when the preview needs scaling | — | failed as expected (11/11 mutants) | **KEEP** |
| 603 | `test_recognising_a_camera_jpeg_from_a_decoded_frame` | usb_cam_stream/test_stream_tiers.py:187 | Recognising a camera jpeg from a decoded frame | — | failed as expected (1/1 mutants) | **KEEP** |
| 604 | `test_only_the_newest_frame_waiting_is_encoded` | usb_cam_stream/test_stream_tiers.py:203 | If encoding falls behind, dropping the backlog is what keeps latency | — | failed as expected (11/11 mutants) | **KEEP** |
| 605 | `test_each_stored_frame_advances_the_tier_sequence` | usb_cam_stream/test_stream_tiers.py:217 | The sequence is how a parked HTTP handler knows there is something | — | failed as expected (11/11 mutants) | **KEEP** |
| 606 | `test_the_topic_subscription_never_encodes_on_the_ros_thread` | usb_cam_stream/test_stream_tiers.py:227 | A subscription callback that encodes a 720p JPEG stalls every other | A2 (shape/None only) | failed as expected (12/12 mutants) | **KEEP (nit)** |
| 607 | `test_nothing_queued_flushes_to_nothing` | web_dashboard/test_batching.py:42 | Nothing queued flushes to nothing | A2 (shape/None only) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 608 | `test_repeated_values_of_one_type_collapse_to_the_newest` | web_dashboard/test_batching.py:46 | Repeated values of one type collapse to the newest | A3 (unsourced number) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 609 | `test_different_types_all_survive_one_flush` | web_dashboard/test_batching.py:58 | Different types all survive one flush | — | failed as expected (3/3 mutants) | **KEEP** |
| 610 | `test_the_batch_is_a_single_frame_with_a_batch_type` | web_dashboard/test_batching.py:69 | The batch is a single frame with a batch type | — | failed as expected (3/3 mutants) | **KEEP** |
| 611 | `test_flushing_empties_the_queue` | web_dashboard/test_batching.py:77 | Flushing empties the queue | A2 (shape/None only) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 612 | `test_clear_discards_without_sending` | web_dashboard/test_batching.py:84 | Clear discards without sending | A2 (shape/None only) | failed as expected (4/4 mutants) | **KEEP (nit)** |
| 613 | `test_a_non_dict_message_is_rejected_rather_than_queued` | web_dashboard/test_batching.py:91 | A non dict message is rejected rather than queued | — | failed as expected (2/2 mutants) | **KEEP** |
| 614 | `test_every_intent_state_transition_survives_coalescing` | web_dashboard/test_batching.py:100 | Every intent state transition survives coalescing | — | failed as expected (5/5 mutants) | **KEEP** |
| 615 | `test_a_single_tick_blip_is_not_swallowed` | web_dashboard/test_batching.py:109 | A single tick blip is not swallowed | — | failed as expected (5/5 mutants) | **KEEP** |
| 616 | `test_repeats_of_one_state_keep_the_newest_sample` | web_dashboard/test_batching.py:121 | Repeats of one state keep the newest sample | — | failed as expected (5/5 mutants) | **KEEP** |
| 617 | `test_transitions_are_delivered_in_the_order_they_happened` | web_dashboard/test_batching.py:133 | Transitions are delivered in the order they happened | — | failed as expected (5/5 mutants) | **KEEP** |
| 618 | `test_a_state_repeated_after_flushing_is_still_sent` | web_dashboard/test_batching.py:141 | A state repeated after flushing is still sent | — | failed as expected (5/5 mutants) | **KEEP** |
| 619 | `test_intents_are_applied_after_the_pose_they_are_drawn_against` | web_dashboard/test_batching.py:151 | Intents are applied after the pose they are drawn against | — | failed as expected (5/5 mutants) | **KEEP** |
| 620 | `test_an_intent_without_a_recognisable_state_still_batches` | web_dashboard/test_batching.py:161 | An intent without a recognisable state still batches | A2 (shape/None only); A5 (lone `is not None`); cleared: weak `is not None`; mutant check inconclusive | failed as expected (5/5 mutants) | **KEEP (nit)** |
| 621 | `test_unflushed_intents_are_capped_rather_than_growing_without_bound` | web_dashboard/test_batching.py:172 | Unflushed intents are capped rather than growing without bound | — | failed as expected (5/5 mutants) | **KEEP** |
| 622 | `test_dropping_is_counted_so_it_can_be_noticed_instead_of_hidden` | web_dashboard/test_batching.py:184 | Dropping is counted so it can be noticed instead of hidden | — | failed as expected (5/5 mutants) | **KEEP** |
| 623 | `test_the_cap_is_never_reached_at_realistic_rates` | web_dashboard/test_batching.py:192 | The cap is never reached at realistic rates | — | failed as expected (6/6 mutants) | **KEEP** |
| 624 | `test_len_reports_everything_waiting` | web_dashboard/test_batching.py:202 | Len reports everything waiting | — | failed as expected (5/5 mutants) | **KEEP** |
| 625 | `test_browser_map_decoding` | web_dashboard/test_dashboard_js.py:29 | Browser map decoding | — | no mutant reached it | **KEEP (unverified)** |
| 626 | `test_browser_draw_frames` | web_dashboard/test_draw_frames_js.py:35 | Browser draw frames | — | no mutant reached it | **KEEP (unverified)** |
| 627 | `test_intent_message_wraps_the_payload_without_rewriting_it` | web_dashboard/test_intent_protocol.py:51 | A pass-through on purpose: the schema is owned by | — | failed as expected (9/9 mutants) | **KEEP** |
| 628 | `test_intent_message_carries_a_server_stamp_alongside_the_cars` | web_dashboard/test_intent_protocol.py:61 | Two clocks, two fields. A laptop whose clock disagrees with the | A5 (bare approx, no tolerance) | failed as expected (9/9 mutants) | **KEEP (nit)** |
| 629 | `test_the_envelope_survives_json_serialization` | web_dashboard/test_intent_protocol.py:70 | It goes out over a WebSocket as JSON text; anything json.dumps | — | failed as expected (9/9 mutants) | **KEEP** |
| 630 | `test_a_well_formed_message_is_accepted` | web_dashboard/test_intent_protocol.py:81 | A well formed message is accepted | A2 (shape/None only) | failed as expected (12/12 mutants) | **KEEP (nit)** |
| 631 | `test_undecodable_data_raises_valueerror_rather_than_something_exotic` | web_dashboard/test_intent_protocol.py:94 | Intent_callback catches ValueError specifically; anything else | — | failed as expected (1/1 mutants) | **KEEP** |
| 632 | `test_a_payload_from_a_newer_schema_is_refused_not_half_drawn` | web_dashboard/test_intent_protocol.py:101 | A payload from a newer schema is refused not half drawn | A2 (shape/None only); A5 (lone `is not None`) | FALSELY PASSED on `return <const>` (stub killed it; sibling catches it) | **REWRITE** |
| 633 | `test_a_missing_severity_is_refused` | web_dashboard/test_intent_protocol.py:105 | A missing severity is refused | A2 (shape/None only); A5 (lone `is not None`) | FALSELY PASSED on `return <const>` (stub killed it; sibling catches it) | **REWRITE** |
| 634 | `test_a_path_of_the_wrong_shape_is_refused` | web_dashboard/test_intent_protocol.py:111 | The most likely mistake in a hand-rolled C++ JSON writer: emitting | A2 (shape/None only); A5 (lone `is not None`) | FALSELY PASSED on `return <const>` (stub killed it; sibling catches it) | **REWRITE** |
| 635 | `test_a_nan_from_a_printf_style_publisher_is_refused` | web_dashboard/test_intent_protocol.py:117 | C's printf("%f", nan) writes `nan`; a slightly better writer emits | A2 (shape/None only); A5 (lone `is not None`) | FALSELY PASSED on `return <const>` (stub killed it; sibling catches it) | **REWRITE** |
| 636 | `test_an_oversized_path_is_refused_before_it_reaches_a_phone` | web_dashboard/test_intent_protocol.py:126 | An oversized path is refused before it reaches a phone | A2 (shape/None only); A5 (lone `is not None`) | FALSELY PASSED on `return <const>` (stub killed it; sibling catches it) | **REWRITE** |
| 637 | `test_the_reason_may_legitimately_be_absent` | web_dashboard/test_intent_protocol.py:132 | The car only re-sends its explanation on state changes and on a | A2 (shape/None only) | failed as expected (10/10 mutants) | **KEEP (nit)** |
| 638 | `test_validation_does_not_depend_on_optional_geometry` | web_dashboard/test_intent_protocol.py:141 | Validation does not depend on optional geometry | A2 (shape/None only) | failed as expected (10/10 mutants) | **KEEP (nit)** |
| 639 | `test_first_grid_is_a_keyframe_carrying_the_whole_map` | web_dashboard/test_mapstream.py:92 | First grid is a keyframe carrying the whole map | — | failed as expected (6/6 mutants) | **KEEP** |
| 640 | `test_keyframe_carries_the_geometry_the_browser_needs_to_place_it` | web_dashboard/test_mapstream.py:102 | Keyframe carries the geometry the browser needs to place it | A5 (bare approx, no tolerance) | failed as expected (6/6 mutants) | **KEEP (nit)** |
| 641 | `test_an_unchanged_grid_sends_nothing_at_all` | web_dashboard/test_mapstream.py:111 | An unchanged grid sends nothing at all | A2 (shape/None only) | failed as expected (8/8 mutants) | **KEEP (nit)** |
| 642 | `test_growing_the_grid_forces_a_keyframe_not_a_patch` | web_dashboard/test_mapstream.py:121 | Growing the grid forces a keyframe not a patch | — | failed as expected (8/8 mutants) | **KEEP** |
| 643 | `test_moving_the_origin_forces_a_keyframe` | web_dashboard/test_mapstream.py:133 | Moving the origin forces a keyframe | — | failed as expected (8/8 mutants) | **KEEP** |
| 644 | `test_a_keyframe_is_resent_periodically_so_a_client_cannot_be_wrong_forever` | web_dashboard/test_mapstream.py:141 | A keyframe is resent periodically so a client cannot be wrong forever | — | failed as expected (10/10 mutants) | **KEEP** |
| 645 | `test_keyframe_sec_zero_disables_the_periodic_refresh` | web_dashboard/test_mapstream.py:153 | Keyframe sec zero disables the periodic refresh | — | failed as expected (10/10 mutants) | **KEEP** |
| 646 | `test_a_small_change_becomes_a_patch_of_just_that_rectangle` | web_dashboard/test_mapstream.py:166 | A small change becomes a patch of just that rectangle | — | failed as expected (10/10 mutants) | **KEEP** |
| 647 | `test_a_patch_is_dramatically_smaller_than_the_grid_it_updates` | web_dashboard/test_mapstream.py:177 | A patch is dramatically smaller than the grid it updates | — | failed as expected (10/10 mutants) | **KEEP** |
| 648 | `test_a_change_covering_most_of_the_map_falls_back_to_a_keyframe` | web_dashboard/test_mapstream.py:188 | A change covering most of the map falls back to a keyframe | — | failed as expected (9/9 mutants) | **KEEP** |
| 649 | `test_sequence_numbers_increase_by_one_per_frame` | web_dashboard/test_mapstream.py:199 | Sequence numbers increase by one per frame | — | failed as expected (10/10 mutants) | **KEEP** |
| 650 | `test_keyframe_then_patches_reconstruct_the_grid_exactly` | web_dashboard/test_mapstream.py:209 | The end-to-end proof: replay a mapping session and compare bytes | — | failed as expected (10/10 mutants) | **KEEP** |
| 651 | `test_a_patch_touching_the_last_row_and_column_is_placed_correctly` | web_dashboard/test_mapstream.py:237 | A patch touching the last row and column is placed correctly | — | failed as expected (10/10 mutants) | **KEEP** |
| 652 | `test_two_separate_changes_are_covered_by_one_bounding_box` | web_dashboard/test_mapstream.py:251 | Two separate changes are covered by one bounding box | — | failed as expected (10/10 mutants) | **KEEP** |
| 653 | `test_current_keyframe_reflects_patches_already_sent` | web_dashboard/test_mapstream.py:268 | A tab that connects mid-session must get the map as it is *now* | — | failed as expected (11/11 mutants) | **KEEP** |
| 654 | `test_a_late_joiner_is_in_sync_for_the_very_next_patch` | web_dashboard/test_mapstream.py:286 | A late joiner is in sync for the very next patch | — | failed as expected (11/11 mutants) | **KEEP** |
| 655 | `test_current_keyframe_is_none_before_any_map_arrives` | web_dashboard/test_mapstream.py:303 | Current keyframe is none before any map arrives | A2 (shape/None only) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 656 | `test_a_browser_that_misses_a_patch_refuses_to_paint_a_wrong_map` | web_dashboard/test_mapstream.py:307 | A browser that misses a patch refuses to paint a wrong map | — | failed as expected (11/11 mutants) | **KEEP** |
| 657 | `test_compression_is_used_when_it_helps_and_declared_in_the_header` | web_dashboard/test_mapstream.py:329 | Compression is used when it helps and declared in the header | — | failed as expected (6/6 mutants) | **KEEP** |
| 658 | `test_compression_can_be_turned_off_entirely` | web_dashboard/test_mapstream.py:337 | Compression can be turned off entirely | — | failed as expected (6/6 mutants) | **KEEP** |
| 659 | `test_a_payload_that_would_grow_is_sent_raw_instead` | web_dashboard/test_mapstream.py:344 | A payload that would grow is sent raw instead | — | failed as expected (1/1 mutants) | **KEEP** |
| 660 | `test_bytes_always_describes_the_frame_that_actually_follows` | web_dashboard/test_mapstream.py:352 | Bytes always describes the frame that actually follows | — | failed as expected (10/10 mutants) | **KEEP** |
| 661 | `test_a_grid_that_does_not_match_its_geometry_is_rejected_loudly` | web_dashboard/test_mapstream.py:366 | A grid that does not match its geometry is rejected loudly | — | failed as expected (3/3 mutants) | **KEEP** |
| 662 | `test_patching_can_be_disabled_leaving_only_keyframes` | web_dashboard/test_mapstream.py:374 | Patching can be disabled leaving only keyframes | — | failed as expected (8/8 mutants) | **KEEP** |
| 663 | `test_unknown_cells_survive_the_round_trip_as_minus_one` | web_dashboard/test_mapstream.py:383 | Unknown cells survive the round trip as minus one | — | failed as expected (6/6 mutants) | **KEEP** |
| 664 | `test_wildcard_hosts_mean_every_interface` | web_dashboard/test_netbind.py:23 | Wildcard hosts mean every interface | — | failed as expected (1/1 mutants) | **KEEP** |
| 665 | `test_a_real_address_is_left_alone` | web_dashboard/test_netbind.py:29 | Someone who names one address wants exactly that one -- deliberately | — | failed as expected (1/1 mutants) | **KEEP** |
| 666 | `test_the_startup_line_never_prints_an_unopenable_url` | web_dashboard/test_netbind.py:36 | 'Serving on http://0.0.0.0:8080/' is not an address anyone can | — | failed as expected (2/2 mutants) | **KEEP** |
| 667 | `test_a_wildcard_host_really_binds_both_ip_families` | web_dashboard/test_netbind.py:55 | The regression test for "unreachable over Tailscale" | — | no mutant reached it | **KEEP (unverified)** |
| 668 | `test_a_named_host_binds_only_that_host` | web_dashboard/test_netbind.py:86 | A named host binds only that host | — | no mutant reached it | **KEEP (unverified)** |
| 669 | `test_every_web_asset_on_disk_is_installed` | web_dashboard/test_packaging.py:65 | The blanket rule: if it is in web/, it ships | — | no mutant reached it | **KEEP (unverified)** |
| 670 | `test_every_script_and_stylesheet_a_page_requests_is_installed` | web_dashboard/test_packaging.py:79 | The tighter rule, and the one that actually broke: whatever the | — | no mutant reached it | **KEEP (unverified)** |
| 671 | `test_the_pages_themselves_are_installed` | web_dashboard/test_packaging.py:97 | The pages themselves are installed | — | no mutant reached it | **KEEP (unverified)** |
| 672 | `test_browser_panel_geometry` | web_dashboard/test_panels_js.py:28 | Browser panel geometry | — | no mutant reached it | **KEEP (unverified)** |
| 673 | `test_browser_proc_panel` | web_dashboard/test_proc_panel_js.py:33 | Browser proc panel | — | no mutant reached it | **KEEP (unverified)** |
| 674 | `test_classify_installed_node_executable` | web_dashboard/test_proccontrol.py:73 | Classify installed node executable | — | failed as expected (3/3 mutants) | **KEEP** |
| 675 | `test_classify_ros2_launch_names_the_launch_file` | web_dashboard/test_proccontrol.py:77 | Classify ros2 launch names the launch file | — | failed as expected (2/2 mutants) | **KEEP** |
| 676 | `test_classify_ros2_run_names_the_executable` | web_dashboard/test_proccontrol.py:81 | Classify ros2 run names the executable | — | failed as expected (2/2 mutants) | **KEEP** |
| 677 | `test_classify_prefers_an_explicit_node_remap` | web_dashboard/test_proccontrol.py:87 | A remapped node answers to the remapped name on the graph, so that | — | failed as expected (3/3 mutants) | **KEEP** |
| 678 | `test_classify_looks_past_a_python_interpreter` | web_dashboard/test_proccontrol.py:95 | Classify looks past a python interpreter | — | failed as expected (3/3 mutants) | **KEEP** |
| 679 | `test_classify_finds_an_extensionless_console_script` | web_dashboard/test_proccontrol.py:100 | Regression. An installed ament_python node is a shebang script | — | failed as expected (3/3 mutants) | **KEEP** |
| 680 | `test_classify_skips_interpreter_flags` | web_dashboard/test_proccontrol.py:109 | Classify skips interpreter flags | — | failed as expected (3/3 mutants) | **KEEP** |
| 681 | `test_classify_ignores_junk` | web_dashboard/test_proccontrol.py:115 | Classify ignores junk | — | failed as expected (3/3 mutants) | **KEEP** |
| 682 | `test_actuation_path_is_protected_by_name` | web_dashboard/test_proccontrol.py:128 | Actuation path is protected by name | — | failed as expected (1/1 mutants) | **KEEP** |
| 683 | `test_protection_survives_a_node_suffix_mismatch` | web_dashboard/test_proccontrol.py:132 | `ackermann_mux` is the package; the executable is | — | failed as expected (2/2 mutants) | **KEEP** |
| 684 | `test_driving_nodes_are_not_protected` | web_dashboard/test_proccontrol.py:138 | Driving nodes are not protected | — | failed as expected (1/1 mutants) | **KEEP** |
| 685 | `test_sanitize_allowlist_strips_protected_entries` | web_dashboard/test_proccontrol.py:143 | Sanitize allowlist strips protected entries | — | failed as expected (2/2 mutants) | **KEEP** |
| 686 | `test_sanitize_allowlist_warns_about_refusals` | web_dashboard/test_proccontrol.py:149 | Sanitize allowlist warns about refusals | — | failed as expected (2/2 mutants) | **KEEP** |
| 687 | `test_sanitize_allowlist_dedupes_and_drops_blanks` | web_dashboard/test_proccontrol.py:161 | Sanitize allowlist dedupes and drops blanks | — | failed as expected (2/2 mutants) | **KEEP** |
| 688 | `test_scan_finds_a_driving_node_and_marks_it_stoppable` | web_dashboard/test_proccontrol.py:171 | Scan finds a driving node and marks it stoppable | — | failed as expected (10/10 mutants) | **KEEP** |
| 689 | `test_scan_reports_the_actuation_path_but_refuses_it` | web_dashboard/test_proccontrol.py:180 | Scan reports the actuation path but refuses it | — | failed as expected (10/10 mutants) | **KEEP** |
| 690 | `test_scan_refuses_bringup_launch_even_though_it_is_a_launch` | web_dashboard/test_proccontrol.py:189 | Scan refuses bringup launch even though it is a launch | — | failed as expected (9/9 mutants) | **KEEP** |
| 691 | `test_scan_ignores_processes_that_are_not_ours` | web_dashboard/test_proccontrol.py:197 | Scan ignores processes that are not ours | — | failed as expected (10/10 mutants) | **KEEP** |
| 692 | `test_scan_will_not_offer_to_kill_the_dashboard_or_its_parents` | web_dashboard/test_proccontrol.py:207 | Pid 100 is the dashboard's own ancestor chain. Even though the | — | failed as expected (10/10 mutants) | **KEEP** |
| 693 | `test_scan_refuses_processes_owned_by_another_user` | web_dashboard/test_proccontrol.py:220 | Scan refuses processes owned by another user | — | failed as expected (10/10 mutants) | **KEEP** |
| 694 | `test_scan_honours_a_custom_allowlist` | web_dashboard/test_proccontrol.py:227 | Scan honours a custom allowlist | — | failed as expected (8/8 mutants) | **KEEP** |
| 695 | `test_scan_survives_a_missing_proc_root` | web_dashboard/test_proccontrol.py:234 | Scan survives a missing proc root | — | failed as expected (3/3 mutants) | **KEEP** |
| 696 | `test_scan_skips_kernel_threads_with_empty_cmdlines` | web_dashboard/test_proccontrol.py:238 | Scan skips kernel threads with empty cmdlines | — | failed as expected (10/10 mutants) | **KEEP** |
| 697 | `test_stop_job_starts_with_sigint_like_ctrl_c` | web_dashboard/test_proccontrol.py:266 | Stop job starts with sigint like ctrl c | — | failed as expected (5/5 mutants) | **KEEP** |
| 698 | `test_stop_job_reports_success_once_the_process_is_gone` | web_dashboard/test_proccontrol.py:274 | Stop job reports success once the process is gone | — | failed as expected (5/5 mutants) | **KEEP** |
| 699 | `test_stop_job_escalates_when_ctrl_c_is_ignored` | web_dashboard/test_proccontrol.py:284 | The reported bug: SIGINT does nothing. Escalate rather than sulk | — | failed as expected (5/5 mutants) | **KEEP** |
| 700 | `test_stop_job_waits_out_the_grace_period_before_escalating` | web_dashboard/test_proccontrol.py:294 | Stop job waits out the grace period before escalating | — | failed as expected (5/5 mutants) | **KEEP** |
| 701 | `test_stop_job_gives_up_after_sigkill_with_an_actionable_message` | web_dashboard/test_proccontrol.py:303 | Stop job gives up after sigkill with an actionable message | — | failed as expected (5/5 mutants) | **KEEP** |
| 702 | `test_stop_job_on_an_already_dead_pid_is_a_success_not_an_error` | web_dashboard/test_proccontrol.py:313 | Stop job on an already dead pid is a success not an error | — | failed as expected (5/5 mutants) | **KEEP** |
| 703 | `test_stop_job_surfaces_a_permission_error` | web_dashboard/test_proccontrol.py:323 | Stop job surfaces a permission error | — | failed as expected (5/5 mutants) | **KEEP** |
| 704 | `test_stop_job_serialises_for_the_wire` | web_dashboard/test_proccontrol.py:336 | Stop job serialises for the wire | — | failed as expected (6/6 mutants) | **KEEP** |
| 705 | `test_alive_reports_false_for_a_missing_pid` | web_dashboard/test_proccontrol.py:346 | Alive reports false for a missing pid | — | failed as expected (3/3 mutants) | **KEEP** |
| 706 | `test_alive_reports_true_when_signalling_is_forbidden` | web_dashboard/test_proccontrol.py:353 | Alive reports true when signalling is forbidden | — | failed as expected (3/3 mutants) | **KEEP** |
| 707 | `test_a_zombie_is_not_alive` | web_dashboard/test_proccontrol.py:374 | A zombie is not alive | — | failed as expected (3/3 mutants) | **KEEP** |
| 708 | `test_a_running_process_is_alive` | web_dashboard/test_proccontrol.py:383 | A running process is alive | — | failed as expected (3/3 mutants) | **KEEP** |
| 709 | `test_stop_job_calls_a_zombie_stopped_rather_than_unkillable` | web_dashboard/test_proccontrol.py:388 | The exact end-to-end failure: SIGKILL lands, the process becomes a | — | failed as expected (5/5 mutants) | **KEEP** |
| 710 | `test_scan_skips_zombies_entirely` | web_dashboard/test_proccontrol.py:414 | A zombie is not running anything, so it must not be listed as a | — | failed as expected (4/4 mutants) | **KEEP** |
| 711 | `test_process_state_message_carries_the_refusals_too` | web_dashboard/test_proccontrol.py:431 | The browser needs the protected rows, not just the stoppable ones: | — | failed as expected (12/12 mutants) | **KEEP** |
| 712 | `test_process_state_message_is_json_serialisable` | web_dashboard/test_proccontrol.py:447 | It goes down a WebSocket as JSON -- a Target object that survived | A5 (no assertion) | FALSELY PASSED on `return <const>` (stub killed it; sibling catches it) | **REWRITE** |
| 713 | `test_process_result_message_reports_the_escalation` | web_dashboard/test_proccontrol.py:455 | Process result message reports the escalation | — | failed as expected (1/1 mutants) | **KEEP** |
| 714 | `test_a_disabled_dashboard_still_sends_an_explicit_empty_state` | web_dashboard/test_proccontrol.py:464 | So the browser hides the panel outright, rather than showing an | — | failed as expected (1/1 mutants) | **KEEP** |
| 715 | `test_quaternion_to_yaw_matches_known_angle` | web_dashboard/test_protocol.py:26 | Quaternion to yaw matches known angle | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 716 | `test_map_header_carries_correct_metadata` | web_dashboard/test_protocol.py:45 | Map header carries correct metadata | A3 (unsourced number); A5 (bare approx, no tolerance) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 717 | `test_map_cells_round_trips_unknown_free_and_occupied` | web_dashboard/test_protocol.py:56 | Map cells round trips unknown free and occupied | — | failed as expected (1/1 mutants) | **KEEP** |
| 718 | `test_scan_header_carries_laser_offset_and_geometry` | web_dashboard/test_protocol.py:72 | Scan header carries laser offset and geometry | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 719 | `test_scan_ranges_round_trips_as_float32` | web_dashboard/test_protocol.py:81 | Scan ranges round trips as float32 | A5 (assert may never run); cleared: loop over fixed inputs, both branches assert | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 720 | `test_pose_message_shape` | web_dashboard/test_protocol.py:93 | Pose message shape | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 721 | `test_drive_message_shape` | web_dashboard/test_protocol.py:102 | Drive message shape | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 722 | `test_speed_message_shape` | web_dashboard/test_protocol.py:110 | Speed message shape | A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 723 | `test_stopwatch_message_shape` | web_dashboard/test_protocol.py:117 | Stopwatch message shape | A3 (unsourced number); A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 724 | `test_stats_message_shape_with_temp_and_wifi` | web_dashboard/test_protocol.py:136 | Stats message shape with temp and wifi | A3 (unsourced number); A5 (bare approx, no tolerance) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 725 | `test_stats_message_allows_missing_temp_and_wifi` | web_dashboard/test_protocol.py:147 | Stats message allows missing temp and wifi | A2 (shape/None only) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 726 | `test_map_header_declares_its_payload_length` | web_dashboard/test_protocol.py:157 | The browser holds one "what does the next binary mean" slot, so a | — | failed as expected (3/3 mutants) | **KEEP** |
| 727 | `test_scan_header_declares_its_payload_length` | web_dashboard/test_protocol.py:172 | Scan header declares its payload length | — | failed as expected (2/2 mutants) | **KEEP** |
| 728 | `test_a_scan_payload_is_not_the_length_of_a_map_payload` | web_dashboard/test_protocol.py:179 | The two are wildly different sizes, which is what makes a length | — | failed as expected (5/5 mutants) | **KEEP** |
| 729 | `test_scan_header_still_defaults_to_float32` | web_dashboard/test_protocol_encoding.py:36 | Scan header still defaults to float32 | — | failed as expected (1/1 mutants) | **KEEP** |
| 730 | `test_u16mm_header_declares_two_bytes_per_beam` | web_dashboard/test_protocol_encoding.py:43 | U16mm header declares two bytes per beam | — | failed as expected (3/3 mutants) | **KEEP** |
| 731 | `test_u16mm_round_trips_to_millimetre_accuracy` | web_dashboard/test_protocol_encoding.py:50 | U16mm round trips to millimetre accuracy | — | failed as expected (1/1 mutants) | **KEEP** |
| 732 | `test_u16mm_maps_every_unusable_reading_to_zero` | web_dashboard/test_protocol_encoding.py:58 | U16mm maps every unusable reading to zero | — | failed as expected (1/1 mutants) | **KEEP** |
| 733 | `test_u16mm_is_exactly_half_the_size_of_float32` | web_dashboard/test_protocol_encoding.py:68 | U16mm is exactly half the size of float32 | — | failed as expected (2/2 mutants) | **KEEP** |
| 734 | `test_u16mm_holds_the_full_range_of_this_car_s_lidar` | web_dashboard/test_protocol_encoding.py:73 | U16mm holds the full range of this car s lidar | — | failed as expected (1/1 mutants) | **KEEP** |
| 735 | `test_decimation_thins_the_beams_and_widens_the_angle_step` | web_dashboard/test_protocol_encoding.py:80 | Decimation thins the beams and widens the angle step | A5 (bare approx, no tolerance) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 736 | `test_decimation_keeps_the_first_beam_so_angle_min_stays_true` | web_dashboard/test_protocol_encoding.py:92 | Decimation keeps the first beam so angle min stays true | — | failed as expected (1/1 mutants) | **KEEP** |
| 737 | `test_decimation_also_applies_to_the_float32_encoding` | web_dashboard/test_protocol_encoding.py:99 | Decimation also applies to the float32 encoding | A3 (unsourced number) | failed as expected (4/4 mutants) | **KEEP (nit)** |
| 738 | `test_decimation_of_one_changes_nothing` | web_dashboard/test_protocol_encoding.py:107 | Decimation of one changes nothing | A5 (bare approx, no tolerance) | failed as expected (3/3 mutants) | **KEEP (nit)** |
| 739 | `test_odd_counts_decimate_without_losing_the_tail` | web_dashboard/test_protocol_encoding.py:114 | Odd counts decimate without losing the tail | — | failed as expected (3/3 mutants) | **KEEP** |
| 740 | `test_the_numpy_and_pure_python_quantisers_agree` | web_dashboard/test_protocol_encoding.py:123 | The numpy and pure python quantisers agree | — | failed as expected (1/1 mutants) | **KEEP** |
| 741 | `test_an_identical_commanded_path_is_dropped` | web_dashboard/test_protocol_encoding.py:147 | An identical commanded path is dropped | — | failed as expected (1/1 mutants) | **KEEP** |
| 742 | `test_a_meaningfully_different_commanded_path_is_kept` | web_dashboard/test_protocol_encoding.py:156 | A meaningfully different commanded path is kept | — | failed as expected (1/1 mutants) | **KEEP** |
| 743 | `test_a_difference_far_below_a_pixel_counts_as_identical` | web_dashboard/test_protocol_encoding.py:166 | A difference far below a pixel counts as identical | — | failed as expected (1/1 mutants) | **KEEP** |
| 744 | `test_a_difference_in_only_the_x_axis_is_still_a_difference` | web_dashboard/test_protocol_encoding.py:174 | A difference in only the x axis is still a difference | — | failed as expected (1/1 mutants) | **KEEP** |
| 745 | `test_paths_of_different_lengths_are_left_alone` | web_dashboard/test_protocol_encoding.py:182 | Paths of different lengths are left alone | — | failed as expected (1/1 mutants) | **KEEP** |
| 746 | `test_thinning_never_mutates_the_callers_payload` | web_dashboard/test_protocol_encoding.py:190 | Thinning never mutates the callers payload | — | failed as expected (1/1 mutants) | **KEEP** |
| 747 | `test_thinning_tolerates_a_missing_or_empty_path` | web_dashboard/test_protocol_encoding.py:199 | Thinning tolerates a missing or empty path | A2 (shape/None only); A5 (lone `is not None`) | FALSELY PASSED on `return <const>` (stub killed it; sibling catches it) | **REWRITE** |
| 748 | `test_thinning_leaves_every_other_field_untouched` | web_dashboard/test_protocol_encoding.py:205 | Thinning leaves every other field untouched | A3 (unsourced number) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 749 | `test_the_line_is_passed_through_unchanged` | web_dashboard/test_racing_line_protocol.py:38 | Rounding and decimation happen where the waypoint count is known -- | A3 (unsourced number) | failed as expected (1/1 mutants) | **KEEP (nit)** |
| 750 | `test_the_message_is_json_serializable` | web_dashboard/test_racing_line_protocol.py:48 | The message is json serializable | — | failed as expected (1/1 mutants) | **KEEP** |
| 751 | `test_none_clears_the_line` | web_dashboard/test_racing_line_protocol.py:54 | A controller shutting down should take its line off the map, not | — | failed as expected (1/1 mutants) | **KEEP** |
| 752 | `test_the_message_carries_what_the_readout_needs` | web_dashboard/test_racing_line_protocol.py:62 | The message carries what the readout needs | — | failed as expected (1/1 mutants) | **KEEP** |
| 753 | `test_a_stamp_is_added` | web_dashboard/test_racing_line_protocol.py:69 | A stamp is added | — | failed as expected (1/1 mutants) | **KEEP** |
| 754 | `test_disabled_stopwatch_does_not_run_with_lb_held` | web_dashboard/test_stopwatch.py:12 | Disabled stopwatch does not run with lb held | A3 (unsourced number); A5 (bare approx, no tolerance) | failed as expected (4/4 mutants) | **KEEP (nit)** |
| 755 | `test_enabled_stopwatch_runs_only_while_lb_is_held` | web_dashboard/test_stopwatch.py:18 | Enabled stopwatch runs only while lb is held | A3 (unsourced number); A5 (bare approx, no tolerance) | failed as expected (5/5 mutants) | **KEEP (nit)** |
| 756 | `test_stale_joy_stops_at_watchdog_deadline` | web_dashboard/test_stopwatch.py:28 | Stale joy stops at watchdog deadline | A3 (unsourced number); A5 (bare approx, no tolerance) | failed as expected (5/5 mutants) | **KEEP (nit)** |
| 757 | `test_missing_lb_button_never_runs` | web_dashboard/test_stopwatch.py:39 | Missing lb button never runs | A3 (unsourced number); A5 (bare approx, no tolerance) | failed as expected (5/5 mutants) | **KEEP (nit)** |
| 758 | `test_reset_clears_elapsed_but_keeps_enabled_state` | web_dashboard/test_stopwatch.py:48 | Reset clears elapsed but keeps enabled state | A3 (unsourced number); A5 (bare approx, no tolerance) | failed as expected (6/6 mutants) | **KEEP (nit)** |
| 759 | `test_parse_spec_reads_a_well_formed_catalogue` | web_dashboard/test_tuning.py:55 | Parse spec reads a well formed catalogue | — | failed as expected (1/1 mutants) | **KEEP** |
| 760 | `test_parse_spec_rejects_an_unsupported_version` | web_dashboard/test_tuning.py:63 | Parse spec rejects an unsupported version | — | failed as expected (1/1 mutants) | **KEEP** |
| 761 | `test_parse_spec_survives_junk` | web_dashboard/test_tuning.py:71 | A node mid-restart, a different version, or the wrong node entirely | — | failed as expected (1/1 mutants) | **KEEP** |
| 762 | `test_parse_spec_drops_unusable_entries_but_keeps_the_rest` | web_dashboard/test_tuning.py:79 | Parse spec drops unusable entries but keeps the rest | — | failed as expected (1/1 mutants) | **KEEP** |
| 763 | `test_coerce_request_clamps_rather_than_refusing` | web_dashboard/test_tuning.py:95 | Coerce request clamps rather than refusing | — | failed as expected (2/2 mutants) | **KEEP** |
| 764 | `test_coerce_request_refuses_wrong_types` | web_dashboard/test_tuning.py:102 | Coerce request refuses wrong types | A2 (shape/None only) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 765 | `test_coerce_request_refuses_non_finite` | web_dashboard/test_tuning.py:109 | Coerce request refuses non finite | A2 (shape/None only) | failed as expected (2/2 mutants) | **KEEP (nit)** |
| 766 | `test_coerce_request_bool_needs_a_real_bool` | web_dashboard/test_tuning.py:116 | Coerce request bool needs a real bool | — | failed as expected (2/2 mutants) | **KEEP** |
| 767 | `test_format_scalar` | web_dashboard/test_tuning.py:130 | Format scalar | — | failed as expected (1/1 mutants) | **KEEP** |
| 768 | `test_format_scalar_keeps_floats_floating` | web_dashboard/test_tuning.py:134 | A whole-number double written as bare `4` reloads as an int and the | — | failed as expected (1/1 mutants) | **KEEP** |
| 769 | `test_update_yaml_rewrites_only_the_requested_values` | web_dashboard/test_tuning.py:145 | Update yaml rewrites only the requested values | — | failed as expected (4/4 mutants) | **KEEP** |
| 770 | `test_update_yaml_preserves_comments` | web_dashboard/test_tuning.py:156 | Update yaml preserves comments | — | failed as expected (4/4 mutants) | **KEEP** |
| 771 | `test_update_yaml_appends_a_missing_key` | web_dashboard/test_tuning.py:163 | Update yaml appends a missing key | — | failed as expected (4/4 mutants) | **KEEP** |
| 772 | `test_update_yaml_handles_bools` | web_dashboard/test_tuning.py:170 | Update yaml handles bools | — | failed as expected (4/4 mutants) | **KEEP** |
| 773 | `test_update_yaml_rejects_an_unknown_node` | web_dashboard/test_tuning.py:176 | Update yaml rejects an unknown node | — | failed as expected (1/1 mutants) | **KEEP** |
| 774 | `test_update_yaml_rejects_a_file_without_ros_parameters` | web_dashboard/test_tuning.py:181 | Update yaml rejects a file without ros parameters | — | failed as expected (3/3 mutants) | **KEEP** |
| 775 | `test_update_yaml_round_trips_the_real_configs` | web_dashboard/test_tuning.py:191 | The real files, which are mostly comments explaining the numbers | — | failed as expected (4/4 mutants) | **KEEP** |
| 776 | `test_values_needing_save_only_reports_real_differences` | web_dashboard/test_tuning.py:218 | Values needing save only reports real differences | — | failed as expected (1/1 mutants) | **KEEP** |
| 777 | `test_values_needing_save_includes_keys_absent_from_the_file` | web_dashboard/test_tuning.py:225 | Values needing save includes keys absent from the file | — | failed as expected (1/1 mutants) | **KEEP** |
| 778 | `test_values_needing_save_falls_back_to_everything_on_an_unreadable_file` | web_dashboard/test_tuning.py:231 | Better to write the whole tune than to silently save none of it | — | failed as expected (1/1 mutants) | **KEEP** |
| 779 | `test_values_needing_save_treats_bools_as_bools` | web_dashboard/test_tuning.py:237 | Values needing save treats bools as bools | — | failed as expected (1/1 mutants) | **KEEP** |
| 780 | `test_every_element_the_script_looks_up_exists_in_its_page` | web_dashboard/test_web_assets.py:58 | Every element the script looks up exists in its page | — | no mutant reached it | **KEEP (unverified)** |
| 781 | `test_no_duplicate_ids` | web_dashboard/test_web_assets.py:68 | No duplicate ids | — | no mutant reached it | **KEEP (unverified)** |
| 782 | `test_the_dashboard_still_loads_its_stylesheet_and_script` | web_dashboard/test_web_assets.py:74 | The dashboard still loads its stylesheet and script | — | no mutant reached it | **KEEP (unverified)** |
| 783 | `test_the_panel_manager_loads_after_the_dashboard` | web_dashboard/test_web_assets.py:81 | Panels.js MOVES section elements out of the sidebar. dashboard.js | — | no mutant reached it | **KEEP (unverified)** |
| 784 | `test_every_section_is_a_details_element` | web_dashboard/test_web_assets.py:93 | Collapsing is the browser's job here, not JavaScript's -- that is | — | no mutant reached it | **KEEP (unverified)** |
| 785 | `test_every_section_has_a_digest_element_for_its_collapsed_headline` | web_dashboard/test_web_assets.py:102 | Every section has a digest element for its collapsed headline | — | no mutant reached it | **KEEP (unverified)** |
| 786 | `test_the_javascript_knows_about_exactly_the_sections_that_exist` | web_dashboard/test_web_assets.py:112 | The javascript knows about exactly the sections that exist | — | no mutant reached it | **KEEP (unverified)** |
| 787 | `test_a_checkbox_never_sits_inside_a_summary` | web_dashboard/test_web_assets.py:123 | Clicking a control inside <summary> also toggles the section, which | — | no mutant reached it | **KEEP (unverified)** |
| 788 | `test_the_sidebar_can_receive_pointer_events` | web_dashboard/test_web_assets.py:135 | The decision log had overflow-y:auto and was still impossible to | — | no mutant reached it | **KEEP (unverified)** |
| 789 | `test_the_sidebar_is_bounded_and_scrolls` | web_dashboard/test_web_assets.py:144 | Unbounded, its bottom (the decision log, the tuning button, reset | — | no mutant reached it | **KEEP (unverified)** |
| 790 | `test_scrollable_regions_are_visibly_scrollable` | web_dashboard/test_web_assets.py:158 | Scrollable regions are visibly scrollable | — | no mutant reached it | **KEEP (unverified)** |
| 791 | `test_the_decision_log_scrolls` | web_dashboard/test_web_assets.py:164 | The decision log scrolls | — | no mutant reached it | **KEEP (unverified)** |
| 792 | `test_the_view_controls_are_pinned_outside_the_scroll_region` | web_dashboard/test_web_assets.py:174 | The banner explaining why the picture looks odd, and the reset-view | — | no mutant reached it | **KEEP (unverified)** |
| 793 | `test_the_page_is_well_formed` | web_dashboard/test_web_assets.py:187 | A stray unclosed tag reflows the whole sidebar in ways that are | — | no mutant reached it | **KEEP (unverified)** |
| 794 | `test_the_map_handles_pointer_events_not_just_mouse_events` | web_dashboard/test_web_assets.py:236 | The map handles pointer events not just mouse events | — | no mutant reached it | **KEEP (unverified)** |
| 795 | `test_the_canvas_claims_touch_gestures_from_the_browser` | web_dashboard/test_web_assets.py:247 | Without touch-action:none the browser scrolls and page-zooms first | — | no mutant reached it | **KEEP (unverified)** |
| 796 | `test_pinch_zoom_and_the_wheel_share_one_zoom_implementation` | web_dashboard/test_web_assets.py:253 | Two copies of the world-frame/body-frame branch is how they drift | — | no mutant reached it | **KEEP (unverified)** |
| 797 | `test_the_phone_breakpoint_agrees_between_the_stylesheet_and_the_script` | web_dashboard/test_web_assets.py:278 | The phone breakpoint agrees between the stylesheet and the script | — | no mutant reached it | **KEEP (unverified)** |
| 798 | `test_the_half_detent_agrees_between_the_stylesheet_and_the_script` | web_dashboard/test_web_assets.py:285 | The half detent agrees between the stylesheet and the script | — | no mutant reached it | **KEEP (unverified)** |
| 799 | `test_the_sheet_moves_by_transform_only` | web_dashboard/test_web_assets.py:293 | Rule 2 of the stylesheet, and it bites hardest here: a sheet that | — | no mutant reached it | **KEEP (unverified)** |
| 800 | `test_the_peek_height_is_a_variable_both_sides_can_read` | web_dashboard/test_web_assets.py:311 | Panels.js reads --sheet-peek off the computed style to decide which | — | no mutant reached it | **KEEP (unverified)** |
| 801 | `test_the_phone_strip_is_filled_by_the_dashboard` | web_dashboard/test_web_assets.py:319 | It is display:none on a laptop, so nothing else would notice it | — | no mutant reached it | **KEEP (unverified)** |
| 802 | `test_every_font_size_goes_through_the_type_scale` | web_dashboard/test_web_assets.py:332 | The whole responsive strategy is eight variables that the media | — | no mutant reached it | **KEEP (unverified)** |
| 803 | `test_the_phone_breakpoint_rescales_every_size_in_the_scale` | web_dashboard/test_web_assets.py:344 | Adding a token to the scale and forgetting to re-state it at the | — | no mutant reached it | **KEEP (unverified)** |
| 804 | `test_the_stylesheet_has_balanced_braces` | web_dashboard/test_web_assets.py:361 | The stylesheet has balanced braces | — | no mutant reached it | **KEEP (unverified)** |
