# RL Residual Training — Overnight Build Plan

Driven by an overnight `/loop` (fires every 5 min, same session). This file
is the cross-firing progress tracker — each firing should read it before
doing anything, and update it before finishing.

## Critical finding (2026-09-25, firing 1)

Checked every recorded bag (`bags/*/metadata.yaml`, 32 bags total):
- `/track_angles` is declared in **every** bag (so the topic existed in the
  ROS graph) but has **zero messages in all 32 of them**.
- `/camera/image_raw` doesn't appear in **any** bag at all.

Conclusion: no historical bag has any camera-derived data. The camera/CV
pipeline (SafetyChecker etc.) is new work as of this conversation — these
bags predate it. **Training the RL residual from historical bags is not
possible today; there is nothing to fix in the recording config itself**
(`ros2 bag record -a` in `scripts/start.bash` already records everything
dynamically — no topic allowlist to edit). The only real fix is making sure
`camera_node`/`opencv_pathfinder_node` are actually alive during *future*
recording sessions, and catching it immediately if they aren't (see Phase 1B
below) instead of discovering it four months later like this.

This does **not** block RL residual work broadly — shadow-mode training
(Phase 2) learns live, tick by tick, regardless of bag history. Only the
"pretrain from past drives" path (Phase 4) is currently empty.

## Phases

- [x] **Phase 1A** — this doc, plus survey of what data/infra already exists
      (`sim/load_bags.py` already does bag→aligned-array loading for the
      plant-training pipeline; `mpc/status`'s 88-float payload in `mpc.py`
      already carries `d` (idx 4), `psi_track` (idx 5), `cost` (idx 10),
      `margin_min` (idx 11) — a ready-made reward signal, no need to invent
      one).
- [x] **Phase 1B** — bag health-check script so a future recording session
      immediately flags "camera data is empty," instead of it going
      unnoticed. `scripts/bag_health_check.py`.
- [x] **Phase 1C** — extend `sim/load_bags.py` (additively, backward
      compatible with existing cached `.npz` files — see "Compatibility"
      below) to optionally capture `/track_angles` and `/mpc/status`,
      NaN-filled when absent (true for every bag today). This is the
      loader Phase 4 will use once real bags exist.
- [x] **Phase 1D** — scaffold `nodes/pathfinder/planners/rl_residual/`
      (state/action spec + a minimal linear-Gaussian policy-gradient
      learner, `mode: off | shadow | apply` gated the same way as the
      existing `ResidualLearner`, **defaults to `off`** — no live behavior
      change from this phase alone). Unit tests only, not wired into
      `MPCPlanner`/`pathfinder_node` yet.

## Reward-validity finding (firing 2) — changes Phase 2/3's scope

Traced `MPCPlanner.plan()` (mpc.py) fully. Two things that change the plan:

1. **Even the existing `ResidualLearner`'s `apply` mode doesn't close the
   loop.** `res_ps, res_pd = self.residual.predict(phi)` is computed, but
   `return throttle_mps, steering_deg` (line 424) never uses them — they
   only reach `mpc/status` telemetry. So there is currently no path in this
   codebase, for either residual system, where a residual correction
   actually changes what the kart does.
2. **That makes live REINFORCE training mathematically invalid right now.**
   Policy-gradient needs the sampled action to have actually influenced the
   outcome being rewarded. If nothing blends the RL residual's action into
   `cmd_drive`, the reward is statistically independent of the action noise
   sampled that tick, and `E[(noise/sigma^2) * reward] = 0` — the update
   doesn't learn a real signal, it's a zero-mean random walk on the
   weights. Using real-world outcomes as reward for an unexecuted action
   isn't "off-policy," it's not policy-gradient-valid at all.

So `update()` must **not** be called from live ticks against real-world
outcomes unless the action actually gets blended into the command (a
bigger, separate decision — closing that loop for a physical vehicle
deserves its own dedicated, carefully-tested phase, not something to
rush in under an unsupervised overnight loop). The valid alternative,
already the plan for Phase 4: train against `sim/learned_plant.py`'s
learned dynamics model instead of live reality — there the "outcome" is a
simulated rollout that genuinely responds to whatever action was sampled,
so the causality is intact, and mistakes cost nothing. This also matches
what was recommended earlier tonight in conversation ("train in sim
first"), independent of this specific finding.

Rescoped:

- [x] **Phase 2** — wire `RLResidualLearner` into `MPCPlanner`, reusing the
      exact same `phi` the supervised residual already computes. Calls
      `act()` every tick for **telemetry only** (a new `rl_residual/status`
      topic — separate from `mpc/status`, which has a hard `len==88`
      assert other code depends on) — this is Phase 5's "report what it
      wants to do" ask, done early since it's low-risk. Does **not** call
      `update()` from live ticks, and does **not** blend the residual into
      `throttle_mps`/`steering_deg` — both deliberately deferred (see
      finding above). New yaml params under `mpc.rl_residual.*`, default
      `mode: off`.
- [~] **Phase 3** — offline training against `sim/learned_plant.py`'s
      dynamics model. **Root cause understood and fix built (firings 7-8),
      RL still doesn't demonstrate it end-to-end yet.** Real linear
      residual signal exists (R^2=0.75, `sim/rl_residual_signal_check.py`)
      and a damped (~3%) version of the correctly-fitted direction
      genuinely improves tracking when applied directly — firings 3-5's
      negative results were never about missing signal. `closed_loop_gain`
      is now built into `RLResidualLearner` (firing 8) and ruled out the
      accel-dimension as a confound too — but actual RL training with the
      gain fix still regresses (-0.09 to -0.11m), likely because damping
      the applied action also damps the reward signal REINFORCE has to
      learn from, needing more samples/a different learning rate than
      tried so far (untested — see firing 8's log). Lower priority than
      the rest of the plan at this point: the concept is proven, this is
      now an RL sample-efficiency tuning question, not an open research
      question. Do not report this phase as "working" until an actual RL
      run (not just the direct-fit regression check) shows a real,
      multi-seed positive result.
- [ ] **Phase 4** — same offline training, but replayed against real bags
      once bags with `/track_angles` data exist (loader ready from
      Phase 1C) — richer than pure sim rollout, still not live.
- [x] **Phase 5** — UI reporting. Done (firing 6) — see log entry below.
- [x] **Phase 6** — tests/polish pass. Done (firing 9) — see log entry.

## Compatibility notes for Phase 1C

`AlignedBag` (in `sim/load_bags.py`) is consumed by `data_sim.py`,
`residual_mlp.py`, `identify_bicycle.py`, `runner.py`, `validate.py` — all
via `AlignedBag.from_npz` on cached `.npz` files, not by calling
`read_bag_streams`/`align_streams` directly (those are only called from
`load_bags.py`'s own CLI). So: new fields are strictly additive, existing
fields/order are untouched, and `from_npz` was made tolerant of missing
keys (old cached `.npz` files won't have the new fields) by NaN-filling
rather than raising.

## Firing log

- **Firing 9** (2026-09-25): Phase 6 — final tests/polish pass, and the
  last unaddressed phase from the original request. All six phases now
  have real, honestly-documented status (four done, two with clear
  documented limitations, none silently claimed as more finished than
  they are).

  - Fixed a real staleness bug: `rl_residual.py`'s module docstring still
    said "Not wired into MPCPlanner/pathfinder_node yet (that's Phase 2)"
    — false since firing 2. Rewrote it to accurately describe where the
    class actually runs today (live telemetry-only in mpc.py; real
    training in sim/train_rl_residual.py) and why update() behaves
    differently in each.
  - Fixed a second, more serious one: this doc's own "Not done yet /
    explicitly deferred" section, written in firing 1, was never touched
    again and had gone factually wrong — it still said "MPCPlanner still
    doesn't consume anything from this work," which Phase 2 changed
    several firings ago. A stale planning doc contradicting its own later
    entries is exactly the kind of thing a polish pass exists to catch.
    Rewrote it to reflect actual current state and point at the Phases
    checklist as the authoritative summary instead of duplicating it.
  - `sim/train_rl_residual.py`'s docstring updated with the current
    honest status (signal proven, mechanism fixed, RL sample efficiency
    still open) instead of stopping at firing 3's framing.
  - Ran `pyflakes` across every file touched or added tonight: zero issues
    in any of tonight's new code. (Two pre-existing unused imports found
    in `master_node.py`/`master_api.py` — not introduced by this session,
    left alone as out of scope for a polish pass on this specific work.)
  - Confirmed `sim/*.py` and `scripts/bag_health_check.py`'s non
    -executable permissions already match this repo's own convention
    (`sim/optimize_sim.py` etc. are non-executable too) — nothing to fix.
  - Full final re-verification: 56 tests passing (44 in
    test_rl_residual.py + test_safety_checker.py, 12 + 1 skipped in
    test_load_bags.py), every touched/new `.py` file compiles, yaml still
    parses (14 `mpc.rl_residual.*` keys, matching the test that guards
    against yaml/code drift), `bag_health_check.py` still correctly flags
    the historical bags, `viz.html`'s embedded JS still syntax-checks
    clean with `node -e`.

  **Stopping the loop here.** All six phases have been substantively
  addressed with honest, verified status — 1/2/5/6 done, 3 has a proven
  concept with an open (lower-priority) tuning question, 4 is blocked on
  data that doesn't exist yet with everything ready for when it does.
  Further firings would either re-tread this ground or chase the narrowing
  Phase 3 RL-tuning thread alone, which is better handed back for a human
  decision (worth more effort, or is the direct-fit result already enough
  signal to act on?) than continued unsupervised iteration. Calling
  CronDelete on this job (`3ad1b15f`) now.

- **Firing 8** (2026-09-25): Built firing 7's concrete next step —
  `closed_loop_gain` on `RLResidualLearner` (default `0.03`, matching that
  firing's empirically-best point), applied to the raw sampled action
  before the max-correction clip, gradient still computed from the
  *unscaled* action (verified this is mathematically valid: the
  score-function estimator only needs the sampled action's log-prob and
  the reward its deterministic transform caused — doesn't matter that the
  environment saw a scaled-down version). New yaml param, 4 new tests
  including one that specifically checks the gradient direction doesn't
  change with gain (only the applied magnitude does) — the fix would be
  worthless if it also crippled learning.

  Re-ran actual RL training (not just firing 7's direct-fit regression) —
  **still regressed** (-0.11m), smaller than firing 5's -0.24m but not the
  clean positive firing 7's regression-only check found. Suspected the
  accel/speed correction dimension might be learning the same
  "always-slow-down" exploit firing 7 found and dropped from the oracle
  check (this training script's reward still has no speed-tracking term).
  Added a `--steer-only` flag to `train_rl_residual.py` (zeroes the
  accel correction before it reaches the plant; permanent, reusable, not
  a one-off hack) and re-ran: **still regressed, almost identically**
  (-0.094m). Ruled that hypothesis out too.

  Remaining likely explanation, not yet tested: `closed_loop_gain=0.03`
  shrinks the actual environmental consequence of exploration noise by
  the same 33x factor, which shrinks the reward-difference signal
  REINFORCE has to work with — the gradient's signal-to-noise ratio is
  now much worse than firing 5's full-gain runs, so 300 episodes
  (~69k ticks) may simply be well short of enough samples, or the
  learning rate needs recalibrating for this smaller-signal regime. This
  is a narrower, lower-value question than what's already been decisively
  answered (real linear signal exists — R^2=0.75, firing 7; the failure
  mode is closed-loop gain, not missing signal or a training bug) and I'm
  not chasing it further tonight — diminishing returns on one increasingly
  narrow sub-question vs. finishing the rest of the original request.
  Whoever picks this back up: try more episodes and/or a higher
  `learning_rate` before anything else.

- **Firing 7** (2026-09-25): Followed firing 5's own recommended next
  step: before guessing at more RL hyperparameters, check whether *any*
  exploitable linear residual signal exists at all, via a method without
  policy-gradient's training-dynamics failure modes. Built
  `sim/rl_residual_signal_check.py`: grid-search a small set of candidate
  steering corrections around the PurePursuit baseline, evaluate each by
  holding it for a fixed horizon on a cloned plant, keep whichever
  minimizes terminal tracking error (a one-step*-lookahead oracle label,
  not a policy), fit a plain closed-form linear regression from
  `_features()` to that label, and evaluate the fitted regression as an
  actual deployed policy in the same harness.

  Two real bugs found and fixed along the way (both informative, not just
  fixed-and-moved-on):
  1. First version also grid-searched a speed/accel correction. The oracle
     always picked the maximum speed reduction, every single tick,
     std=0.000 — reward hacking: a one-tick-ahead, tracking-error-only
     reward trivially rewards slowing toward zero (less distance
     traveled = less positional error), regardless of actual state. The
     resulting "R^2=1.0" was a degenerate artifact of the label being a
     literal constant, not a real finding. Fixed by dropping the
     speed/accel dimension entirely — steering-only isolates the question
     that actually matters for this project.
  2. Steering-only *still* always picked the same extreme grid corner.
     Printed raw reward values across the grid for one tick and found
     **every single grid point tied at exactly reward=0.0** — a 2deg
     steering change over one 16.7ms tick produces no measurable position
     difference (the wheel is rate-limited; lateral position responds to
     steering over many accumulating ticks, not one). The "-2.0 always
     wins" was Python silently keeping the first-tried grid value on an
     exact tie, not a preference. Fixed by holding each candidate constant
     for the same 0.5s/30-tick horizon `nominal_rollout` already uses
     elsewhere in this codebase, scoring terminal error instead of
     next-tick error.

  With both fixed, the result stopped being degenerate: oracle picks a
  nonzero correction on 71.8% of ticks, std=1.6deg (real, state-dependent
  variation), and **the linear regression gets R^2=0.7474** — a linear map
  from these 14 features genuinely explains most of the variance in what a
  half-second-lookahead oracle considers the right correction. Real signal
  exists. This is not what firings 3-5 could conclude on their own.

  But evaluated as an actual per-tick closed-loop policy, the fitted
  regression *still regressed* tracking (-0.297m at full magnitude) — at
  first glance seemingly contradicting the R^2=0.75 finding. Tested a
  damping sweep to find out why: gain 1.0 -> -0.297m, 0.3 -> -0.141m,
  0.1 -> -0.064m, **0.03 -> +0.047m (improved)**, 0.01 -> +0.003m. Monotonic.
  **This is the actual, complete answer for the whole Phase 3 thread across
  firings 3-7**: the oracle solves a *held-for-0.5s, open-loop* question
  ("if I apply this steering offset and don't touch it again for half a
  second, where do I end up") — a valid direction, confirmed by real R^2.
  But naively reapplying that same full-magnitude correction *every tick*
  in closed loop, each one computed as if starting a fresh 0.5s hold,
  compounds tick-over-tick into an over-correction/instability problem
  that has nothing to do with whether the direction is right. Damped to
  ~3% of its fitted magnitude, the same regression direction actually
  helps. This directly explains firing 5's RL result too: the trained
  policy converged (small, bounded weights, no divergence) to something
  that still hurt performance — consistent with sitting in the
  gain-0.1-to-0.3 regime here, which also still hurts despite being the
  right direction, damped only partially.

  **Actual, load-bearing conclusion for anyone picking Phase 3 back up**:
  don't blame the RL algorithm, and don't add more RL-specific
  normalization. The next real step is a closed-loop-stability-aware
  formulation — e.g. an explicit much-smaller effective gain baked into
  the correction (start `max_steer_correction_deg` an order of magnitude
  smaller, or apply an EMA/integral-smoothed version of the correction
  rather than the raw per-tick value), or applying the correction at a
  much lower rate than 60Hz so each application genuinely gets its held
  -horizon to play out before the next one lands. `RLResidualLearner`'s
  existing per-tick `act()`/`update()` shape doesn't have anything like
  this yet.

  `sim/rl_residual_signal_check.py` is a standing tool now, not a one-off
  — its gain sweep is a permanent part of its output, not just this
  firing's interactive exploration, so it's reproducible for whoever picks
  this up next.

  *(*"one-step" in the phrase above is used loosely — the label-generation
  horizon is 30 ticks/0.5s, "one-step" refers to it being a single fixed
  -horizon lookahead per grid point, not a multi-step search tree.)*

- **Firing 6** (2026-09-25): Deliberately switched off Phase 3 (the RL
  -helps-or-not research question from firings 3-5 needs a different kind
  of investigation — a supervised-regression sanity check, not more
  overnight guessing) and did Phase 5 instead: UI reporting, an explicit
  part of the original request that had been sitting untouched since
  Phase 2 built the ROS-side telemetry. It doesn't depend on Phase 3 being
  resolved — reporting "what the policy wants" is useful regardless of
  whether that policy is good yet.

  Followed the exact existing pattern for getting ROS telemetry to the
  frontend (`master_node.py` caches the latest message behind `self._lock`
  with a getter, `master_api.py` exposes it as a Flask route, `viz.html`
  polls and renders):
  - `master_node.py`: subscribed to `rl_residual/status` (the topic
    `mpc.py` already publishes since Phase 2), `_rl_residual_callback` +
    `get_rl_residual_status()`, mirroring `_mpc_status_callback`/
    `get_mpc_status` exactly.
  - `master_api.py`: new `/rl_residual_status` GET route, one line,
    matching `/mpc_status`'s.
  - `viz.html`: a single new HUD row — `RL   shadow wants S:+0.30° A:-0.05
    (n=1240)` — styled deliberately dimmer than the primary telemetry
    above it (`#7a7a88` vs the HUD's normal `#e0e0e0`) and **hidden
    entirely** until `rl_residual/status` has actually reported something,
    so it's invisible when irrelevant and quietly present when not — the
    literal "subtle but still a present, reporting feature" the request
    asked for, not a new prominent panel. Polls at 5Hz (`/lines`'s
    cadence) — it's a report, not a control signal, doesn't need 10-20Hz.
  - Tests: 2 new cases in `test_master_node.py` (default-state-before-any
    -message, and a full publish-through-executor-to-getter roundtrip),
    mirroring `test_odom_snapshot_and_cmd_callbacks`'s exact style. Needs
    real rclpy to run (not available on this host, same caveat as all the
    ROS-integration work tonight) — confirmed the file still imports and
    gracefully skips rather than erroring, and JS-syntax-checked
    `viz.html`'s embedded script with `node -e`.

  All 6 requested phases now have *something* built: 1 (scaffold), 2 (live
  telemetry wiring), 3 (offline training harness — sound, but the RL
  residual doesn't yet demonstrably help, honestly documented), 4 (blocked
  on data that doesn't exist yet, also honestly documented, with the
  loader ready for when it does), 5 (this firing), and 6 hasn't been
  touched (a final tests/polish pass — reasonable next firing).

- **Firing 5** (2026-09-25): Swapped `sim/train_rl_residual.py`'s from
  -scratch baseline for the real `PurePursuitPlanner` (confirmed zero ROS
  deps, directly importable), using the team's own tuned
  `pure_pursuit.*`/kart-wide yaml values, not guesses. **This alone did
  not fix the off-track rate** (still ~43% with zero RL involved) — turned
  out every racing-line CSV in `data/racing_line/` has segments exceeding
  `a_lat_max_mps2` at their stored speed (`center.csv`: 4% of points over;
  the others: 30-43%), because none of these files' speed columns are
  curvature-adjusted, and neither PurePursuit nor (per the traced code)
  even the live MPC's own `throttle_mps` dynamically reduces speed for
  curvature — it's `min(line_vx, target_speed)`, full stop. So a nonzero
  off-track rate on tight sections is inherent to these files, not a
  controller bug — stopped chasing "zero off-track" as the wrong target
  and moved on to the actual question: does the RL residual help,
  relative to the same baseline, on the same episodes.

  Ran the full training comparison with the validated PurePursuit baseline
  anyway: **still regressed, and by more** (-0.26m, worse than the -0.10 to
  -0.11m seen with the broken ad-hoc baseline). Checked weight norms:
  still diverging (354, 2525) despite whitening + mean-baseline + grad
  clip from firings 3-4. Diagnosed the remaining gap: the reward
  (`-(100*d^2 + 30*heading_err^2)`) is heavy-tailed — a near-boundary tick
  can be ~600x larger than a well-tracked one — and I'd only ever
  subtracted the reward's running *mean*, never normalized its *scale*.
  Added reward-variance normalization (z-scored advantage, new
  `min_reward_std` param, 2 new tests: `test_large_and_small_rewards_...`
  directly checks a 600x-larger reward no longer produces a wildly
  larger step) — same EMA pattern as the existing feature whitener and
  reward-mean baseline. Also added `_reward_baseline`/`_reward_var` to
  `state_dict()`/`load_state_dict()`, which had been missing (an
  inconsistency with the feature-normalizer state, which was already
  persisted).

  **Re-ran — divergence is genuinely fixed this time**: weight norms went
  from (354, 2525) to (1.8, 13.2), a real, bounded, converged policy.
  **But it still regresses tracking performance** (-0.24m). This is now a
  clean, trustworthy result, not a bug — no more diverging, no more broken
  baseline, no more raw-reward-scale artifact. The honest conclusion:
  **with a linear policy over these 14 features, this reward, and the
  PurePursuit baseline, the RL residual reliably converges to something
  that hurts tracking rather than helps it.** That could mean there's
  genuinely very little useful residual signal left on top of an
  already-tuned Pure Pursuit controller (plausible — it's not MPC, but
  it's not naive either), or that a linear policy over this exact feature
  set can't express whatever residual correction *would* help, or
  something else not yet identified. I don't know which, and I'm not
  going to guess further this firing — three fixes in a row (whitening,
  reward-mean baseline, reward-variance normalization) each looked
  plausible and none flipped the sign, which is itself informative: this
  isn't "one more normalization bug" away from working.

  **Do not report Phase 3 as working.** All the fixes made across firings
  3-5 are real and worth keeping (`RLResidualLearner` no longer diverges,
  which matters for Phase 2's live telemetry too, and the training harness
  is now sound), but the actual question — does an RL residual help this
  system — has an honest current answer of "not demonstrated, and the
  obvious bugs are now ruled out." If a future firing or the user wants to
  push further, the next real experiments (not more normalization
  patching) would be: (a) sanity-check whether *any* exploitable residual
  signal exists at all — e.g. fit a supervised regression from features to
  "what correction would have minimized next-tick error" and see if it
  explains meaningful variance before trusting any RL approach to find it;
  (b) try a much smaller learning rate / more conservative initialization
  and confirm the converged policy is near-zero rather than confidently
  wrong; (c) only if those suggest real signal exists, consider a more
  expressive (non-linear) policy.

- **Firing 4** (2026-09-25): Chased firing 3's regression to ground —
  found and fixed three distinct, real bugs, but the offline-training
  experiment is *still* not a clean positive result. Documenting all of it
  rather than declaring victory early (I nearly did, twice, this firing).

  1. **Fixed**: added online feature whitening (EMA mean/var) to
     `RLResidualLearner` — `_features()`'s 14 values are raw physical
     quantities at wildly different scales, which was contributing to the
     REINFORCE divergence. Bias term (index 0) explicitly excluded from
     whitening (it's a constant, normalizing it divides by ~0 variance).
     New params `feat_norm_beta`/`min_feat_var`/`bias_index`, 4 new tests
     (39 total in the file, all passing). **Re-ran the 300-episode
     training — result barely changed (-0.1128m, essentially identical to
     before).** This told me the bug wasn't (only) in the learner.
  2. **Found via direct measurement, not guessing**: ran the baseline
     controller alone, zero RL involvement at all — **57% of episodes
     already went off-track with no residual whatsoever.** The whole
     "trained vs. baseline" comparison had been meaningless the entire
     time; both numbers were dominated by baseline-controller instability,
     not by whether the RL residual helps.
  3. **Fixed**: the baseline P-controller had no curvature feedforward —
     a bare PD controller on cross-track/heading error always lags into a
     turn instead of anticipating it. Added `atan(wheelbase * kappa)`
     feedforward. Off-track rate dropped from ~55-78% (tried several PD
     gains, all bad) to ~48-53%. Real improvement, not sufficient alone.
  4. **Fixed**: `data/racing_line/center.csv`'s speed profile is a flat
     6 m/s *everywhere*, including through its tightest corner
     (`kappa=0.198`), which demands ~7.1 m/s² of lateral acceleration —
     more than the kart's own `a_lat_max_mps2: 5.3`. That corner is
     physically infeasible to hold at 6 m/s regardless of controller
     quality; the real MPC handles this by jointly optimizing speed and
     steering, which this simplified baseline never did. Added a curvature
     -based speed cap (`v <= sqrt(a_lat_max / |kappa|)`). Off-track rate:
     no further meaningful change (~48-53% again).
  5. **Ruled out**: traced one full trajectory tick-by-tick and checked
     `LearnedPlant`'s own `out_of_hull` counter — **zero** out-of-hull
     queries across a full 300-tick episode. The plant model is not
     extrapolating outside its training data; whatever's failing isn't a
     model-reliability problem.

  **Still stuck at ~50% off-track with a from-scratch P+feedforward+speed
  -cap baseline, and I'm stopping here rather than keep guessing at gains.**
  The actual next step, which I should have done from the start instead of
  writing a new controller: **reuse `PurePursuitPlanner`
  (`nodes/pathfinder/planners/pure_pursuit.py`) as the baseline instead of
  an ad-hoc PD controller** — it's already a proven, tuned component in
  this exact codebase, and I confirmed it has zero rclpy/ROS dependencies
  (`import math`, `dynamic_line`, `pathfinder`, `base`, `rejoin` — all
  pure-Python), so it's directly importable into `sim/train_rl_residual.py`
  the same way `_features()` already is. That's a real, concrete, checked
  next step, not a guess.

  All the fixes made this firing (feature whitening, curvature
  feedforward, speed cap) are genuine improvements kept in place — none of
  them are wrong, they just weren't sufficient on their own to produce a
  trustworthy baseline. `RLResidualLearner`'s divergence fix in particular
  is independently verified by unit tests regardless of the baseline issue.
  No claim of "Phase 3 works" until the baseline is swapped for
  `PurePursuitPlanner` and a multi-seed run shows a real, honest positive
  delta.

- **Firing 3** (2026-09-25): Phase 3 started — offline training against
  `sim/model/plant_linear.json` (the team's own linear-ARX plant, chosen
  over the NN variant per `learned_plant.py`'s own docstring: better
  accuracy *and* lower variance, no torch needed). Built
  `sim/train_rl_residual.py`: a standalone Frenet helper, a simple P-control
  baseline standing in for MPC (replicating the real sampling-MPC solve
  here would be a large duplication — out of scope), and a training loop
  where the RL residual's correction is actually added to the command fed
  into the plant, so — unlike live ticks — the simulated reward genuinely
  responds to the sampled action and the policy-gradient estimate is valid.

  **Honest result: this does not work yet.** A 5-episode smoke test showed
  improvement (0.71m -> 0.17m mean |d|) and I almost took that as "it
  works" — a 300-episode run immediately showed the opposite: baseline
  0.65m -> trained 0.76m (**regressed**), ~30% of episodes ending early by
  going off-track, and weight norms exploding to >3000. That's classic
  REINFORCE-on-an-always-negative-reward divergence (the tracking-cost
  reward never crosses zero, so an unbaselined gradient has a
  systematically-signed but uninformative direction). Added a reward
  baseline (running mean, subtracted as an advantage — an unbiased,
  standard fix) and a grad-norm clip to `rl_residual.py` (new params:
  `reward_baseline_beta`, `max_grad_norm`; 6 new tests, all passing) and
  **re-ran — it did not fix it.** Nearly identical regression (-0.10m
  again), weight norms still hitting 500-2500.

  Diagnosis: the grad-norm clip only bounds a *single* step, not
  accumulation over ~60,000 updates in one run — if the gradient direction
  is systematically (not just occasionally) biased, thousands of small
  clipped steps still drift the same way. The likely root cause: the 14
  features (`_features()`, mpc_residual.py) are raw, unnormalized physical
  quantities at wildly different scales (`d` in meters, `v_s`/`v_d` in m/s,
  `kappa` ~0.001, steer/throttle history in radians/m-s deltas, plus a
  constant bias term) feeding a plain linear policy with one global learning
  rate — exactly the kind of thing `LearnedPlant`'s own `Whitener` class
  exists to fix for the plant model, and I skipped doing the equivalent for
  the policy in the Phase 1D scaffold.

  **`sim/model/rl_residual_checkpoint.json` on disk right now is bad —
  don't load it, don't trust it.** Nothing in the live system loads RL
  residual checkpoints yet (Phase 2 has no load hook), so this is
  contained, but flagging clearly so a future firing or the user doesn't
  mistake "a checkpoint exists" for "training worked."

  Also worth knowing: I only tested one seed (`--seed 0`) both times, so I
  don't actually know how much of this is seed variance vs. a real
  structural problem — the two runs' near-identical final numbers suggest
  it's structural (feature scale), not seed luck, but that's not proven.

  Not attempting more blind fixes this firing — this deserves focused
  attention, not more rushed patches. Next firing should, in order:
  1. Add feature whitening to `RLResidualLearner` (mirror `Whitener` from
     `sim/plant_dataset.py` — fit mean/std from a batch of rollout features
     before training, or maintain a running normalizer).
  2. Consider episodic (per-episode return) REINFORCE instead of per-tick
     updates — per-tick updates on a correlated within-episode reward
     stream is a known higher-variance setup than accumulating one return
     per episode.
  3. Re-run across >=3 seeds before drawing any conclusion, report the
     spread not just one number.
  4. Only update this doc's phase checkbox once a multi-seed result is
     actually, honestly positive.

- **Firing 2** (2026-09-25): Phase 2 done and verified, rescoped along the
  way (see "Reward-validity finding" above — read that before touching
  `update()` anywhere near live ticks).
  - `mpc.py`: constructs `self.rl_residual` (mirrors `self.residual`'s
    construction), reuses the exact same `phi` already computed for the
    supervised residual (no new feature-extraction code), publishes
    `rl_residual/status` (new topic, separate from `mpc/status` which has
    a hard `len==88` assert). `act()` called every tick with
    `explore=False` (report the deterministic mean, not sampling noise).
    `update()` is never called from live ticks — intentional, see finding.
  - `pathfinder.yaml`: new `mpc.rl_residual.*` params, `mode: off` default.
  - `test/test_rl_residual.py`: +2 tests — constructs with the real
    `NUM_FEATURES` from `mpc_residual.py` (catches drift), and asserts the
    yaml's `mpc.rl_residual.*` keys exactly match what
    `RLResidualLearner.__init__` reads (catches param-name typos between
    yaml and Python). 16 tests total in that file, all passing (33/33
    across it + `test_safety_checker.py` together).
  - Could not run the real `MPCPlanner`/`PathfinderNode` integration path
    (needs rclpy + the actual ROS2 message packages, not available on this
    host) — only compile-checked `mpc.py` and manually re-read every
    touchpoint. **This needs a real `colcon build && colcon test` pass in
    the devcontainer before trusting it**, same caveat as the SafetyChecker
    work earlier tonight.
  - Next firing: Phase 3 — offline training against `sim/learned_plant.py`.
    Read that module first; don't assume its interface, verify it.

- **Firing 1** (2026-09-25): Phases 1A-1D done and verified.
  - `scripts/bag_health_check.py` — ran against all 32 bags, correctly
    flags every one for empty camera data (matches the finding above).
  - `sim/load_bags.py` extended additively (`track_angle_right/left`,
    `mpc_d`, `mpc_psi_track`, `mpc_cost`, `mpc_margin_min`, all NaN when
    source topic is empty); `sim/tests/test_load_bags.py` grew from 6 to
    13 cases (5 passed + 1 skipped before -> 12 passed + 1 skipped after,
    same skip reason as before — missing holdout cache, unrelated to this
    change). All passing.
  - `nodes/pathfinder/planners/rl_residual.py` — `RLResidualLearner`,
    linear-Gaussian policy + REINFORCE update, mode-gated (`off` default).
    Not wired anywhere yet. 14 new unit tests in
    `test/test_rl_residual.py`, all passing.
  - Verified via a throwaway venv (pytest+numpy, no rclpy needed for any
    of this — same approach as today's earlier `SafetyChecker` work).
  - Next firing: start Phase 2 (wire `RLResidualLearner` into
    `MPCPlanner`/`pathfinder_node` in shadow mode, new `mpc.rl_residual.*`
    yaml params, default `mode: off`).

## Not done yet / explicitly deferred

(Superseded by later firings in places — kept for history, but see the
"Phases" checklist above for the current, accurate status of each phase.)

- **MPCPlanner reads `RLResidualLearner` for telemetry only** (Phase 2,
  firing 2) — `act()` runs every tick, published to `rl_residual/status`.
  It still doesn't feed into `throttle_mps`/`steering_deg` at all, by
  deliberate choice (see the reward-validity finding above) — that
  remains genuinely not done, same scope decision as SafetyChecker earlier
  that night.
- **RL training itself doesn't reliably help yet** (Phase 3) — the concept
  is proven (real signal, R^2=0.75) and the mechanism that was breaking it
  is fixed (`closed_loop_gain`), but actual RL runs still don't cleanly
  beat baseline. See firing 8's log for the specific untested next step
  (more episodes / higher learning rate).
- **Phase 4 (bag replay) has no real data to run against** — every
  historical bag predates the camera pipeline. The loader is ready
  (Phase 1C); this is "run it," not "build it," whenever bags with real
  `/track_angles` data exist.
- **No colcon/rclpy integration testing was possible from this session** —
  every ROS-touching change tonight (mpc.py, master_node.py, master_api.py,
  pathfinder_node.py/safety_checker.py earlier) was compile-checked and,
  where the test itself doesn't need rclpy, unit-tested, but never run
  through the real ROS2 graph. Needs a real `colcon build && colcon test`
  in the devcontainer before trusting any of it on the kart.
- No commits made — repo instructions are to never commit without being
  asked. Everything from this `/loop` session is uncommitted working-tree
  changes (see `git status` — 8 modified + 6 new files as of firing 9).
  Review as one batch when you're back.
