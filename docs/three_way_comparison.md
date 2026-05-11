# Three-way comparison — Plans A, B, C

Last updated: 2026-05-10 (post-shipping diagnostic revisions)

## Headline

| Plan | Architecture | Data | Val MAE (lin mm/s) | Val MAE (ang deg/s) | Compose score |
|---|---|---|---|---|---|
| **A** | ACT 50k step | 100 ep | 1.272 | 0.073 | **43.89** |
| **B** | ACT 40k step (val-split) | 300 ep | 1.329 | 0.065 | **112.90** ✅ shipping |
| **C** | Diffusion 40k step | 300 ep (same as B) | 3.44 ¹ | 0.239 ¹ | **86.03** |

¹ *Plan C val MAE is methodologically biased (see Phase 5). The ACT vs Diffusion val comparison is not apples-to-apples; compose is the only fair metric.*

**Final ranking: B > C > A.** Plan B wins (112.90); Plan C beats A by 1.96× but loses to B by 23%.

## Lessons (revised post-diagnostic)

1. **`Total end-effector path length: 0.00 m` is a measurement artifact, not a fact about the gripper.** Earlier versions of this document built a whole "regression to tiny actions" thesis on this single field. Per-tick TICK-logged diagnostics during a Plan B compose showed the gripper TCP actually traverses **200/87/300 mm vertically + 90–100 mm horizontally** across the three trials. The model commands realistic ~−10 mm/s lin_z (matching the training distribution mean) and the controller executes it. See `analysis/plot_deploy_diagnostic.py` and Phase 7 below for the data.

2. **Architecture was not the bottleneck.** Plan A, Plan B, and Plan C all produce reasonable gross motion at deploy. Plan B beat Plan C primarily on trial 3, not on a fundamental ACT-vs-Diffusion difference.

3. **The real bottleneck is last-cm alignment for insertion.** The model successfully descends to insertion altitude in ~5–7 s, then commands ≈0 velocity (correctly slowing for alignment). It then fails to make the precise corrections needed to thread the plug into the port. Cable ends 4–7 cm short on trials 1–2; 7 cm or 19 cm on trial 3 depending on plan.

4. **Plan B's win came from robustness on OOD geometry.** Plan A collides on trial 3 (-35 penalty); Plan B and C don't collide. Plan B happens to drift to within the bounding radius (+35.69); Plan C drifts further (1.00 score). Trials 1+2 are essentially tied across all three plans.

5. **The val-MAE Plan-A-vs-Plan-B wash is still accurate.** 3× the data did not improve average prediction quality on held-out episodes. The compose-score improvement is about outlier reduction, not better predictions on average.

**Shipped: Plan B step-40k.** Plan C is preserved on disk and on the branch but not deployed.

---

## Phase 1 — Data collection (✅ complete)

| Metric | Value |
|---|---|
| Episodes added | 200 (101–300) |
| Final dataset | **300 episodes / 179,095 frames** |
| Wall clock | 270 min (4 h 30 m) |
| Drops | 0 |
| Dataset size on disk | 1.7 GB |
| HF dataset | `saivemu/aic_act_v1` |

Rate decay: started at 1.50 ep/min, decayed to ~0.52 ep/min mid-run, partially recovered to 0.86 ep/min near completion. Per-episode capture time stayed normal (~40 s); slowdown was in the inter-episode gap (Gazebo physics state accumulation across resets).

Quality audit from data-collection scoring.yaml: **92/100 trials achieved full insertion** (tier_3 = 75), 3/100 partial, 5/100 no insertion. No contacts or force penalties on any trial.

---

## Phase 2 — Training (✅ complete)

| Metric | Value |
|---|---|
| Train slice | episodes [0..254] (255 eps, 151,891 frames) |
| Val holdout | episodes [255..299] (45 eps, 27,204 frames) |
| Steps | 40,000 (Plan A's 50k regressed; see Phase 3) |
| Final loss | 0.035 |
| Final grad norm | 4.201 |
| Wall clock | 44 m 44 s |
| Step rate | 14.9 step/s |
| Output dir | `outputs/train/act_aic_v1_planb` |
| W&B | `aic-act-plan-b @ RebisVla` |

---

## Phase 3 — Per-dim val MAE (✅ complete)

### Plan B on its own held-out [255..299]

| step | lin_mean (mm/s) | ang_mean (deg/s) | best on dims |
|---|---|---|---|
| 10k | 1.678 | 0.090 | — |
| 20k | 1.336 | 0.067 | lin_x, ang_y |
| 30k | 1.443 | 0.073 | ang_z |
| **40k** | **1.329** | **0.065** | lin_y, lin_z, ang_x — winner |

### Apples-to-apples: Plan A vs Plan B at step 40k on the same val [255..299]

| Plan | lin_mean (mm/s) | ang_mean (deg/s) |
|---|---|---|
| **Plan A** 40k | **1.272** | 0.073 |
| **Plan B** 40k | 1.329 | **0.065** |

Net: **indistinguishable.** A wins linear by 4%, B wins angular by 11%. 3× the training data did not improve average prediction quality on held-out data. The compose-score improvement from A → B has to be explained by something other than per-frame prediction accuracy (and per Phase 7, it is: reduced outlier failures, especially on trial 3).

### Plan B vs CheatCode (training-data) actions

See `analysis/plots/training_action_overlay.png` (regenerate via `analysis/plot_action_overlay.py`).

The plot overlays Plan B's predicted action chunks on ground-truth CheatCode actions for 4 training episodes. Key observations:
- **Linear dims**: CheatCode commands consistently ~−10 mm/s lin_z (downward) and bimodal lin_x/y. Plan B's chunk predictions track these closely.
- **Angular dims**: CheatCode commands are mostly zero (~90% within ±0.5 deg/s) with brief bursts. Plan B also predicts ~zero with bursts.
- **TCP path length**: training episodes have 53–353 mm of xy traversal (computed from observation.state[:3]). The gripper does move meaningfully in training.

This contradicts the earlier "regression to small actions" framing. Plan B reproduces CheatCode's training-data actions well in the prediction sense.

---

## Phase 4 — Compose eval (✅ complete)

**Plan B compose total: 112.90 — 2.57× over Plan A's 43.89.**

| Metric | Plan A | Plan B step 40k | Δ |
|---|---|---|---|
| **Compose total score** | **43.89** | **112.90** | **+157%** |
| Trial 1 | 42.58 | 33.67 | -21% |
| Trial 2 | 36.30 | 43.54 | +20% |
| Trial 3 | -35.00 (collision) | **35.69** (clean) | **+70 pts** |
| Reported "path length" | mixed | 0.00 m (all trials) ¹ | scoring artifact |
| Final dist trial 1 | 0.05 m | 0.07 m | slightly worse |
| Final dist trial 2 | 0.07 m | 0.05 m | slightly better |
| Final dist trial 3 | collision | 0.07 m | recovery |
| Contacts | 1 (trial 3) | **0** | ✅ |
| Force penalties | 0 | 0 | unchanged |

¹ *See Phase 7. The reported path length is computed in the scoring bag from the `gripper/tcp` TF in `aic_world`; our per-tick TICK diagnostic shows the actual TCP traversed 200–300 mm vertically, but the bag-side measurement doesn't reflect that.*

### Where the 2.57× win came from

The improvement is **entirely driven by trial 3**: Plan A collides with the enclosure (−35), Plan B navigates cleanly through the same OOD scene config (+35.69). Trials 1+2 are within ±20%.

3× the training data didn't make the model better at *predicting* CheatCode actions on held-out frames (val MAE wash), but it *did* reduce the OOD-scene catastrophic failure rate. The yaw=3.0 trial geometry that pushed Plan A into collision is now closer to Plan B's in-distribution manifold.

### Tier-3 breakdown

- Trial 1: tier_3 = 16.08 ("No insertion. Final plug port distance: 0.07m.")
- Trial 2: tier_3 = 25.00 ("No insertion. Final plug port distance: 0.05m.") — 25 is the partial-insertion bonus floor
- Trial 3: tier_3 = 17.09 ("No insertion. Final plug port distance: 0.07m.")

**No trial achieved full insertion** (would have been +75 each). The gripper drives to insertion altitude but doesn't make the precise xy alignment needed to thread the plug.

---

## Phase 5 — Plan C (Diffusion Policy) training (✅ complete)

Trained on the same 300-ep dataset, same train/val split as Plan B.

| Metric | Plan B (ACT) | Plan C (Diffusion) |
|---|---|---|
| Params | 52M | **271M** (5.2×) |
| Optimizer LR | 1e-5 | 1e-4 (default) |
| Step rate | 14.9 step/s | 10.8 step/s |
| Wall clock | 44 m | **62 m** (1.4×) |
| Final loss | 0.035 (L1) | 0.000 (ε-MSE) — *different scales, not directly comparable* |
| GPU mem | 2.3 GB | 7.2 GB |
| W&B | aic-act-plan-b | `aic-dp-plan-c` (run u4vwif7b) |
| Steps | 40k | 40k |

### Per-dim val MAE evolution (held-out [255..299])

| step | lin_mean (mm/s) | ang_mean (deg/s) |
|---|---|---|
| 10k | 6.74 | 0.853 |
| 20k | 5.66 | 0.778 |
| 30k | 4.83 | 0.299 |
| **40k** | **3.44** | **0.239** |

### ⚠️ Eval methodology caveat — Plan C val MAE is biased

`aic_utils/lerobot_robot_aic/scripts/eval_checkpoints.py` measures MAE by iterating the val dataloader (one frame per step), calling `policy.predict_action_chunk(batch)`, and comparing slot 0 to GT.

This works cleanly for ACT (n_obs_steps=1, stateless predict_action_chunk). For diffusion (n_obs_steps=2, queue-based), the script fakes 2 timesteps by **duplicating the single observation** along the n_obs_steps dim. The diffusion model was trained with two distinct timesteps so it has never seen `obs[t] == obs[t-1]` — this is OOD input that likely degrades predictions.

The **2.6× linear / 3.7× angular gap is partly a measurement artifact, not a model-quality fact**. The fair test is compose (Phase 6).

---

## Phase 6 — Plan C compose eval (✅ complete)

**Plan C compose total: 86.03.** Beats Plan A (43.89) by 1.96×; loses to Plan B (112.90) by 23%.

| Trial | Plan A | Plan B | Plan C | Diff (C vs B) |
|---|---|---|---|---|
| 1 | 42.58 | 33.67 | **41.32** | C +23% |
| 2 | 36.30 | 43.54 | **43.71** | C +0.4% |
| 3 | -35.00 (collision) | 35.69 (clean) | **1.00** | C −97% |
| **Total** | **43.89** | **112.90** | **86.03** | **C −23.8%** |

### Per-trial details for Plan C

- **Trial 1**: tier_1=1, tier_2=17.70, tier_3=22.62. Cable ended 0.05 m from port. No contacts.
- **Trial 2**: tier_1=1, tier_2=17.71, tier_3=25.00. Cable ended 0.04 m from port. No contacts.
- **Trial 3**: tier_1=1, tier_2=**0**, tier_3=**0**. Cable ended **0.19 m** from port — *outside the bounding radius*, so no tier_2 or tier_3 credit ("Plug is not within max bounding radius from target port").

### Compose-truth ranking is much closer than val-MAE suggested

- Val MAE (biased): C is 2.6× worse than B on linear, 3.7× worse on angular
- Compose: C is 23% worse than B overall, **tied or slightly better on trials 1+2**

The biased val eval was the reason I almost skipped deploying Plan C. The compose result shows the policies are much closer in practice than that signal indicated.

### Engineering notes

- **Deploy adapter ended up much smaller than estimated.** Initial estimate: ~110 LoC + 1 hr. Actual: **~30 LoC + 30 min.** Key realization: `policy.select_action()` is polymorphic across ACT and Diffusion — it internally manages each architecture's queue/chunk-replay state. The control loop didn't need changes. Edits were: (1) type-dispatch the policy class load, (2) branch state+action normalization on MEAN_STD vs MIN_MAX.

- **Build gotcha — pixi build cache.** `docker compose build` initially served a stale conda package of `aic_example_policies` because pixi's build cache hashes by lockfile, not source content. Fix: added a Dockerfile RUN that overlays the COPY'd source onto site-packages after `pixi install`. This pattern is required when iterating on local conda packages inside docker builds.

---

## Phase 7 — Post-shipping diagnostic (✅ complete)

After shipping Plan B to ECR (`plan-b-v3`), we instrumented `RunACT.py` with per-tick logging and ran a diagnostic compose to test the "regression to tiny actions" hypothesis that had been the dominant interpretive framework through phases 4–6.

### Setup

Temporarily added a `[TICK]` log line per control tick emitting: trial-relative sim time, raw 6-D action, commanded last_target xyz, observed tcp xyz, tcp_lin_vel xyz, wrist force magnitude. Built a debug image (not pushed to ECR), ran a single 3-trial compose with `AIC_ENABLE_ACL=true`, parsed the resulting log with `analysis/plot_deploy_diagnostic.py`.

The TICK-logging edit is **not** in the deployed RunACT.py. To rerun, re-add the log line and rebuild a debug image. The plot script is in the repo and handles parsing.

### Findings

**The gripper actually moves substantially in all three trials.** Compose total this run: 105.68 (within sim variance of the original 112.90).

| Trial | tcp_z Δ | tcp xy cumulative | Commanded vs observed lin_z |
|---|---|---|---|
| 1 | **−200 mm** | 94 mm | cmd −9.13 / obs −6.77 mm/s (74% efficiency) |
| 2 | −87 mm | 91 mm | cmd −9.62 / obs −2.91 mm/s (30% — something resisting briefly) |
| 3 | **−300 mm** (workspace floor) | 101 mm | cmd −12.95 / obs −10.01 mm/s (77%) |

Force stays near baseline (~20 N from gripper+cable static load) the whole time on all three trials. No contact, no constraint resisting motion.

### Why the scoring's "Total end-effector path length: 0.00 m" doesn't match this

`aic_scoring/src/ScoringTier2.cc:670–678` computes path length by summing L2 displacement of the `gripper/tcp` TF frame relative to `aic_world` across consecutive TF samples in the scoring bag. The score is then:

```cpp
// ScoringTier2.cc:527
if (measurement <= min_range) {
    return max_score;
}
```

Score = `CalculateInverseProportionalScore(6.0, 0.0, minPath+1.0, minPath, totalPath)` where `minPath = initial plug-port distance`. If `totalPath ≤ minPath`, the formula returns max score (6.0) regardless of how small `totalPath` is.

So "Total end-effector path length: 0.00 m" with score 6 means the bag-side accumulated path of `gripper/tcp` is below the minimum threshold — which yields a max-score "perfect efficiency" result. **The "0.00 m" is the actual computed value, but it doesn't mean the gripper is stationary.** Plausible explanations for the disagreement with the TCP motion we observed:

1. **TF sampling rate / dedupe**: TfCallback dedupes by `header.stamp`. With chained TF lookups, the gripper/tcp composite samples may be sparser than the raw joint TF updates, so the bag-side cumulative path is undersampled.
2. **Frame mismatch**: Our diagnostic logged `controller_state.tcp_pose` (a custom field), the scoring uses TF `gripper/tcp` → `aic_world`. They should be the same physical pose, but the scoring's bag-side data may not resolve continuously.

Either way: the deployed `aic-runact:plan-b-v3` image is fine, and the model is doing the right gross motion. We can't reliably use the scoring report's `Total end-effector path length` field as evidence about the gripper's actual motion.

### The actual failure mode

Looking at the trial-time evolution: the model descends rapidly for ~5–7 s, then commands ≈0 lin_z. That's correctly slowing for the alignment phase. **But the alignment phase never produces the precise xy corrections needed to thread the plug into the port.** Cable ends 4–7 cm short.

**This reframes everything:**

- ACTION_SCALE = 6.0 (grkw's setting) would have made gross motion overshoot — wrong move
- Removing the 2 cm offset clamp wouldn't help — the clamp barely fires when actions are small
- "Architecture diversity" hypothesis (Plans A → B → C) was solving a problem that doesn't exist: gross motion was always fine

The **real question** is what happens in the last 5 cm of CheatCode demos. If CheatCode also goes quiet and the cable settles via physics, our model can't learn alignment because the data doesn't have it. If CheatCode commands sustained tiny corrections during alignment, our model is failing to predict them. This is the next investigation to run (Phase 8, TBD).

---

## Recommendation

**Ship Plan B step-40k (`aic-runact:plan-b-v3`).** 2.57× over Plan A. Robust on OOD geometry. Both Plans B and C avoid Plan A's collision.

**Don't pursue more data or architecture-diversity experiments until we've characterized the alignment phase in CheatCode demos.** The path-length-0.00 m red herring sent us on two training runs that probably didn't address the real bottleneck.

---

## Files

### Plan B
- Checkpoint: `/home/saivemu/code/aic-train/outputs/train/act_aic_v1_planb/checkpoints/last/pretrained_model`
- HF: https://huggingface.co/StrivingBapan/aic_act_v1_planb_300ep
- ECR: `973918476471.dkr.ecr.us-east-1.amazonaws.com/aic-team/bot-squad-l2-learning-loop:plan-b-v3`
- W&B: `aic-act-plan-b @ RebisVla`

### Plan C
- Checkpoint: `/home/saivemu/code/aic-train/outputs/train/dp_aic_v1_planc/checkpoints/last/pretrained_model`
- W&B: `aic-dp-plan-c @ RebisVla` (run u4vwif7b)

### Analysis artifacts
- `analysis/plot_action_overlay.py` — training-time action overlay (GT vs predicted)
- `analysis/plots/training_action_overlay.png` — generated output
- `analysis/plot_deploy_diagnostic.py` — parser+plotter for per-tick TICK log
- Diagnostic plot needs a fresh debug compose run; the TICK logging is not in the deployed RunACT.py

---

## Cost summary

- Plan A: ~5 hr (done before this session)
- Plan B: ~5.5 hr (data 4.5 + train 0.75 + eval 0.25)
- Plan C: ~1.4 hr (train 1.0 + val MAE 0.1 + deploy adapter 0.5 + compose 0.2)
- Submission iteration (v1 → v3, including lazy-import fix): ~2 hr
- Post-shipping diagnostic (Phase 7): ~1 hr
- Total elapsed for this session: ~10 hr
