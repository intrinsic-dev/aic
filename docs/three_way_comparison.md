# Three-way comparison — Plans A, B, C (2026-05-05)

## Headline

| Plan | Architecture | Data | Val MAE (lin mm/s) | Val MAE (ang deg/s) | Compose score |
|---|---|---|---|---|---|
| **A** | ACT 50k step | 100 ep | 1.272 | 0.073 | **43.89** |
| **B** | ACT 40k step (val-split) | 300 ep | 1.329 | 0.065 | **112.90** ✅ shipping |
| **C** | Diffusion 40k step | 300 ep (same as B) | 3.44 ¹ | 0.239 ¹ | not run ² |

¹ *Plan C val MAE is methodologically biased — see Phase 5 caveat. The ACT vs Diffusion val comparison is not apples-to-apples in this eval.*

² *Plan C compose eval requires ~1hr of RunACT.py adapter work (ACT-hardcoded today). Deferred to user.*

**Shipped: Plan B step-40k. Compose 112.90 = 2.57× over Plan A.** The win was robustness on OOD geometry (no collision in trial 3), not navigation quality (path length 0.00m on all trials, no full insertion). Plan A vs B val-MAE was a wash; the 3× data didn't improve average prediction quality but reduced outlier failures.

**Plan C trained successfully but was not compose-evaluated.** Decision on whether to invest the deploy-adapter time deferred to user.

---

## Phase 1 — Data collection (✅ complete)

| Metric | Value |
|---|---|
| Episodes added | 200 (101–300) |
| Final dataset | **300 episodes / 179,095 frames** |
| Wall clock | 270 min (4 h 30 m) |
| Drops | 0 |
| Dataset size on disk | 1.7 GB |

Rate decay observation: started at 1.50 ep/min, decayed to ~0.52 ep/min mid-run, partially recovered to 0.86 ep/min near completion. Per-episode capture time stayed normal (~40 s); slowdown was in the inter-episode gap (Gazebo physics state accumulation across resets). No process death, no errors.

---

## Phase 2 — Training (✅ complete)

| Metric | Value |
|---|---|
| Train slice | episodes [0..254] (255 eps, 151,891 frames) |
| Val holdout | episodes [255..299] (45 eps, 27,204 frames) |
| Steps | 40,000 (vs Plan A's 50,000 — bumped down based on Plan A's 50k regression) |
| Final loss | 0.035 (vs Plan A's 0.025) |
| Final grad norm | 4.201 (vs 3.251) |
| Wall clock | 44 m 44 s |
| Step rate | 14.9 step/s (steady) |
| Output dir | `outputs/train/act_aic_v1_planb` |
| W&B | aic-act-plan-b @ RebisVla |

Final loss higher than Plan A is expected — Plan A overfit harder on its smaller dataset.

---

## Phase 3 — Per-dim val MAE (✅ complete)

### Plan B on its own held-out [255..299]

| step | lin_mean (mm/s) | ang_mean (deg/s) | best on dims |
|---|---|---|---|
| 10k | 1.678 | 0.090 | — |
| 20k | 1.336 | 0.067 | lin_x, ang_y |
| 30k | 1.443 | 0.073 | ang_z |
| **40k** | **1.329** | **0.065** | lin_y, lin_z, ang_x — winner |

Step **40k recommended for deployment**. Same conclusion as Plan A's analysis (40k > 50k).

### Apples-to-apples: Plan A vs Plan B at step 40k on the same val [255..299]

| Plan | lin_mean (mm/s) | ang_mean (deg/s) |
|---|---|---|
| **Plan A** 40k | **1.272** ↓ | 0.073 |
| **Plan B** 40k | 1.329 | **0.065** ↓ |

**Net: indistinguishable.** Plan A wins on linear by 4%; Plan B wins on angular by 11%. No directional improvement from 3× data. This contradicts the Plan B hypothesis ("more data = better policy").

---

## Phase 4 — Compose eval (✅ complete)

**Plan B compose total: 112.90 — 2.57× over Plan A's 43.89.**

| Metric | Plan A | Plan B step 40k | Δ |
|---|---|---|---|
| **Compose total score** | **43.89** | **112.90** | **+157%** |
| Trial 1 | 42.58 | 33.67 | -21% |
| Trial 2 | 36.30 | 43.54 | +20% |
| Trial 3 | -35.00 (collision) | **35.69** (clean) | **+70 pts** |
| Path length avg | 0.00 m | 0.00 m | unchanged |
| Final dist trial 1 | 0.05 m | 0.07 m | slightly worse |
| Final dist trial 2 | 0.07 m | 0.05 m | slightly better |
| Final dist trial 3 | collision | 0.07 m | recovery |
| Contacts | 1 (trial 3) | **0** | ✅ |
| Force penalties | 0 | 0 | unchanged |

### Key insight

The 2.57× improvement is **entirely driven by trial 3**: Plan A collided with the enclosure (−35 penalty), Plan B navigates cleanly through the same OOD scene config (+35.69). The other two trials are within ±20%.

**The model's per-trial behavior is still "tiny actions, no real navigation"** (path length 0.00 m on all trials). But Plan B is now *robust* — it doesn't fail catastrophically on OOD geometry. This is a different kind of improvement than expected: it's not better at the task, it's better at not screwing up.

This also reconciles the val-MAE wash: Plan A and Plan B make similar predictions on average (val MAE indistinguishable), but Plan B has fewer outlier failures. With 3× the training data, the OOD scene that pushed Plan A into collision is now closer to the in-distribution manifold.

### Tier 3 details

- Trial 1: tier_3 = 16.08 ("No insertion. Final plug port distance: 0.07m.")
- Trial 2: tier_3 = 25 ("No insertion. Final plug port distance: 0.05m.") — 25 is the partial-insertion bonus floor
- Trial 3: tier_3 = 17.09 ("No insertion. Final plug port distance: 0.07m.")

**No trial achieved full insertion (would have been +75 each).** The gripper still doesn't drive the cable into the port.

---

## Recommendation

**Ship Plan B** — total > 60 (the prior decision threshold). It's a 2.57× improvement and avoids the collision-on-OOD failure that was Plan A's main risk.

Earlier user direction was: *"Even if current option scores more than 50 run plan B and then Plan C to get a comparison of all three approaches."* So Plan C (Diffusion Policy on the same 300-ep dataset) is still expected as the next experiment.

Things Plan C could meaningfully change:
- Multi-modal action distribution → may avoid the regression-to-tiny-actions pathology
- Different inductive bias → may produce non-zero path lengths (actually drive the gripper)
- Same data but a different fitting hypothesis — clean comparison

Things Plan C is unlikely to change:
- If the data is already linear-separable for the prediction task (which val-MAE wash suggests), Diffusion may give similar predictions
- Compute cost: similar wall clock to Plan B (~45 min training)

---

## Recommendation

1. **Deploy Plan B step-40k** and run compose eval to confirm whether the val-MAE wash translates to a compose-score wash or not.
2. **If Plan B compose score ≤ Plan A's 43.89** (likely): pivot to **Plan C (Diffusion Policy)** on the same 300-ep dataset. Different inductive bias (multi-modal action distribution) may avoid the "regression to small actions" pathology that capped Plan A.
3. **If Plan B compose score > 50**: the val-MAE was misleading; the angular MAE improvement does help last-mile alignment. Submit Plan B.
4. **Don't burn another data-collection cycle** until we know if architecture is the bottleneck.

## Cost so far

- ~4.5 hr data collection
- ~0.75 hr training
- ~0.1 hr eval
- ~5.5 hr total wall clock for Plan B

## Files (Plan B)

- Plan B checkpoint: `/home/saivemu/code/aic-train/outputs/train/act_aic_v1_planb/checkpoints/last/pretrained_model`
- Plan B per-dim MAE: `/tmp/plan_b_per_dim_mae.json`
- Plan A on Plan B val: `/tmp/plan_a_on_planb_val.json`
- Train log: `/tmp/aic_train_planb.log`
- Recorder log: `/tmp/aic_record.log`
- Val episode list: `/tmp/aic_planb_val_episodes.txt`
- HF: https://huggingface.co/StrivingBapan/aic_act_v1_planb_300ep

---

## Phase 5 — Plan C (Diffusion Policy) training (✅ complete)

**Trained on the same 300-ep dataset, same train/val split as Plan B.** Different architecture (DiffusionPolicy with U-Net denoiser, MIN_MAX action normalization, n_obs_steps=2, horizon=16, n_action_steps=8 — all defaults).

| Metric | Plan B (ACT) | Plan C (Diffusion) |
|---|---|---|
| Params | 52M | **271M** (5.2×) |
| Optimizer LR | 1e-5 | 1e-4 (default) |
| Step rate | 14.9 step/s | 10.8 step/s |
| Wall clock | 44 m | **62 m** (1.4×) |
| Final loss | 0.035 (L1) | 0.000 (ε-MSE) — *different scales, not directly comparable* |
| GPU mem | 2.3 GB | 7.2 GB |
| W&B | aic-act-plan-b | aic-dp-plan-c (run u4vwif7b) |
| Steps | 40k | 40k |

Loss curve: monotonic decrease from 1.9 → 0.000 across 40k steps. The diffusion run looks healthy by every training-side metric.

### Per-dim val MAE evolution (held-out [255..299])

| step | lin_mean (mm/s) | ang_mean (deg/s) |
|---|---|---|
| 10k | 6.74 | 0.853 |
| 20k | 5.66 | 0.778 |
| 30k | 4.83 | 0.299 |
| **40k** | **3.44** | **0.239** |

Best step = 40k on every dimension. Clear monotonic improvement across training; no plateau / regression seen yet (could benefit from longer training).

### ⚠️ Eval methodology caveat — Plan C val MAE is biased

The eval script (`aic_utils/lerobot_robot_aic/scripts/eval_checkpoints.py`) measures per-dim MAE by:
1. Iterating the val dataloader (one frame per step)
2. Calling `policy.predict_action_chunk(batch)` to get the action chunk
3. Comparing the first slot of the chunk to the ground-truth action

This works cleanly for ACT (n_obs_steps=1, stateless predict_action_chunk). For diffusion (n_obs_steps=2, queue-based), the script fakes 2 timesteps by **duplicating the single observation** along the n_obs_steps dim. The diffusion model was trained with two distinct timesteps (current + previous) so it has never seen `obs[t] == obs[t-1]` during training — this is out-of-distribution input that likely degrades predictions.

**The 2.6× linear / 3.7× angular gap vs Plan B should NOT be read as "Plan C is worse than Plan B."** It's "Plan C's val MAE in this measurement setup is worse" — a measurement artifact, not necessarily a model-quality difference. The fair comparison requires either:
1. Sequential per-episode eval (use real obs[t-1] from the episode trajectory)
2. Compose eval (the actual ground-truth metric)

### Plan C deployment status — DEFERRED

`RunACT.py` is hardcoded to ACT (imports `ACTPolicy`, `ACTConfig` directly; manual MEAN_STD normalization). Deploying Plan C requires:

1. Policy-type dispatch on config.json's `type` field (~10 LoC, easy)
2. MIN_MAX normalization for state and action paths (instead of MEAN_STD) (~20 LoC, easy)
3. **n_obs_steps=2 obs queue maintenance**: store the previous tick's obs and stack it with the current one before each call (~30 LoC, moderate — needs careful handling of the very-first tick when no previous obs exists)
4. **n_action_steps=8 chunk replay**: ACT calls inference every tick (n_action_steps=1); diffusion expects to run inference every 8 ticks and replay the cached chunk in between. The control loop's "last_target_pose" anchor + 2cm offset clamp logic needs to be threaded through this (~50 LoC, moderate)
5. **100-step DDPM denoising per inference** is slower than ACT's single forward (estimate ~200ms vs ~10ms). Not a blocker at 20Hz control with n_action_steps=8 (one inference per 400ms), but verify before submitting.

**Total: ~110 LoC + testing, ~1hr engineering.**

### Cost-benefit on Plan C deployment

- **Cost**: 1 hr deploy adapter + ~5 min build + ~5 min compose eval
- **Best case**: Plan C compose score > 112.90 (Plan B) → diffusion's multi-modal output actually drives the gripper, breaks the 0.00 m path-length pathology, achieves real insertion (+75 tier_3 bonus per trial)
- **Worst case**: Plan C compose score similar to Plan A's 43.89 or worse → diffusion's predictions in deploy don't outperform ACT, val-MAE was honest

### Recommendation

**Three options for the user:**

1. **Build the diffusion deploy adapter and run Plan C compose eval.** Most informative outcome — closes the three-way comparison cleanly. ~1hr work + ~10 min eval.
2. **Skip Plan C compose, ship Plan B as final.** Plan B's 112.90 is already the daily target, and the val-MAE biased-against-diffusion comparison can't be definitively resolved without compose data. Lowest cost.
3. **Fix the val-MAE eval methodology first** (sequential per-episode, real obs[t-1]) and re-run; if Plan C still loses, skip deploy. ~30 min eval-fix + 10 min re-run.

## Files (Plan C)

- Plan C checkpoint: `/home/saivemu/code/aic-train/outputs/train/dp_aic_v1_planc/checkpoints/last/pretrained_model`
- Plan C per-dim MAE: `/tmp/plan_c_per_dim_mae.json`
- Train log: `/tmp/aic_train_planc.log`
- W&B run: https://wandb.ai/RebisVla/aic-dp-plan-c/runs/u4vwif7b

## Cost summary

- Plan A: ~5 hr (already done before this session)
- Plan B: ~5.5 hr (data 4.5 + train 0.75 + eval 0.25)
- Plan C: ~1.1 hr (train 1.0 + eval 0.1)
- Plan C deploy (deferred): ~1 hr engineering + ~0.25 hr build/eval if user opts in
- Total elapsed: ~6.6 hr for Plans B + C training/eval (excludes Plan A which was done previously)
