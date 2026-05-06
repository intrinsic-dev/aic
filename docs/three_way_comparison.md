# Three-way comparison — Plans A, B, C (2026-05-05)

## Headline

| Plan | Architecture | Data | Val MAE (lin mm/s) | Val MAE (ang deg/s) | Compose score |
|---|---|---|---|---|---|
| **A** | ACT 50k step | 100 ep | 1.272 | 0.073 | **43.89** |
| **B** | ACT 40k step (val-split) | 300 ep | 1.329 | 0.065 | **112.90** ✅ shipping |
| **C** | Diffusion 40k step | 300 ep (same as B) | 3.44 ¹ | 0.239 ¹ | **86.03** |

¹ *Plan C val MAE is methodologically biased — see Phase 5 caveat. The ACT vs Diffusion val comparison is not apples-to-apples in this eval.*

**Final ranking: B > C > A.** Plan B wins (112.90); Plan C beats A by 1.96× but loses to B by 23%. The difference between B and C is **trial 3 alone**: Plan B navigates the OOD geometry to 0.07 m (35.69 score); Plan C fails to drive the cable to within the bounding radius and scores 1.00 (tier_1 only). Trials 1 and 2 are essentially tied between B and C (~within 5%).

**Lessons:**

1. **Architecture diversity didn't help here.** Diffusion's multi-modal action distribution didn't break the path-length-0.00m pathology that capped Plans A and B — Plan C also had path length 0.00 m on trials 1 and 2. The "regression to small actions" hypothesis was the right diagnosis, but Diffusion didn't fix it on this dataset.
2. **Plan C's val-MAE bias was real but didn't invert the ranking.** Val MAE said C was 2.6× worse than B; compose said C is 23% worse. The duplicate-obs hack hurt Plan C in val MAE but the gap in actual task performance is much smaller.
3. **The trial 3 OOD geometry remains the discriminator.** It collides Plan A (–35), navigates cleanly with Plan B (+35.69), and is missed entirely by Plan C (1.00). The 3× data + ACT formulation seems to have learned the right inductive bias for this scene; diffusion did not, despite seeing the same training data.

**Shipped: Plan B step-40k.** Plan C is preserved on disk and on the new branch but not deployed.

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

---

## Phase 6 — Plan C compose eval (✅ complete)

**Plan C compose total: 86.03.** Beats Plan A (43.89) by 1.96×; loses to Plan B (112.90) by 23%.

| Trial | Plan A | Plan B | Plan C | Diff (C vs B) |
|---|---|---|---|---|
| 1 | 42.58 | 33.67 | **41.32** | C +23% |
| 2 | 36.30 | 43.54 | **43.71** | C +0.4% |
| 3 | -35.00 (collision) | 35.69 (clean) | **1.00** | C –97% |
| **Total** | **43.89** | **112.90** | **86.03** | **C –23.8%** |

### Per-trial details for Plan C

- **Trial 1**: tier_1=1, tier_2=17.70, tier_3=22.62. Cable ended 0.05 m from port. Path length 0.00 m. No contacts.
- **Trial 2**: tier_1=1, tier_2=17.71, tier_3=25.00. Cable ended 0.04 m from port. Path length 0.00 m. No contacts.
- **Trial 3**: tier_1=1, tier_2=**0**, tier_3=**0**. Cable ended **0.19 m** from port — *outside the bounding radius*, so no tier_2 or tier_3 credit awarded ("Plug is not within max bounding radius from target port").

The headline finding: **Plan C beats Plan B on trials 1+2 (slightly) and is dramatically worse on trial 3 OOD geometry.**

### Diagnosis

Plan B's win comes entirely from trial 3. Plan A collides into the enclosure at trial 3 (–35). Plan B navigates the cable cleanly to within the bounding radius (+35.69). Plan C — same training data, same number of steps — fails to drive the cable close enough to the port to get any tier_2 or tier_3 credit. Path length still 0.00 m on all three trials, same pathology as A and B; the OOD difference is whether the model "stays still and lets physics settle the cable into a productive position" (B) or "stays still and the cable drifts elsewhere" (C).

The val-MAE methodology was biased against Plan C (n_obs_steps=2 fake stacking), but compose shows the real ranking is **closer than val MAE suggested**:
- Val MAE (biased): C is 2.6× worse than B on linear, 3.7× worse on angular
- Compose (truth): C is 23% worse than B overall, **tied or slightly better on in-distribution trials**

Diffusion's multi-modal action distribution did NOT break the regression-to-tiny-actions pathology (path length 0.00 m on Plan C trials 1+2). The 5× model size (271M vs 52M) and different inductive bias didn't help on this dataset.

### Engineering notes — what made the deploy adapter much smaller than estimated

Initial estimate was ~110 LoC + 1 hr. **Actual: ~30 LoC + 30 min.** Key realization: `policy.select_action()` is a polymorphic interface across ACT and Diffusion — it internally manages each architecture's queue/chunk-replay state. So the control loop in `insert_cable()` didn't need any changes. The only edits were:

1. Type-dispatch the policy class load (read `type` field from config.json) — ~15 LoC
2. Branch state and action normalization on MEAN_STD vs MIN_MAX — ~15 LoC

The `n_obs_steps=2 obs queue` and `n_action_steps=8 chunk replay` items in the original estimate were wrong — `policy.select_action()` already handles those internally.

### Build gotcha — pixi build cache

`docker compose build` initially served a stale conda package of `aic_example_policies` (built before the Diffusion edits) because pixi's build cache hashes by lockfile, not source content. Fix: added a Dockerfile RUN that overlays the COPY'd source onto site-packages after `pixi install`:

```dockerfile
RUN cp -r /ws_aic/src/aic/aic_example_policies/aic_example_policies/. \
       /ws_aic/src/aic/.pixi/envs/default/lib/python3.12/site-packages/aic_example_policies/
```

This guarantees the container always runs the COPY'd source, even when pixi caches a stale package build. Recommended pattern for future image rebuilds where conda-package source changes.

## Files (Plan C)

- Plan C checkpoint: `/home/saivemu/code/aic-train/outputs/train/dp_aic_v1_planc/checkpoints/last/pretrained_model`
- Plan C per-dim MAE: `/tmp/plan_c_per_dim_mae.json`
- Train log: `/tmp/aic_train_planc.log`
- Compose log: `/tmp/aic_compose_planc.log`
- W&B run: https://wandb.ai/RebisVla/aic-dp-plan-c/runs/u4vwif7b

## Cost summary

- Plan A: ~5 hr (already done before this session)
- Plan B: ~5.5 hr (data 4.5 + train 0.75 + eval 0.25)
- Plan C: ~1.4 hr (train 1.0 + val MAE 0.1 + deploy adapter 0.5 + compose 0.2)
- Total elapsed: ~7 hr for Plans B + C end-to-end (excludes Plan A which was done previously)
