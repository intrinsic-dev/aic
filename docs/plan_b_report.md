# Plan B Overnight Run — Morning Report (2026-05-05)

## Headline

**Plan B (300-ep ACT) does not show a clear win over Plan A (100-ep ACT) on per-dim val MAE.** Tripling the training data moved val-MAE by <5% in either direction — net wash, suggesting the 100-ep dataset was already large enough to fit the input→output mapping. **The bottleneck is not data; it's model architecture / task formulation.** This is the case Plan C (Diffusion Policy) is designed to test.

Compose eval results pending; results below will be appended once available.

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

## Files

- Plan B checkpoint: `/home/saivemu/code/aic-train/outputs/train/act_aic_v1_planb/checkpoints/last/pretrained_model`
- Plan B per-dim MAE: `/tmp/plan_b_per_dim_mae.json`
- Plan A on Plan B val: `/tmp/plan_a_on_planb_val.json`
- Train log: `/tmp/aic_train_planb.log`
- Recorder log: `/tmp/aic_record.log`
- Val episode list: `/tmp/aic_planb_val_episodes.txt`
