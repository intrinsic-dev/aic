# Scoring

Each trial is scored using a tiered scoring system. Scores are cumulated across all trials.

## Scoring Tiers Overview

| Tier | Name | Description |
|------|------|-------------|
| Tier 1 | Model Validity | Prerequisite check that submission loads and runs |
| Tier 2 | Performance & Convergence | Quantitative metrics for motion quality |
| Tier 3 | Cable Insertion | Primary objective - successful or partial insertion verified |

## Tier 1: Model Validity (Prerequisite)

A sanity check to ensure the submission loads and runs without errors.

- The model must be able to successfully activate the submitted policy and respond to the `InsertCable` action request. The submitted policy must also send valid commands to the robot arm controller via `MotionUpdate` (target position/velocities) or `JointMotionUpdate` (target joint states).
- The policy must comply with all behavioral requirements defined in [Challenge Rules](./challenge_rules.md#aic_model)
- Submissions failing this check will not be scored

| Outcome | Score |
|---------|-------|
| Validation passed | 1 |
| Validation failed | 0 |

## Tier 2: Performance & convergence

Quantitative metrics measuring the quality of the robot's motion during task execution.

### Trajectory smoothness (0-5 points)

Measures the smoothness of the end effector trajectory. Lower jerk values indicate smoother, more controlled motion. Jerk is only accumulated when the arm is moving (speed > 0.01 m/s), so stationary periods do not dilute the average.

- **Metric**: Time-weighted average of linear jerk magnitude (m/s³)
- **Scoring**: Inversely proportional to jerk
  - Jerk = 0 m/s³ → 5 points (maximum)
  - Jerk ≥ 25,000 m/s³ → 0 points (minimum)
  - Linear interpolation between thresholds

### Task duration (0-10 points)

Rewards faster task completion. Only awarded if either
* the task is completed successfully, or
* the final position of the plug is within close proximity to the target port (Tier 3 score > 0). The max acceptable distance is set to half of the distance between the initial position of the plug and the target port.

- **Metric**: Elapsed time from task start to task end
- **Scoring**: Inversely proportional to duration
  - Duration ≤ 5 seconds → 10 points (maximum)
  - Duration ≥ 60 seconds → 1 point (minimum)
  - Linear interpolation between thresholds
- **Not awarded**: 0 points if the final position of the plug is outside the max acceptable distance of the target port (Tier 3 score <= 0).

### Trajectory efficiency (0-5 points)

Measures the total distance traveled by the end effector during task execution.
Shorter, more direct paths score higher.

- **Metric**: Cumulative Euclidean distance of end-effector positions (meters)
- **Scoring**: Inversely proportional to total path length
  - Path length ≤ initial plug-port distance → 5 points (maximum)
  - Path length ≥ 1 m + initial plug-port distance → 0 points (minimum)
  - Linear interpolation between thresholds
- The minimum path length (for a perfect score) is set dynamically to the initial Euclidean distance between the plug and port at the start of the trial

### Insertion force penalty (0 to -10 points)

Penalizes excessive force during the insertion process to encourage gentle manipulation.
The force sensor reading is tared at startup, so the baseline is close to 0 N.

- **Force threshold**: 20 N
- **Duration threshold**: 1 second
- **Penalty**: -10 points if force exceeds threshold for longer than the duration threshold
- **No penalty**: If no excessive force is detected or excessive force is within duration threshold

### Off-Limit contact penalty (0 to -20 points)

Penalizes collisions with restricted areas of the environment (enclosure or task board).

- **Penalty**: -20 points if any contact with off-limit entities is detected
- **No penalty**: If no prohibited contacts occur

## Tier 3: Task Success

The primary objective verifying successful cable insertion. Scoring uses a two-step approach that rewards both full insertion and partial progress toward the port.

### Successful insertion (-10 to 60 points)

If the cable connector is fully inserted into the **correct** target port, verified via contact sensors:

| Outcome | Score |
|---------|-------|
| Correct port insertion | 60 |
| Wrong port insertion | -10 |

### Partial insertion and proximity (0-40 points)

When full insertion is not detected, the score is based on how close the plug is to the port at task completion:

- **Partial insertion** (30-40 points): If the plug is inside a bounding box between the port entrance and the bottom of the port (within a 5 mm x-y tolerance), the score is proportional to insertion depth. Deeper insertion scores higher.
- **Proximity** (0-20 points): If the plug is not inside the port, the score is inversely proportional to the max acceptable distance from the port. The max distance is set to half the distance between the initial position of the plug and the port.
  - At the port entrance → 20 points (maximum)
  - Outside of max distance → 0 points (minimum)
  - Linear interpolation between thresholds

## Total Score Calculation

```
Total Score = Tier 1 + Tier 2 + Tier 3
```

Where:
- **Tier 1**: 0 or 1 point
- **Tier 2**: Sum of smoothness, duration, efficiency, and penalties
- **Tier 3**: Insertion success bonus, or partial insertion / proximity score

## Final Ranking

The final ranking is determined by cumulating scores across all trials, combining the quantitative performance metrics from Tier 2 and the task success score from Tier 3.

## See Also

For reproducible examples that exercise each scoring tier, see the [Scoring Test & Evaluation Guide](./scoring_tests.md).
