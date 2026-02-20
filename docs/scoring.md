# Scoring

Each trial is scored using a tiered scoring system. Scores are cumulated across all trials.

## Scoring Tiers Overview

| Tier | Name | Description |
|------|------|-------------|
| Tier 1 | Model Validity | Prerequisite check that submission loads and runs |
| Tier 2 | Performance & Convergence | Quantitative metrics for motion quality |
| Tier 3 | Task Success | Primary objective - successful cable insertion verified |

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

### Trajectory jerk (1-20 points)

Measures the smoothness of the end effector trajectory. Lower jerk values indicate smoother, more controlled motion.

- **Metric**: Time-weighted average of linear jerk magnitude (m/s³)
- **Scoring**: Inversely proportional to jerk
  - Jerk = 0 m/s³ → 20 points (maximum)
  - Jerk ≥ 25 m/s³ → 1 point (minimum)
  - Linear interpolation between thresholds

### Final plug/port distance (0.5-30 points)

Measures how close the plug is to the port at task completion, with a time multiplier rewarding faster completion.

- **Metric**: Euclidean distance between plug and port frames
- **Base score**: Inversely proportional to distance
  - Distance = 0 m → 10 points (closest)
  - Distance ≥ 1 m → 0.5 points (furthest)
  - Linear interpolation between thresholds
- **Time multiplier**: Inversely proportional to task duration
  - Duration ≤ 5 seconds → 3x multiplier
  - Duration ≥ 60 seconds → 1x multiplier
  - Linear interpolation between thresholds
- **Final score**: Base score × Time multiplier
  - Maximum: 10 × 3 = 30 points
  - Minimum: 0.5 × 1 = 0.5 points

### Trajectory efficiency (0-10 points)

Measures the total distance traveled by the end effector during task execution.
Shorter, more direct paths score higher.

- **Metric**: Cumulative Euclidean distance of end-effector positions (meters)
- **Scoring**: Inversely proportional to total path length
  - Path length ≤ initial plug-port distance → 10 points (maximum)
  - Path length ≥ 10 m → 0 points (minimum)
  - Linear interpolation between thresholds
- The minimum path length (for a perfect score) is set dynamically to the initial Euclidean distance between the plug and port at the start of the trial

### Insertion force penalty (0 to -10 points)

Penalizes excessive force during the insertion process to encourage gentle manipulation.

- **Force threshold**: 25 N
- **Duration threshold**: 1 second
- **Penalty**: -10 points if force exceeds threshold for longer than the duration threshold
- **No penalty**: If no excessive force is detected or excessive force is within duration threshold

### Off-Limit contact penalty (0 to -20 points)

Penalizes collisions with restricted areas of the environment (enclosure or task board).

- **Penalty**: -20 points if any contact with off-limit entities is detected
- **No penalty**: If no prohibited contacts occur

## Tier 3: Task Success

The primary objective verifying successful cable insertion. A model is successful if it correctly aligns and inserts the cable connector into the target port.

- **Verification**: Connection detected via contact sensors and force/torque feedback
- **Score**: Successful completion yields a 100-point bonus

| Outcome | Score |
|---------|-------|
| Connection verified | 100 |
| Connection not verified | 0 |

## Total Score Calculation

```
Total Score = Tier 1 + Tier 2 + Tier 3
```

Where:
- **Tier 1**: 0 or 1 point
- **Tier 2**: Sum of jerk score, distance score, and penalties
- **Tier 3**: Cable insertion success bonus

## Final Ranking

The final ranking is determined by cumulating scores across all trials, combining the quantitative performance metrics from Tier 2 and the task success bonus from Tier 3.

## See Also

For reproducible examples that exercise each scoring tier, see the [Scoring Test & Evaluation Guide](./scoring_tests.md).
