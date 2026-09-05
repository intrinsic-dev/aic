# AIC Offline Bag Scoring

This directory contains `batch_score_bags.py`, a standalone Python script designed to process `.mcap` bag files offline and score the insertion forces across all teams and trials.

## Prerequisites
This script requires the `rosbag2_py` and `rclpy` libraries to parse `.mcap` files natively. 

If you are using the AIC `pixi` environment (specifically on the `ac/aic-logger` branch or later), these dependencies are already installed automatically. Simply run the script via `pixi run python`.

## Usage
Provide the root folder containing the evaluation data.

```bash
cd ~/aic_ws/src/aic
pixi run python src/aic_engine/scripts/batch_score_bags.py --folder /path/to/final_evals [--team team_name] [--eval-dir-name folder_name]
```

- `--folder`: The root directory containing the data. 
- `--team`: *(Optional)* If provided, the script will only process bags belonging to the specified team. Otherwise, it processes all teams.
- `--eval-dir-name`: *(Optional)* If provided, the script will only process bags that have this string in their folder path (defaults to `final_eval`). This is useful for filtering out junk or practice bags.

The script will automatically discover all `.mcap` bags and generate a `scoring_results.csv` in the provided root folder.

## Input Folder Structure
The script assumes the data is organized in the following nested structure to correctly extract the team and trial names:

```text
<root_folder>/
└── <team_name>/
    └── final_eval/
        └── <trial_name>/
            └── <run_folder_name>/
                └── <bag_file>.mcap
```

*Note: If an `.mcap` file is found closer to the root, it will still be processed, but its Team and Trial names will be marked as "Unknown".*

## Scoring Assumptions & Logic
The script parses the necessary topics and enforces the following assumptions:

### Phase Detection
1. **Rosout Indicators**: The `aic_controller/controller_state` topic is published continuously for the entire duration of the bag. Therefore, to isolate the *actual* insertion phases, the script relies on the `/rosout` topic.
   - It marks the start of a new insertion interval when it finds a `SwitchToAICController::Execute` log.
   - If this log is missing, it falls back to using the `TareForceTorqueSensorSkill::Execute` log.
   - It explicitly notes in the CSV whether the switch was successfully found.
2. **Move Robot Ignored**: All forces recorded outside of these active insertion intervals are assumed to be during the `move_robot` phases and are completely ignored.
3. **Cable Segmentation**: Intervals are grouped in pairs to represent a single cable.
   - Interval 1 = SFP Insertion
   - Interval 2 = SC Insertion
   - *Since each bag file is assumed to contain only one cable, the script will output 1 cable row per bag file. If the bag only contains 1 interval (e.g. stopping after a failed SFP insertion), it safely leaves the SC metrics blank.*

### Force Calculation & Biasing
4. **Z-Axis Only**: Only the Z-axis force ($F_z$) is evaluated.
5. **Internal Biasing (Taring)**: To account for unpredictable sensor drift or varying controller bias, the script calculates a dynamic "tare" for **every** insertion. It averages the raw $F_z$ over the **first 0.5s** of the active interval to establish a baseline. This baseline is subtracted from the remainder of the interval to isolate the net absolute force ($|\Delta F_z|$).

### Penalty Application
6. **Thresholds**: 
   - SFP insertions have a threshold of **20.0 N**.
   - SC insertions have a threshold of **30.0 N**.
7. **Time Violation**: A violation occurs if the net absolute force ($|\Delta F_z|$) continuously exceeds the threshold for strictly **greater than 1.0 seconds**.
8. **Penalty Deductions**: 
   - A flat penalty of **-12.0 points** is applied per cable.
   - The penalty is applied exactly **once per cable**. If both the SFP and SC insertions violate their respective thresholds, the penalty is attributed to whichever end triggered it *first*, ensuring teams are not double-penalized for a single cable.

## Output Format
The resulting `scoring_results.csv` produces **one row per Cable** (yielding up to 5 rows for a fully completed 5-cable trial). It includes the following telemetry:
- `Team Name` & `Trial Name`
- `Cable Index` (1 through 5)
- `Penalty Applied` (True/False)
- `Penalty Triggered By` (SFP or SC)
- `SFP/SC Average Force (N)` (The overall average force across the entire insertion, regardless of penalty)
- `SFP/SC Peak Force (N)` (The absolute peak force hit during the insertion, regardless of penalty)
- `SFP/SC Violations` (A formatted string detailing the start, end, and average force of all contiguous >1s penalty intervals)
