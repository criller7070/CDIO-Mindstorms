# Calibration Test Plan (No Code Changes)

This plan helps isolate where the distance error comes from:
- EV3 drive conversion (robot/main.py)
- Camera planner px->mm conversion (tools/tools_path_planner.py)

## Safety Baseline Saved
A restorable snapshot was created here:
- backups/2026-06-08_calibration_baseline/working_tree.patch
- backups/2026-06-08_calibration_baseline/main.py
- backups/2026-06-08_calibration_baseline/commands.txt
- backups/2026-06-08_calibration_baseline/tools_path_planner.py
- backups/2026-06-08_calibration_baseline/vision_app.py
- backups/2026-06-08_calibration_baseline/vision_config.py

## Step 1: EV3-only Straight Test
1. Copy missions/commands_calibration_straight_1000.txt to robot/commands.txt.
2. Run mission once.
3. Measure actual distance in mm.
4. Compute correction factor:

k = target_mm / actual_mm

Example: target=1000, actual=1180 => k=0.847

Interpretation:
- k < 1 means robot drives too far.
- k > 1 means robot drives too short.

## Step 2: Repeatability Test
1. Copy missions/commands_calibration_repeat_500.txt to robot/commands.txt.
2. Run and measure each 500 mm leg.
3. If errors vary a lot between legs, suspect slip/mechanics.
4. If errors are stable and proportional, suspect calibration constants.

## Step 3: Turn + Geometry Check
1. Copy missions/commands_calibration_square_500.txt to robot/commands.txt.
2. Run once.
3. Check if it returns close to start pose.

Interpretation:
- Big heading error: turn scaling is off.
- Good heading but wrong side lengths: distance scaling is off.

## Step 4: Camera-to-MM Isolation
After EV3 straight distance is corrected, run one camera-generated mission and compare:
- Planned total forward mm (sum of FORWARD commands)
- Actual physical path length

If EV3 test is good but camera mission is still wrong, check field dimensions in tools/tools_path_planner.py:
- FIELD_WIDTH_MM
- FIELD_HEIGHT_MM

## Restore to Baseline
To restore exactly to pre-test state:

git apply --reverse backups/2026-06-08_calibration_baseline/working_tree.patch

If needed, manually copy backup files from backups/2026-06-08_calibration_baseline/ back to original paths.
