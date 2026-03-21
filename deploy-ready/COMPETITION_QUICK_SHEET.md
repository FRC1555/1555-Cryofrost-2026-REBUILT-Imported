# Competition Quick Sheet

This folder is the student fast-access kit for the `codex/AI-Modified` branch.

## Open These First

1. `deploy-ready/java/frc/robot/RobotContainer.java`
2. `deploy-ready/java/frc/robot/Constants.java`
3. `deploy-ready/pathplanner/autos/Blue Auto Shot.auto`
4. `deploy-ready/pathplanner/autos/Red Auto Shot.auto`
5. `deploy-ready/pathplanner/paths/Blue Auto.path`

## What Each File Does

- `RobotContainer.java`
  Controller bindings, named commands, and auto chooser wiring.
- `Constants.java`
  Robot ports, drivetrain numbers, and tuning-related constants.
- `DriveSubsystem.java`
  Swerve driving, odometry, and PathPlanner integration.
- `VisionSubSystem2026Rebuilt.java`
  Vision target info and AprilTag pose updates.
- `ShooterSubsystem.java`
  Shooter control.
- `IntakeSubsystem.java`
  Intake roller and arm control.
- `ConveyorBeltSubsystem.java`
  Conveyor control and feed behavior.
- `pathplanner/autos/*.auto`
  Which actions run in autonomous.
- `pathplanner/paths/*.path`
  Where the robot drives in autonomous.

## When You Are In A Hurry

- If controls are wrong, check `RobotContainer.java`.
- If hardware IDs or robot measurements are wrong, check `Constants.java`.
- If auto actions are wrong, check the `.auto` files.
- If auto driving is wrong, check the `.path` file and `DriveSubsystem.java`.
- If vision is wrong, check `VisionSubSystem2026Rebuilt.java`.

## Do Not Forget

- Build from the repo root, not from inside `deploy-ready`.
- This folder is a curated mirror for quick access.
- The real build still uses the files under `src/` and `vendordeps/`.

## Current Match Autos

- `Blue Auto Shot`
- `Red Auto Shot`
