# Repo Map

## Main Robot Code

- `src/main/java/frc/robot/Robot.java`
  Controls robot mode transitions and starts autonomous.
- `src/main/java/frc/robot/RobotContainer.java`
  Wires subsystems, controller buttons, and PathPlanner named commands.
- `src/main/java/frc/robot/Constants.java`
  Holds robot-wide constants like drivetrain measurements and controller ports.
- `src/main/java/frc/robot/Configs.java`
  Holds shared motor-controller configuration for the active hardware.

## Subsystems

- `src/main/java/frc/robot/subsystems/DriveSubsystem.java`
- `src/main/java/frc/robot/subsystems/VisionSubSystem2026Rebuilt.java`
- `src/main/java/frc/robot/subsystems/ShooterSubsystem.java`
- `src/main/java/frc/robot/subsystems/IntakeSubsystem.java`
- `src/main/java/frc/robot/subsystems/ConveyorBeltSubsystem.java`
- `src/main/java/frc/robot/subsystems/MAXSwerveModule.java`

## Autonomous Helpers

- `src/main/java/frc/robot/commands/AutoShoot.java`
- `src/main/java/frc/robot/commands/AutoIntakeDown.java`
- `src/main/java/frc/robot/commands/AutoIntakeUp.java`
- `src/main/java/frc/robot/commands/AutoIntakeIn.java`
- `src/main/java/frc/robot/commands/AutoIntakeOut.java`
- `src/main/java/frc/robot/commands/AutoConveyorIn.java`
- `src/main/java/frc/robot/commands/AutoConveyorOut.java`

## PathPlanner Files

- `src/main/deploy/pathplanner/autos/`
- `src/main/deploy/pathplanner/paths/`
- `src/main/deploy/pathplanner/settings.json`

## Vendor Dependencies

- `vendordeps/`

## AI Branch Notes

- `AI/START_HERE.md`
- `AI/CHANGED_FILES.md`
