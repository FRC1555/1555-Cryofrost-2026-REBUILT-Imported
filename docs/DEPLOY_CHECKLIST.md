# Deploy Checklist

## Before You Build

- Open `deploy-ready/COMPETITION_QUICK_SHEET.md` if you need the short event-day version.
- Confirm you are on the correct branch.
- Confirm the robot is the version you expect for this code.
- Confirm `vendordeps/` is present and current.
- Confirm the selected auto names in PathPlanner match the ones in `RobotContainer.java`.
- Confirm any CAN ID changes in `Constants.java` match the real robot wiring.

## Before You Deploy

- Open `RobotContainer.java` and verify controller bindings look correct.
- Open `Constants.java` and verify drivetrain and hardware constants match the robot.
- Open `src/main/deploy/pathplanner/autos/` and verify the intended auto exists.
- Open `src/main/deploy/pathplanner/paths/` and verify the intended path exists.
- Build the project and fix any errors before trying to deploy.

## On the Driver Station

- Verify the correct auto shows up in the chooser.
- Verify gyro reset works.
- Verify shooter, intake, and conveyor controls respond correctly.
- Verify the vision camera is online if vision is part of the test.

## Before a Match

- Rebuild after any last-minute code change.
- Reconfirm auto timing and path selection.
- Reconfirm battery, radio, and Driver Station connection.
- Do not deploy from `main` or `Working-Branch` unless the team intentionally wants those branches.
