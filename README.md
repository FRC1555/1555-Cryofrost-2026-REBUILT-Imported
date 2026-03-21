# FRC 1555 Cryofrost 2026

This branch is organized so students can find the robot code they need to edit and deploy without digging through review leftovers or template files.

## Start Here

For normal robot work, the files that matter most are:

- `src/main/java/frc/robot/Robot.java`
- `src/main/java/frc/robot/RobotContainer.java`
- `src/main/java/frc/robot/Constants.java`
- `src/main/java/frc/robot/subsystems/`
- `src/main/java/frc/robot/commands/`
- `src/main/deploy/pathplanner/autos/`
- `src/main/deploy/pathplanner/paths/`
- `vendordeps/`

Use [docs/DEPLOY_CHECKLIST.md](docs/DEPLOY_CHECKLIST.md) before practice, testing, or competition deployment.

Use [docs/REPO_MAP.md](docs/REPO_MAP.md) when you need to know where to make a change.

## Current Autos

- `Blue Auto Shot`
- `Red Auto Shot`

These autos use PathPlanner named commands registered in `RobotContainer.java`.

## Deploy Workflow

1. Open the project in WPILib VS Code.
2. Make sure you are on the `codex/AI-Modified` branch if you want the cleaned AI version.
3. Review [docs/DEPLOY_CHECKLIST.md](/C:/Users/bsope/source/1555-Cryofrost-2026-REBUILT-Imported/docs/DEPLOY_CHECKLIST.md).
4. Build with `gradlew.bat build` or use WPILib build/deploy.
5. Deploy to the robot.

## Notes

- `main` and `Working-Branch` were intentionally left untouched.
- AI branch notes live in [AI/START_HERE.md](AI/START_HERE.md).
