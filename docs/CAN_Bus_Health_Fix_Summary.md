# CAN Bus Health Fix — Summary

## Context
Switched from **Shuffleboard** (old dashboard software) to **Elastic** (new dashboard software). Elastic natively reads roboRIO CAN status via NetworkTables for the bus health indicator, which revealed the persistent CAN errors.

## Problem
Elastic dashboard showed CAN bus health as **red** — constant CAN errors on multiple IDs.

## Root Causes & Fixes

### 1. SparkMax-vs-SparkFlex Type Mismatch (CAN 2, 3, 4, 13, 14)
**Log errors:**
```
CANSparkMax object created for CAN ID 3, which is not a SPARK MAX
[CAN SPARK] IDs: 3, 4, Unable to retrieve SPARK firmware version
[CAN SPARK] IDs: 3, 4, 14, WPILib or External HAL Error: CAN: Message not found
```
**Fix:** Changed `SparkMax` → `SparkFlex` in 4 files across two projects:

| File | Change |
|------|--------|
| `1555-Cryofrost-2026-REBUILT-Imported\subsystems\ConveyerBeltSubSystem.java` | CAN 3: `SparkMax` → `SparkFlex` |
| `1555-Cryofrost-2026-REBUILT\subsystems\CoralSubsystem.java` | CAN 2,3,4: `SparkMax` → `SparkFlex`, sim classes too |
| `1555-Cryofrost-2026-REBUILT\subsystems\AlgaeSubsystem.java` | CAN 13,14: `SparkMax` → `SparkFlex` (was already importing SparkFlex but not using it) |
| `1555-Cryofrost-2026-REBUILT\Configs.java` | `SparkMaxConfig` → `SparkFlexConfig` for Coral & Algae |

**Result:** CAN 2, 3, 4, 13, 14 errors are gone.

**Same issue still present on CAN 5 and CAN 6 (Front Left Module):**
```
CANSparkMax object created for CAN ID 5, which is not a SPARK MAX
CANSparkFlex object created for CAN ID 6, which is not a SPARK Flex
```
**Cause:** The devices are physically correct — CAN 5 *is* a Spark MAX (turning, square box) and CAN 6 *is* a Spark Flex (driving, attached to NEO). The "not a SPARK X" errors are a **symptom of the bad CAN connection** on that module, not a type swap. When CAN communication is degraded enough, the device identification handshake fails, producing these misleading warnings.

### 2. CAN 6 (Front Left Drive) Still Timing Out
**Current error:**
```
[Spark Flex] IDs: 6, timed out while waiting for Period Status 5: HAL: CAN Receive has Timed Out
```
**Status:** Unresolved — physical CAN wiring issue. CAN 5 briefly timed out during wiggle test but recovered. CAN 6 never recovered throughout a 3.5-min session.

**CAN chain:** roboRIO → ... → CAN 5 (Spark MAX) → CAN 6 (Spark Flex) → PDU (end)

**Checklist:**
- ✅ CAN 6 works via USB (REV Hardware Client can control it)
- ✅ PDU CAN termination is ON
- ✅ CAN_H/CAN_L voltage normal (~2.5 V idle, ~2.0 V differential active)
- ❌ CAN 6 timed out consistently 100% of the time
- ❌ CAN connector on CAN 6 Spark Flex is inaccessible (screwed into frame)
- ❌ Wiggle test at CAN 6 connector briefly disturbed CAN 5 but didn't change CAN 6 behavior

**Suspect:** Faulty CAN transceiver on the CAN 6 Spark Flex, or internal connector pin not mating. Swap CAN 6 with CAN 12 (Rear Left Drive) to confirm if fault follows the device.

### 3. PhotonVision Camera Not Connected
**Log error:**
```
Could not find any PhotonVision coprocessors on NetworkTables
```
**Status:** REM'd out — `VisionSubSystem2026Rebuilt` import and field commented in `RobotContainer.java`, null guard added in `DriveSubsystem.java`. Re-enable when the coprocessor is back online. Latest log (12:59 PM) confirms no PhotonVision errors with REM'd code.

### 4. Code Documentation Added (May 31, 2026)
Added class-level Javadoc and CAN ID documentation across the entire project. All 21 Java files reviewed; 15 received new or improved documentation.

| File | Documentation Added |
|------|--------------------|
| `subsystems/IntakeSubsystem.java` | CAN IDs 16 (arm pivot), 17 (rollers); direction hints |
| `subsystems/TransferSubSystem.java` | Purpose ("moves note from intake to shooter"), CAN ID 2 |
| `subsystems/ConveyerBeltSubSystem.java` | Purpose ("assists feeding"), CAN ID 3 |
| `subsystems/ShooterSubsystem.java` | Already well-documented (no changes) |
| `subsystems/DriveSubsystem.java` | Already well-documented from REV template |
| `subsystems/MAXSwerveModule.java` | Already well-documented from REV template |
| `subsystems/VisionSubSystem2026Rebuilt.java` | Class-level: camera "RightCAM", dead code, REM'd status |
| `commands/AutoIntakeOut.java` | **WARNING**: infinite `while(true)` loop — hangs robot if called |
| `commands/AutoConveyerOut.java` | **WARNING**: infinite `while(true)` loop — hangs robot if called |
| `commands/AutoIntakeDown.java` | Factory class pattern, arm direction note |
| `commands/AutoIntakeUp.java` | Same |
| `commands/AutoIntakeIn.java` | Same |
| `commands/AutoConveyerIn.java` | Same + extends Command note |
| `commands/AutoShoot.java` | Example of correct pattern (plain class, no `extends Command`) |
| `commands/AlignToTargetCommand.java` | All commented out (noted) |
| `RobotContainer.java` | Controls overview, vision REM'd, fixed typos |
| `Constants.java` | Full CAN ID map comment at top |
| `Configs.java` | Class-level: config presets, old Coral/Algae stubs |
| `HubShiftUtil.java` | Class-level: match shift scheduling strategy |

## Deployment
Deployed the fixed code from `1555-Cryofrost-2026-REBUILT-Imported` (GradleRIO 2026.2.1, remote `https://github.com/FRC1555/1555-Cryofrost-2026-REBUILT-Imported.git`) to roboRIO at 10.15.55.2.

The original `1555-Cryofrost-2026-REBUILT` project (GradleRIO 2025) is incompatible with the roboRIO's 2026 v1.2 image — both were fixed but only the Imported project can deploy currently.

## Branch
Renamed `Working-Branch` → `26OffSeasonCode` (commit `4b2b4b7`). All work going forward is on `26OffSeasonCode` until the team physically renames it.

## Outstanding Issues
- **CAN 6 (Front Left Drive)** still timing out — CAN voltage normal, connector inaccessible behind screws. Swap with CAN 12 or replace Spark Flex.
- **CAN 5 (Front Left Turning)** had brief timeouts during wiggle test — recovered. Monitor.
- **PhotonVision** REM'd out — re-enable when coprocessor is back online.
- **`extends Command` pattern** on 4 factory classes (`AutoIntakeIn`, `AutoIntakeDown`, `AutoIntakeUp`, `AutoConveyerIn`) is misleading — these are factory classes, not command overrides. `AutoShoot` is the correct pattern (plain class). Compiles fine, low priority.
- **`AutoIntakeOut.TransferOut()`** and **`AutoConveyerOut.IntakeOutSystem()`** both have infinite `while(true)` loops with no return — will hang the robot if called.
