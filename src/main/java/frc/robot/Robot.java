// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj.simulation.BatterySim;
// import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.HubShiftUtil.ShiftInfo;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    configureDiagnosticsTab();
  }

  private void configureDiagnosticsTab() {
    // Keep electrical health widgets together for quick between-match checks.
    ShuffleboardTab diagnosticsTab = Shuffleboard.getTab("Diagnostics");

    diagnosticsTab
        .addBoolean("CAN Bus Healthy", this::isCanBusHealthy)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(0, 0)
        .withSize(2, 2)
        .withProperties(Map.of("Color when true", "Lime", "Color when false", "Red"));

    diagnosticsTab
        .addDouble("Battery Voltage", RobotController::getBatteryVoltage)
        .withWidget(BuiltInWidgets.kDial)
        .withPosition(2, 0)
        .withSize(2, 2)
        .withProperties(Map.of("Min", 8.0, "Max", 13.5));

    diagnosticsTab
        .addBoolean("Brownout Active", RobotController::isBrownedOut)
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withPosition(4, 0)
        .withSize(2, 2)
        .withProperties(Map.of("Color when true", "Red", "Color when false", "Lime"));

    diagnosticsTab
        .addDouble("CAN Utilization %", () -> getCanStatus().percentBusUtilization * 100.0)
        .withWidget(BuiltInWidgets.kDial)
        .withPosition(6, 0)
        .withSize(2, 2)
        .withProperties(Map.of("Min", 0.0, "Max", 100.0));

    diagnosticsTab
        .addString("CAN Error Summary", this::getCanErrorSummary)
        .withWidget(BuiltInWidgets.kTextView)
        .withPosition(0, 2)
        .withSize(3, 1);
  }

  private CANStatus getCanStatus() {
    return RobotController.getCANStatus();
  }

  private boolean isCanBusHealthy() {
    CANStatus canStatus = getCanStatus();
    return canStatus.transmitErrorCount == 0
        && canStatus.receiveErrorCount == 0
        && canStatus.percentBusUtilization < 0.85;
  }

  private String getCanErrorSummary() {
    CANStatus canStatus = getCanStatus();
    return String.format(
        "TX: %d  RX: %d  BUS: %.1f%%",
        canStatus.transmitErrorCount,
        canStatus.receiveErrorCount,
        canStatus.percentBusUtilization * 100.0);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
    ShiftInfo official = HubShiftUtil.getOfficialShiftInfo();   

 // Official shift info 
    SmartDashboard.putString("Shift/Official/CurrentShift",   official.currentShift().toString());
    SmartDashboard.putNumber("Shift/Official/RemainingTime", Math.round(official.remainingTime()*10)/10.0);
    SmartDashboard.putNumber("Shift/Official/ElapsedTime",   Math.round(official.elapsedTime()*10)/10.0);
      SmartDashboard.putBoolean("Shift/Official/Active",        official.active());
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    /*
     * String autoSelected = SmartDashboard.getString("Auto Selector",
     * "Default"); switch(autoSelected) { case "My Auto": autonomousCommand
     * = new MyAutoCommand(); break; case "Default Auto": default:
     * autonomousCommand = new ExampleCommand(); break; }
     */

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /**
   * This function is called periodically during simulation
   *
   * <p>This collects each subsystem's physics model's current draw to update the battery simulation
   */
  // @Override
  // public void simulationPeriodic() {
  //   // SimBattery estimates loaded battery voltages
  //   RoboRioSim.setVInVoltage(
  //       BatterySim.calculateDefaultBatteryLoadedVoltage(
  // }
}
