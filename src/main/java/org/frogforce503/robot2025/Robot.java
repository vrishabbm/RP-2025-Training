// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.frogforce503.robot2025;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import java.lang.reflect.Field;

import org.frogforce503.robot2025.fields.FieldConfig;
import org.frogforce503.robot2025.fields.FieldConfig.VENUE;
import org.frogforce503.robot2025.hardware.RobotHardware;
import org.frogforce503.robot2025.hardware.RobotHardwareCompBot;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private static final double loopOverrunWarningTimeout = 0.2;


  private static Timer timer = new Timer();
  public static RobotHardware bot;
  private RobotContainer robotContainer;
  
  /*
   * Robot Constructor 
   */
  public Robot() {
    RobotStatus.getInstance().setCurrentRobot(RobotStatus.Bot.SimBot);

    bot = switch (RobotStatus.getInstance().getCurrentRobot()) {
      case CompBot -> new RobotHardwareCompBot();
      default -> new RobotHardwareCompBot();
    };
  }
 
 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Commands.waitSeconds(20).andThen(Commands.runOnce(()->Threads.setCurrentThreadPriority(true, 10))); Fix #2 ALSO CHECK TELEOPINIT
    Logger.recordMetadata("ProjectName", "FF2025_" + RobotStatus.getInstance().getCurrentRobot().name().toUpperCase()); // Set a metadata value

    Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables

    if (isReal()) { // RUNNING ON ROBORIO
        Logger.addDataReceiver(new WPILOGWriter());
    } else { // REPLAY
        setUseTiming(false); // Run as fast as possible
        // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
        // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
        // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    }

    FieldConfig.getInstance().setVenue(VENUE.GIRLS_COMP);

    robotContainer = new RobotContainer();
    Logger.start();

    // Adjust loop overrun warning timeout
    try {
      Field watchdogField = IterativeRobotBase.class.getDeclaredField("m_watchdog");
      watchdogField.setAccessible(true);
      Watchdog watchdog = (Watchdog) watchdogField.get(this);
      watchdog.setTimeout(loopOverrunWarningTimeout);
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to disable loop overrun warnings.", false);
    }

    // Disable alerts for disconnected controllers
    DriverStation.silenceJoystickConnectionWarning(true);

    // Configure brownout voltage
    RobotController.setBrownoutVoltage(6.0);

    // Switch thread to high priority to improve loop timing
    // Threads.setCurrentThreadPriority(true, 1);
  }

  @Override
  public void robotPeriodic() {
    // SignalLogger.enableAutoLogging(false); // use and see if works
    CommandScheduler.getInstance().run();
    robotContainer.periodic();
  }

  @Override
  public void autonomousInit() {
    // RobotContainer.intake.idle();
    robotContainer.autoChooser.startAuto();

    timer.restart();
  }

  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("AUTON TIMER ELAPSED", timer.get());
  }

  @Override
  public void teleopInit() {

  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {
    // robotContainer.drive.brake();
    // robotContainer.drive.coastOut();
  }

  @Override
  public void disabledPeriodic() {
    // robotContainer.autoChooser.periodic();
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}