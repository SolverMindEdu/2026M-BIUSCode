// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.LoggedTracer;
import java.io.File;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private Command autonomousCommand;
  private final RobotContainer robotContainer;

  /**
   * The stick if one is in, the RIO's own flash if not. AdvantageKit's default is "/U/logs" alone,
   * and with no stick present lvuser cannot create it, so the writer fails open and nothing is
   * logged. Flash holds roughly 0.5 to 1 MB per second of match, so prune /home/lvuser/logs by hand
   * whenever it is the one being used.
   */
  private static String logFolder() {
    return new File("/U").exists() ? "/U/logs" : "/home/lvuser/logs";
  }

  public Robot() {
    Logger.recordMetadata("ProjectName", "MOBIUS");
    Logger.recordMetadata("Team", "10015");

    switch (Constants.kCurrentMode) {
      case REAL -> {
        Logger.addDataReceiver(new WPILOGWriter(logFolder()));
        Logger.addDataReceiver(new NT4Publisher());
      }
      case SIM -> Logger.addDataReceiver(new NT4Publisher());
      case REPLAY -> {
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim")));
      }
    }

    Logger.start();

    // Serves deploy/elastic on port 5800 so Elastic can pull a layout straight off the robot.
    WebServer.start(
        5800, Filesystem.getDeployDirectory().toPath().resolve("elastic").toString());

    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    LoggedTracer.reset();
    robotContainer.updateMovingShot();
    CommandScheduler.getInstance().run();
    LoggedTracer.record("Commands");
    robotContainer.logDriveState();
    robotContainer.logMatchState();
    robotContainer.logDashboardState();
    robotContainer.updatePowerMonitor();
    LoggedTracer.record("DriveState");
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
