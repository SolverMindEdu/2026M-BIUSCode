// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.Timer;
import org.littletonrobotics.junction.Logger;

public class LoggedTracer {
  private LoggedTracer() {}

  private static double startTime = -1.0;

  public static void reset() {
    startTime = Timer.getFPGATimestamp();
  }

  public static void record(String epochName) {
    double now = Timer.getFPGATimestamp();
    Logger.recordOutput("LoggedTracer/" + epochName + "MS", (now - startTime) * 1000.0);
    startTime = now;
  }
}
