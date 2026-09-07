// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ballpath;

import org.littletonrobotics.junction.AutoLog;

public interface BallPathIO {
  @AutoLog
  public static class BallPathIOInputs {
    public boolean[] connected = new boolean[5];
    public double[] appliedVolts = new double[5];
    public double[] statorCurrentAmps = new double[5];
    public double[] velocityRotPerSec = new double[5];
  }

  public default void updateInputs(BallPathIOInputs inputs) {}

  public default void setIndexer(double percent) {}

  public default void setVerticalRoller(double percent) {}

  public default void setSingulatorTop(double percent) {}

  public default void setSingulatorBottom(double percent) {}

  public default void setFeedRotPerSec(double rotPerSec) {}

  public default void stop() {}
}
