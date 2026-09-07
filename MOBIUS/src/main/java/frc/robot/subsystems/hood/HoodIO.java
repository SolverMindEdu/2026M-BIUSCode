// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean connected = false;
    public double positionDegrees = 0.0;
    public double velocityDegreesPerSec = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setAngle(double degrees) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}
}
