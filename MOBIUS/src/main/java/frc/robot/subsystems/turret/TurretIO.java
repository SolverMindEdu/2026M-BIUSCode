// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean motorConnected = false;
    public boolean encoderConnected = false;

    public double encoderAbsoluteRotations = 0.0;

    public double positionDegrees = 0.0;

    public double velocityDegreesPerSec = 0.0;
    public double setpointDegrees = 0.0;

    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double supplyCurrentAmps = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setPositionSetpoint(double degrees, double velocityDegPerSec) {}

  public default void setProfiledSetpoint(double degrees) {}

  public default void stop() {}

  public default void seedPosition(double degrees) {}

  public default void setSoftLimitsEnabled(boolean enabled) {}
}
