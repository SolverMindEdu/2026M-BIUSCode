// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeDeployIO {
  @AutoLog
  public static class IntakeDeployIOInputs {
    public boolean connected = false;
    public double positionRotations = 0.0;
    public double velocityRotPerSec = 0.0;
    public double setpointRotations = 0.0;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    /** True once the controller has rebooted while enabled -- its position zero is then wrong. */
    public boolean rebooted = false;
  }

  public default void updateInputs(IntakeDeployIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setPositionSetpoint(
      double rotations, double cruiseRotPerSec, double accelRotPerSecSq) {}

  public default void setSoftLimitsEnabled(boolean enabled) {}

  public default void stop() {}

  public default void setGains(double kP, double kD, double kG, double kA) {}
}
