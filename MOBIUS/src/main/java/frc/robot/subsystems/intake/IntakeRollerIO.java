// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
  @AutoLog
  public static class IntakeRollerIOInputs {
    public boolean connected = false;
    public double appliedVolts = 0.0;
    public double statorCurrentAmps = 0.0;
    public double velocityRotPerSec = 0.0;
  }

  public default void updateInputs(IntakeRollerIOInputs inputs) {}

  public default void setPercent(double percent) {}

  public default void stop() {}
}
