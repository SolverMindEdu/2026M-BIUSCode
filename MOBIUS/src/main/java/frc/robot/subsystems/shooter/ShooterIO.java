// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean leftConnected = false;
    public boolean rightConnected = false;
    public double leftRpm = 0.0;
    public double rightRpm = 0.0;
    public double leftAppliedVolts = 0.0;
    public double rightAppliedVolts = 0.0;
    public double leftSupplyVolts = 0.0;
    public double leftStatorCurrentAmps = 0.0;
    public double rightStatorCurrentAmps = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setRpm(double rpm) {}

  public default void stop() {}

  public default void setGains(double kP, double peakTorqueAmps) {}
}
