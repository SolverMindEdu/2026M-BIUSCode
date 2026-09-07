// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;

public final class ShooterConstants {
  private ShooterConstants() {}

  public static final CANBus kCanBus = CANBus.roboRIO();
  public static final int kLeftId = 15;
  public static final int kRightId = 23;

  // --- Direction ------------------------------------------------------------------------------

  public static final boolean kLeftInverted = false;
  public static final boolean kRightInverted = true;

  public static final double kGearRatio = 54.0 / 38.0;

  public static final double kTargetRpm = 800.0;
  public static final double kToleranceRpm = 100.0;

  public static final double kSpinUpSettleSecs = 0.5;

  public static final double kIdleRpm = 0.0;

  public static final double kTestRpm = 400.0;

  public static final double kStallCurrentAmps = 35.0;

  public static final double kStallRpm = 100.0;

  public static final double kStatorAmps = 120.0;
  public static final double kSupplyAmps = 80.0;

  public static final double kV = 0.0;
  public static final double kS = 0.0;

  public static final double kP = 999999.0;

  public static final double kPeakTorqueAmps = 80.0;
}
