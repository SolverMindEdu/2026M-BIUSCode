// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import com.ctre.phoenix6.CANBus;

public final class HoodConstants {
  private HoodConstants() {}

  public static final CANBus kCanBus = CANBus.roboRIO();
  public static final int kMotorId = 18;

  // TODO(10015): MEASURE THIS.
  public static final double kGearRatio = 5.0;

  public static final boolean kInverted = false;

  public static final double kDeployDegrees = 200.0;
  public static final double kStowDegrees = 0.0;

  // --- Travel ---------------------------------------------------------------------------------

  public static final boolean kTravelMeasured = true;

  public static final double kMinDegrees = 0.0;

  public static final double kMaxDegrees = 300.0;

  public static final double kLimitMarginDegrees = 5.0;

  public static final double kJogVolts = 1.0;

  public static final double kStatorAmps = 40.0;
  public static final double kSupplyAmps = 40.0;

  public static final double kToleranceDegrees = 10.0;

  // TODO(10015) if the hood is gravity-loaded, unlike

  public static final double kCruiseRotPerSec = 3.0;

  public static final double kAccelRotPerSecSq = 15.0;

  public static final double kS = 0.0;

  public static final double kA = 0.05;

  public static final double kG = 0.2;

  public static final double kV = 0.6;

  public static final double kP = 15.0;

  public static final double kD = 0.0;
}
