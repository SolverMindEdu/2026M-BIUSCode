// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

public final class IntakeRollerConstants {
  private IntakeRollerConstants() {}

  public static final CANBus kCanBus = CANBus.roboRIO();
  public static final int kMotorId = 11;

  public static final boolean kInverted = true;

  public static final double kIntakePercent = 0.85;

  public static final double kShootPercent = 0.4;

  public static final double kStatorAmps = 60.0;
  public static final double kSupplyAmps = 60.0;
}
