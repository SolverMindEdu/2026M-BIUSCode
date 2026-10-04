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

  /** Stator limit while assisting a shot. Intaking keeps the full limit for actually grabbing fuel. */
  public static final double kShootStatorAmps = 30.0;

  /** Backwards kick on the first deploy of a match, to clear anything sitting in the rollers. */
  public static final double kPurgePercent = 0.4;

  public static final double kPurgeSecs = 0.3;

  public static final double kStatorAmps = 90.0;
  // Raised with the stator limit: at 120 A stator the roller draws ~39 A supply, so a 40 A
  // supply limit would throttle the voltage before the stator limit was ever reached.
  public static final double kSupplyAmps = 60.0;
}
