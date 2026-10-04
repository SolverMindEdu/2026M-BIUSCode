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

  /** Pre-spin speed: the left bumper toggle in teleop, and PrepShoot in auto. */
  public static final double kSpinUpRpm = 1000.0;
  public static final double kToleranceRpm = 100.0;

  public static final double kSpinUpSettleSecs = 0.1;

  /** Resting speed while enabled: most of the spin-up saved for a fraction of the current of a full hold. */
  public static final double kIdleRpm = 600.0;



  public static final double kStallCurrentAmps = 35.0;

  public static final double kStallRpm = 100.0;

  public static final double kStatorAmps = 100.0;
  public static final double kSupplyAmps = 90.0;

  // Trial settings: torque current is per motor, not battery current.
  public static final boolean kUseBangBang = true;
  public static final double kBangBangP = 999999.0;
  public static final double kBangBangTorqueAmps = 120.0;
  // Reference uses 20 rad/s; our mechanism velocities are RPM.
  public static final double kTorqueControlToleranceRpm = 20.0 * 60.0 / (2.0 * Math.PI);
  public static final double kTorqueControlHoldSecs = 0.020;

  // Slot 0 voltage gains -- the live control path while kUseBangBang is false.
  public static final double kS = 0.272;
  public static final double kV = 0.1723;
  public static final double kA = 0.0;
  public static final double kP = 1.2;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
}
