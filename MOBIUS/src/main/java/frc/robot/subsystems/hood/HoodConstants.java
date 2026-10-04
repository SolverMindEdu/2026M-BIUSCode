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

  public static final double kStowDegrees = 0.0;

  // --- Travel ---------------------------------------------------------------------------------

  public static final boolean kTravelMeasured = true;

  public static final double kMinDegrees = 0.0;

  public static final double kMaxDegrees = 300.0;

  public static final double kLimitMarginDegrees = 5.0;



  public static final double kStatorAmps = 40.0;
  public static final double kSupplyAmps = 25.0;

  public static final double kToleranceDegrees = 10.0;

  /** Below this the hood counts as stopped, for deciding it has run out of travel. */
  public static final double kStoppedDegPerSec = 5.0;

  /** Above this it is pushing against a stop rather than coasting to a halt. */
  public static final double kStopCurrentAmps = 20.0;

  /** Loops of not moving before a target it cannot reach counts as reached, at 20 ms each. */
  public static final int kTravelLimitLoops = 10;

  // TODO(10015) if the hood is gravity-loaded, unlike

  public static final double kCruiseRotPerSec = 3.0;

  public static final double kAccelRotPerSecSq = 15.0;

  public static final double kS = 0.0;

  public static final double kA = 0.05;

  public static final double kG = 0.2;

  public static final double kV = 0.6;

  /** Volts per hood ROTATION of error, so a 5 degree miss asks for 1.4 V. 15 gave 0.2 V. */
  public static final double kP = 100.0;

  /**
   * Volts per rotation-per-second of error rate. With kD at zero the loop had no damping at all,
   * so a ball hitting the hood rang against kP instead of settling. Raise until the ringing after
   * a shot stops; back off if the hood buzzes while holding still.
   */
  public static final double kD = 1.5;
}
