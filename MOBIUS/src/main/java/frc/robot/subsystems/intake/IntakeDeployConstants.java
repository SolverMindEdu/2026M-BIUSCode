// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.CANBus;

public final class IntakeDeployConstants {
  private IntakeDeployConstants() {}

  public static final CANBus kCanBus = CANBus.roboRIO();
  public static final int kMotorId = 21;

  public static final boolean kInverted = false;

  public static final double kGearRatio = 1.0 / 0.2133;

  // --- Travel, in MOTOR ROTATIONS ------------------------------------------------------------

  public static final double kRetractRotations = 0.03;

  public static final double kDeployRotations = 4.7;

  public static final double kLimitMarginRotations = 0.05;

  // --- Calibration ---------------------------------------------------------------------------

  public static final double kJogVolts = 1.0;

  public static final double kStatorAmps = 70.0;

  public static final double kSupplyAmps = 35.0;

  // --- Motion profile and gains ---------------------------------------------------------------

  public static final double kCruiseRotPerSec = 16.0;

  public static final double kAccelRotPerSecSq = 50.0;

  public static final double kRetractCruiseRotPerSec = 40.0;

  public static final double kRetractAccelRotPerSecSq = 250.0;

  public static final double kS = 0.34;

  public static final double kV = 0.1202;

  public static final double kA = 0.0096;

  public static final double kG = 0.0;

  public static final double kP = 25.0;

  public static final double kD = 0.0;

  /** Tight: a deploy is only done when the intake is actually all the way out. */
  public static final double kToleranceRotations = 0.02;

  /** Looser, and only for deciding the intake is home enough to release the motor. */
  public static final double kRetractHomeRotations = 0.1;

  // --- Frontal impact compliance -------------------------------------------------------------

  /**
   * Full authority: the intake has to reach the stop against a stretching net, and a cap here was
   * leaving it short. Compliance under a frontal hit is given up in exchange.
   */
  public static final double kPeakDeployDutyCycle = 1.0;

  public static final double kPeakRetractDutyCycle = 1.0;

  // --- Collision compliance ------------------------------------------------------------------

  public static final double kCollisionCurrentAmps = 30.0;

  public static final double kCollisionVelocityRotPerSec = 1.0;

  /** Loops of stalling before the intake settles for where it got to, at 20 ms each. */
  public static final int kSettleLoops = 25;

  /**
   * Only settle once the intake is nearly at its target. Stalling far from it is the intake failing
   * to break loose, and giving up there latches it wherever it started.
   */
  public static final double kSettleWindowRotations = 0.5;

  /**
   * Deploy gets a wider window than retract. Retract lands on a hard stop in a repeatable place;
   * deploy pushes into a net that stretches, so where it runs out of travel moves around, and a
   * stall a rotation short of kDeployRotations is still the intake being out, not a jam.
   */
  public static final double kDeploySettleWindowRotations = 1.2;

  // Shooting slowly walks the intake home rather than bobbing: about 3 s end to end.
  public static final double kShootStowCruiseRotPerSec = 1.8;

  public static final double kShootStowAccelRotPerSecSq = 4.0;

  /** Stator limit while stowing during a shot. Deploy and retract keep the full limit. */
  public static final double kShootStowStatorAmps = 40.0;
}
