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

  public static final double kDeployRotations = 4.68;

  public static final double kLimitMarginRotations = 0.05;

  // --- Calibration ---------------------------------------------------------------------------

  public static final double kJogVolts = 1.0;

  public static final double kStatorAmps = 100.0;

  public static final double kSupplyAmps = 60.0;

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

  public static final double kToleranceRotations = 0.1;

  // --- Frontal impact compliance -------------------------------------------------------------

  public static final double kPeakDeployDutyCycle = 0.75;

  public static final double kPeakRetractDutyCycle = 1.0;

  // --- Collision compliance ------------------------------------------------------------------

  public static final double kCollisionCurrentAmps = 30.0;

  public static final double kCollisionVelocityRotPerSec = 1.0;

  /** Loops of stalling on the way in before the intake settles for where it got to, at 20 ms each. */
  public static final int kRetractSettleLoops = 25;

  /**
   * Only settle once the intake is nearly home. Stalling far from the target is the intake failing
   * to break loose, and giving up there latches it wherever it started.
   */
  public static final double kRetractSettleWindowRotations = 0.5;

  /** How far in the intake comes on each shooting pulse, measured from the deployed position. */
  public static final double kShootPulseLowRotations = 4.5;

  /** Time at each end of the shooting pulse. */
  public static final double kShootPulsePeriodSecs = 0.15;
}
