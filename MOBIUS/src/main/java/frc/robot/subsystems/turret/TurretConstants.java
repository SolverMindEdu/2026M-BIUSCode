// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.math.geometry.Translation2d;

public final class TurretConstants {
  private TurretConstants() {}

  public static final int kMotorId = 10;
  public static final int kEncoderId = 9;

  // TODO(10015): confirm both devices share this bus, since FusedCANcoder cannot cross buses.
  public static final CANBus kCanBus = CANBus.roboRIO();
  public static final boolean kPositiveMatchesGyro = false;

  public static final boolean kInverted = false;

  // --- Calibration limits -------------------------------------------------------------------

  public static final double kCalibrationMaxVolts = 1.0;

  public static final double kCalibrationStatorAmps = 35.0;

  // --- Gear train ---------------------------------------------------------------------------

  public static final double kRotorToSensorRatio = (32.0 / 12.0) * (32.0 / 14.0);

  public static final double kSensorToMechanismRatio = 83.0 / 10.0;

  public static final double kTotalReduction = kRotorToSensorRatio * kSensorToMechanismRatio;
  // --- Travel, measured from the homed zero ----------------------------------------------------

  public static final boolean kTravelMeasured = true;

  private static final double kLimitMarginDegrees = 5.0;

  public static final double kMaxAngleDegrees = 337.7 - kLimitMarginDegrees;
  public static final double kMinAngleDegrees = -296.7 + kLimitMarginDegrees;
  public static final Translation2d kRobotToTurret = new Translation2d(0.0, 0.0);
  // TODO(10015): measure from CAD -- back is negative x, right is negative y.

  public static final double kForwardOffsetDegrees = -170.0;

  // --- Motion profile ------------------------------------------------------------------------

  public static final double kCruiseVelocityRotPerSec = 1.5;

  public static final double kAccelerationRotPerSecSq = 16.0;

  public static final double kJerkRotPerSecCubed = 0.0;

  // --- Slot 0 gains --------------------------------------------------------------------------
  // TODO(10015): replace kS, kV, kA with SysId output.

  public static final double kS = 0.10;

  public static final double kV = 6.07;

  public static final double kA = 0.0;

  public static final double kP = 30.0;

  public static final double kI = 0.0;

  public static final double kProfileThresholdDegrees = 90.0;

  public static final double kD = 6.75;

  public static final double kG = 0.0;

  // TODO(10015): derive from goal width at maximum range.
  public static final double kAngleToleranceDegrees = 1.0;
}
