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

  // --- Running limits -------------------------------------------------------------------------
  // Stator is brief slew torque; supply is what the battery actually sees. Raise kStatorAmps
  // until Turret/StatorCurrentAmps stops flat-topping during a hard slew.
  public static final double kStatorAmps = 60.0;

  public static final double kSupplyAmps = 40.0;

  // --- Gear train ---------------------------------------------------------------------------

  public static final double kRotorToSensorRatio = (32.0 / 12.0) * (32.0 / 14.0);

  public static final double kSensorToMechanismRatio = 83.0 / 10.0;

  public static final double kTotalReduction = kRotorToSensorRatio * kSensorToMechanismRatio;
  // --- Travel, measured from the homed zero ----------------------------------------------------

  public static final boolean kTravelMeasured = true;

  private static final double kLimitMarginDegrees = 5.0;

  public static final double kMaxAngleDegrees = 337.7 - kLimitMarginDegrees;
  public static final double kMinAngleDegrees = -296.7 + kLimitMarginDegrees;

  /** Middle of the travel; the wrap nearest here leaves the most room in both directions. */
  public static final double kCentreAngleDegrees = (kMinAngleDegrees + kMaxAngleDegrees) / 2.0;

  /** An early unwrap only happens if re-centring buys at least this much extra travel. */
  public static final double kRecentreGainDegrees = 90.0;
  // Y flipped from CAD: WPILib is +Y LEFT, and the sign-reversing aim error across the hub said
  // the lateral term was being applied backwards.
  public static final Translation2d kRobotToTurret = new Translation2d(-0.19685, 0.13335);

  /** The turret is placed at its physical zero before enabling. */
  public static final double kHomeAngleDegrees = 0.0;

  /**
   * Physical zero points 170 degrees clockwise from the intake/front, viewed from above.
   * With clockwise-positive turret angles, aiming forward requires a -170 degree setpoint.
   */
  public static final double kForwardOffsetDegrees = -172.0;

  // --- Motion profile ------------------------------------------------------------------------

  /**
   * Lowered from 1.7. kV alone asks for kV * cruise volts, and at 1.7 that is 10.7 V of a 12 V rail
   * before kP has said anything, so the loop had no authority left exactly when it was moving
   * fastest. At 1.45 the feedforward wants 9.1 V and leaves about 3 V for feedback.
   */
  public static final double kCruiseVelocityRotPerSec = 1.45;

  public static final double kAccelerationRotPerSecSq = 16.0;

  /** Floor on a measured profile step, so a fast or duplicated loop cannot stall the profile. */
  public static final double kMinProfileStepSecs = 0.004;

  /**
   * Ceiling on a measured profile step. Past this the loop stalled rather than ran, and replaying
   * the whole gap at once would throw the setpoint somewhere the turret was never asked to go;
   * logs show stalls into the hundreds of milliseconds and one of 13 seconds.
   */
  public static final double kMaxProfileStepSecs = 0.100;

  // --- Slot 0 gains --------------------------------------------------------------------------
  // TODO(10015): replace kS, kV, kA with SysId output.

  /**
   * Applied from the RIO, not the Talon slot, so it can switch off inside kStaticDeadbandDegrees.
   * The same regression that gave kA put kinetic friction at 0.1848 V, so this was already right.
   * Breakaway is higher, around 0.45 V, but raising kS to meet it would over-drive by 0.27 V once
   * the turret is moving and set up a limit cycle; closing that gap is kP's job, not kS's.
   */
  public static final double kS = 0.18;

  /** Inside this error kS is dropped, so it stops flipping sign across the target and shaking the
   * turret. Tightened from 0.5, which switched kS off across the whole band where stiction is what
   * is holding the turret short of the target. */
  public static final double kStaticDeadbandDegrees = 0.15;

  /**
   * 0.124 V per ROTOR rotation per second -- the usual 12 V / 5800 RPM figure for this motor --
   * expressed in the mechanism rotations the slot actually uses, because Feedback's
   * SensorToMechanismRatio is kTotalReduction. That conversion is the whole point of writing it
   * this way: a bare 0.124 in the slot would be a fiftieth of the feedforward intended.
   *
   * <p>Was 7.6, which is 21 percent above this and drove the turret past its own profile -- logged
   * peaks of 776 deg/s against a 612 deg/s commanded ceiling, then hunting back.
   */
  public static final double kV = 0.124 * kTotalReduction;

  /**
   * Least-squares fit of V = kS*sign(w) + kV*w + kA*a over 13117 logged moving samples, RMS
   * residual 0.63 V. TODO(10015): replace with a SysId sweep; the routine is wired up and has
   * never been run.
   */
  public static final double kA = 0.0727;

  /**
   * Volts per mechanism ROTATION of error, so 120 is 0.33 V per degree. The old 16 was 0.044 V per
   * degree -- 22 degrees of error to ask for a single volt -- and against a breakaway of about
   * 0.45 V that left the turret stalled anywhere inside roughly 6 degrees of the target, which is
   * what the logs show: 2.0 degrees median at rest with 0.24 V applied and nothing moving.
   *
   * <p>120 with kD 0 is critically damped or better for any plausible kA: zeta = (kV + kD) /
   * (2*sqrt(kP*kA)) = 1.06 at the fitted kA = 0.0727, and still above 0.7 even if kA is out by a
   * factor of two. Expected standing error about 0.9 degrees.
   *
   * <p>Next step once this is confirmed on the robot: kP 200 with kD 1.35, which is zeta = 1.0 and
   * about 0.5 degrees. Do not raise kP without kD past this point.
   */
  public static final double kP = 120.0;

  public static final double kI = 0.0;

  /** Zero is deliberate: at kP 120 the plant's own back-EMF already damps it (see kP). */
  public static final double kD = 0.0;

  /** Scales kP in blind spots, where the aim comes from odometry alone; kD follows its square root to keep the damping. */
  public static final double kBlindSpotGainScale = 0.5;

  public static final double kG = 0.0;

  /**
   * Goal moves smaller than this are ignored. Pose noise is a few centimetres, which at shooting
   * range is a fraction of a degree of bearing, and chasing it only buzzes the turret. The cost is a
   * standing aim error of at most this much: 3.9 cm of ball placement at 3 m, 9 cm at 7 m, against a
   * shot tolerance of kStartShotToleranceDegrees. A wrap or an unwrap is hundreds of degrees and is
   * unaffected. Tightened from 0.75 now that the loop settles inside a degree; at the old gains it
   * was lost in the stiction band anyway.
   */
  public static final double kGoalDeadbandDegrees = 0.3;

  // --- Unwrap gating ---------------------------------------------------------------------------
  // Running out of travel costs a full-turn sweep, and through it the barrel points anywhere but at
  // the target. Fuel sent during that sweep is thrown away, so the feed is held off until the
  // turret is back on the new wrap.

  /**
   * A goal that moves further than this in one loop is a wrap change, not tracking. Ordinary
   * tracking moves at most kCruiseVelocityRotPerSec * 360 * 0.02, about 12 degrees, so anything
   * near a half turn can only be an unwrap.
   */
  public static final double kUnwrapDetectDegrees = 180.0;

  /** Back within this of the new wrap counts as arrived; it is an error band, not a speed, so it
   * still resolves while the turret tracks a spinning chassis. Tightened from 8 degrees, which was
   * sized around the old gains' stiction band and allowed feeding 8 degrees off target. */
  public static final double kUnwrapSettledDegrees = 3.0;

  /** Wall-clock time inside that band before the feed is allowed to resume. Time, not a loop
   * count, so it does not stretch when the RIO overruns. */
  public static final double kUnwrapSettledSecs = 0.06;

  /** Backstop, so a turret that never quite settles cannot hold the feed off for a whole match. */
  public static final double kUnwrapTimeoutSecs = 1.5;

  /** Close enough to call a recall to zero done and hand the turret back to tracking. */
  public static final double kZeroToleranceDegrees = 1.0;

  /** Backstop on that recall, so stiction short of the tolerance cannot hold tracking off forever. */
  public static final double kZeroTimeoutSecs = 3.0;

  /** Turret error allowed to start a shot; the turret pre-tracks, so this rarely holds anything back. */
  public static final double kStartShotToleranceDegrees = 30.0;

  /** Turret error allowed once feeding, kept wide so a burst never pauses mid-stream. */
  public static final double kAngleToleranceDegrees = 60.0;
}
