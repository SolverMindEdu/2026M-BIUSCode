package frc.robot.shooting;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Shoot-on-the-move after 6328's LaunchCalculator: advance the pose, walk a lookahead point, aim from it. */
public final class LaunchCalculator {
  /** 6328's control phase delay. */
  public static final double kPhaseDelaySecs = 0.03;

  /** Your measured hopper-to-exit time, added because 6328's kicker releases almost instantly. */
  public static final double kReleaseDelaySecs = 0.13;

  /**
   * One RIO cycle of phase lag. The solve runs on a sample already taken and the command it
   * produces does not reach the motors until the next cycle, so the ball leaves a loop later than
   * the drive state it was aimed from.
   *
   * <p>One nominal period. The measured distribution is worse -- 23.9 ms mean, 31 ms p90, 78 ms
   * p99, with 16 percent of loops past 25 ms -- so this is the floor, not the average. Raising it
   * to the measured mean is a one-line change if the aim proves to trail during fast motion.
   */
  public static final double kRioCycleDelaySecs = 0.020;

  /** Everything between the drive sample the aim is solved from and the ball leaving the robot. */
  public static final double kTotalDelaySecs =
      kPhaseDelaySecs + kRioCycleDelaySecs + kReleaseDelaySecs;

  /** Cap, not a target: the loop stops as soon as it settles. See kLookaheadToleranceMeters. */
  public static final int kLookaheadIterations = 20;

  /**
   * The fixed point is converged once the range stops moving by this much. Measured over the shot
   * table it settles in one to five passes, worst case eleven, so running the full twenty every
   * time did four to twenty times the work and the allocation for no change in the answer -- and
   * this runs twice a loop, once leading and once direct. A millimetre of range is far below the
   * table's own resolution.
   */
  public static final double kLookaheadToleranceMeters = 0.001;

  private static final double kLoopPeriodSecs = 0.02;

  /** Field-relative aim, its rate, and the table row at the lookahead range. */
  public record Parameters(
      Rotation2d turretAngle,
      double turretVelocityRadPerSec,
      double hoodDegrees,
      double rpm,
      double timeOfFlightSecs,
      double distanceMeters,
      boolean withinTableRange,
      Translation2d turretPosition,
      Translation2d lookaheadPosition) {}

  // 3-sample moving average: 20 ms of lag instead of 6328's 40 ms, so the turret keeps up at SOTM speed.
  private final LinearFilter turretVelocityFilter = LinearFilter.movingAverage(3);
  private Rotation2d lastTurretAngle = null;
  private Translation2d lastTarget = null;

  public Parameters update(
      Pose2d pose,
      ChassisSpeeds robotRelativeSpeeds,
      Translation2d robotToTurret,
      Translation2d target,
      ShotTable table) {
    var solved =
        solve(
            pose,
            robotRelativeSpeeds,
            robotToTurret,
            target,
            table,
            kTotalDelaySecs);

    // A target switch is a step in angle, not motion, so it must not be differenced into a rate.
    if (lastTurretAngle == null || !target.equals(lastTarget)) {
      turretVelocityFilter.reset();
      lastTurretAngle = solved.turretAngle();
    }
    double rate =
        turretVelocityFilter.calculate(
            solved.turretAngle().minus(lastTurretAngle).getRadians() / kLoopPeriodSecs);
    lastTurretAngle = solved.turretAngle();
    lastTarget = target;

    return new Parameters(
        solved.turretAngle(),
        rate,
        solved.hoodDegrees(),
        solved.rpm(),
        solved.timeOfFlightSecs(),
        solved.distanceMeters(),
        solved.withinTableRange(),
        solved.turretPosition(),
        solved.lookaheadPosition());
  }

  /** The unfiltered solve; turretVelocityRadPerSec is always zero here. */
  public static Parameters solve(
      Pose2d pose,
      ChassisSpeeds robotRelativeSpeeds,
      Translation2d robotToTurret,
      Translation2d target,
      ShotTable table,
      double latencySecs) {
    Pose2d estimated =
        pose.exp(
            new Twist2d(
                robotRelativeSpeeds.vxMetersPerSecond * latencySecs,
                robotRelativeSpeeds.vyMetersPerSecond * latencySecs,
                robotRelativeSpeeds.omegaRadiansPerSecond * latencySecs));
    Rotation2d heading = estimated.getRotation();
    Translation2d offset = robotToTurret.rotateBy(heading);
    Translation2d turretPosition = estimated.getTranslation().plus(offset);

    // Omega cross r; 6328's X term flips the offset's Y contribution, which is only harmless on the centreline.
    double omega = robotRelativeSpeeds.omegaRadiansPerSecond;
    Translation2d turretVelocity =
        new Translation2d(
                robotRelativeSpeeds.vxMetersPerSecond, robotRelativeSpeeds.vyMetersPerSecond)
            .rotateBy(heading)
            .plus(new Translation2d(-omega * offset.getY(), omega * offset.getX()));

    Translation2d lookahead = turretPosition;
    double distance = target.getDistance(turretPosition);
    for (int i = 0; i < kLookaheadIterations; i++) {
      double timeOfFlight = table.get(distance).timeOfFlightSecs();
      lookahead = turretPosition.plus(turretVelocity.times(timeOfFlight));
      double next = target.getDistance(lookahead);
      boolean settled = Math.abs(next - distance) < kLookaheadToleranceMeters;
      distance = next;
      if (settled) {
        break;
      }
    }

    var row = table.get(distance);
    return new Parameters(
        target.minus(lookahead).getAngle(),
        0.0,
        row.hoodDegrees(),
        row.rpm(),
        row.timeOfFlightSecs(),
        distance,
        table.isWithinRange(distance),
        turretPosition,
        lookahead);
  }
}
