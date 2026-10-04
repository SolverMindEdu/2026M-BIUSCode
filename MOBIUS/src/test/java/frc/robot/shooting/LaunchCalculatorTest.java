package frc.robot.shooting;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.shooting.ShotTable.ShotParameters;
import org.junit.jupiter.api.Test;

class LaunchCalculatorTest {
  private static final double kEpsilon = 1e-9;

  /** A one-second flight everywhere, so every lookahead is exactly velocity times one. */
  private static ShotTable table() {
    var table = new ShotTable();
    table.put(0.5, new ShotParameters(1500, 50, 1.0));
    table.put(20.0, new ShotParameters(2500, 150, 1.0));
    return table;
  }

  @Test
  void stationaryAimsStraightAtTheTarget() {
    var p =
        LaunchCalculator.solve(
            Pose2d.kZero, new ChassisSpeeds(), Translation2d.kZero, new Translation2d(3, 0),
            table(), 0.0);
    assertEquals(0.0, p.turretAngle().getRadians(), kEpsilon);
    assertEquals(3.0, p.distanceMeters(), kEpsilon);
    assertEquals(Translation2d.kZero, p.lookaheadPosition());
  }

  @Test
  void drivingSidewaysLeadsAgainstTheInheritedVelocity() {
    var p =
        LaunchCalculator.solve(
            Pose2d.kZero, new ChassisSpeeds(0, 1, 0), Translation2d.kZero,
            new Translation2d(5, 0), table(), 0.0);
    assertEquals(new Translation2d(0, 1), p.lookaheadPosition());
    assertEquals(Math.atan2(-1, 5), p.turretAngle().getRadians(), kEpsilon);
    assertEquals(Math.hypot(5, 1), p.distanceMeters(), kEpsilon);
  }

  @Test
  void offsetTurretUsesTheTrueCrossProductWhileSpinning() {
    // r = (-0.2, 0.1), omega = 2: v = (-omega*ry, omega*rx) = (-0.2, -0.4); 6328's form gives +0.2 in X.
    var p =
        LaunchCalculator.solve(
            Pose2d.kZero, new ChassisSpeeds(0, 0, 2), new Translation2d(-0.2, 0.1),
            new Translation2d(5, 0), table(), 0.0);
    assertEquals(-0.2, p.turretPosition().getX(), kEpsilon);
    assertEquals(0.1, p.turretPosition().getY(), kEpsilon);
    assertEquals(-0.4, p.lookaheadPosition().getX(), kEpsilon);
    assertEquals(-0.3, p.lookaheadPosition().getY(), kEpsilon);
  }

  @Test
  void latencyAdvancesThePoseBeforeSolving() {
    var p =
        LaunchCalculator.solve(
            Pose2d.kZero, new ChassisSpeeds(1, 0, 0), Translation2d.kZero,
            new Translation2d(5, 0), table(), 0.1);
    assertEquals(0.1, p.turretPosition().getX(), kEpsilon);
    assertEquals(3.9, p.distanceMeters(), kEpsilon);
  }

  @Test
  void switchingTargetsDoesNotInventATurretRate() {
    var calculator = new LaunchCalculator();
    calculator.update(
        Pose2d.kZero, new ChassisSpeeds(), Translation2d.kZero, new Translation2d(5, 0), table());
    var p =
        calculator.update(
            Pose2d.kZero, new ChassisSpeeds(), Translation2d.kZero, new Translation2d(0, 5),
            table());
    assertEquals(0.0, p.turretVelocityRadPerSec(), kEpsilon);
  }

  @Test
  void holdingStillReportsNoRate() {
    var calculator = new LaunchCalculator();
    LaunchCalculator.Parameters p = null;
    for (int i = 0; i < 10; i++) {
      p =
          calculator.update(
              Pose2d.kZero, new ChassisSpeeds(), Translation2d.kZero, new Translation2d(4, 1),
              table());
    }
    assertEquals(0.0, p.turretVelocityRadPerSec(), kEpsilon);
  }

  @Test
  void theDelayCountsControlPhaseTheRioCycleAndTheRelease() {
    // A dropped term here shows up on the field as an aim that trails during fast motion, which is
    // hard to tell from a gain problem, so pin the composition rather than just the total.
    assertEquals(
        LaunchCalculator.kPhaseDelaySecs
            + LaunchCalculator.kRioCycleDelaySecs
            + LaunchCalculator.kReleaseDelaySecs,
        LaunchCalculator.kTotalDelaySecs,
        kEpsilon);
    assertEquals(0.18, LaunchCalculator.kTotalDelaySecs, kEpsilon);
    // One RIO cycle, and the loop actually runs slower than nominal, so never less than that.
    assertTrue(LaunchCalculator.kRioCycleDelaySecs >= 0.020);
  }

}
