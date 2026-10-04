// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.turret.TurretConstants;
import org.junit.jupiter.api.Test;

/**
 * The lookahead loop stops as soon as the range stops moving. This pins that the early exit costs
 * nothing in the answer, since the whole aim hangs off that range.
 */
class LookaheadConvergenceTest {
  private static final Translation2d kTarget = new Translation2d(8.0, 4.0);

  /** Runs the same fixed point the solver does, but always to the full cap. */
  private static double fullyIterated(Translation2d turretPos, Translation2d vel, ShotTable table) {
    double d = kTarget.getDistance(turretPos);
    for (int i = 0; i < LaunchCalculator.kLookaheadIterations; i++) {
      d = kTarget.getDistance(turretPos.plus(vel.times(table.get(d).timeOfFlightSecs())));
    }
    return d;
  }

  @Test
  void stoppingEarlyGivesTheSameRangeAsRunningEveryIteration() {
    var table = ShotMap.table();
    for (double range : new double[] {1.5, 2.0, 2.5, 3.0, 3.5, 4.0}) {
      for (double speed : new double[] {0.0, 1.0, 2.0, 3.0, 4.5}) {
        var pose = new Pose2d(kTarget.getX() - range, kTarget.getY(), Rotation2d.kZero);
        var solved =
            LaunchCalculator.solve(
                pose,
                new ChassisSpeeds(0.0, speed, 0.0),
                TurretConstants.kRobotToTurret,
                kTarget,
                table,
                LaunchCalculator.kTotalDelaySecs);
        double reference = fullyIterated(solved.turretPosition(), velocityOf(speed), table);
        assertEquals(
            reference,
            solved.distanceMeters(),
            LaunchCalculator.kLookaheadToleranceMeters * 2.0,
            "range disagreed at " + range + " m, " + speed + " m/s");
      }
    }
  }

  private static Translation2d velocityOf(double speed) {
    return new Translation2d(0.0, speed);
  }

  @Test
  void theToleranceIsWellBelowTheShotTablesOwnResolution() {
    // Table rows are 0.2 m apart, so a 1 mm settle is two orders of magnitude inside one row.
    assertTrue(LaunchCalculator.kLookaheadToleranceMeters <= 0.01);
  }
}
