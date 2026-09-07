// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.shooting.ShotTable.ShotParameters;

public final class ShotSolver {
  private ShotSolver() {}

  private static final int kMaxIterations = 10;
  private static final double kToleranceSecs = 1e-4;

  public record ShotSolution(
      Translation2d virtualGoal,
      double effectiveDistanceMeters,
      ShotParameters parameters,
      Rotation2d fieldBearing,
      double turretRateRadPerSec,
      boolean converged,
      boolean withinTableRange) {}

  public static ShotSolution solve(
      Translation2d robotTranslation,
      Translation2d fieldVelocity,
      double chassisOmegaRadPerSec,
      Translation2d goalTranslation,
      ShotTable table,
      double warmStartTimeOfFlightSecs) {
    double timeOfFlight = warmStartTimeOfFlightSecs;
    boolean converged = false;

    for (int i = 0; i < kMaxIterations; i++) {
      Translation2d virtualGoal = goalTranslation.minus(fieldVelocity.times(timeOfFlight));
      double distance = virtualGoal.minus(robotTranslation).getNorm();
      double next = table.get(distance).timeOfFlightSecs();
      boolean settled = Math.abs(next - timeOfFlight) < kToleranceSecs;
      timeOfFlight = next;
      if (settled) {
        converged = true;
        break;
      }
    }

    Translation2d virtualGoal = goalTranslation.minus(fieldVelocity.times(timeOfFlight));
    Translation2d toGoal = virtualGoal.minus(robotTranslation);
    double distance = toGoal.getNorm();
    ShotParameters parameters = table.get(distance);

    double fieldBearingRate =
        distance < 1e-6
            ? 0.0
            : (toGoal.getY() * fieldVelocity.getX() - toGoal.getX() * fieldVelocity.getY())
                / (distance * distance);

    return new ShotSolution(
        virtualGoal,
        distance,
        parameters,
        toGoal.getAngle(),
        fieldBearingRate - chassisOmegaRadPerSec,
        converged,
        table.isWithinRange(distance));
  }
}
