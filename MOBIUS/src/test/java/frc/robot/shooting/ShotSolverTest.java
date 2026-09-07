// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.shooting;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.shooting.ShotTable.ShotParameters;
import org.junit.jupiter.api.Test;

class ShotSolverTest {
  private static final double kEpsilon = 1e-6;

  private static final Translation2d kOrigin = new Translation2d(0.0, 0.0);
  private static final Translation2d kGoal = new Translation2d(10.0, 0.0);
  private static final Translation2d kStill = new Translation2d(0.0, 0.0);

  private static ShotTable constantFlightTable() {
    ShotTable table = new ShotTable();
    table.put(1.0, new ShotParameters(2000.0, 20.0, 1.0));
    table.put(20.0, new ShotParameters(4000.0, 40.0, 1.0));
    return table;
  }

  private static ShotTable proportionalFlightTable() {
    ShotTable table = new ShotTable();
    table.put(1.0, new ShotParameters(2000.0, 20.0, 0.1));
    table.put(20.0, new ShotParameters(4000.0, 40.0, 2.0));
    return table;
  }

  @Test
  void aStationaryRobotAimsStraightAtTheGoal() {
    var solution =
        ShotSolver.solve(kOrigin, kStill, 0.0, kGoal, constantFlightTable(), 1.0);

    assertEquals(kGoal.getX(), solution.virtualGoal().getX(), kEpsilon);
    assertEquals(kGoal.getY(), solution.virtualGoal().getY(), kEpsilon);
    assertEquals(10.0, solution.effectiveDistanceMeters(), kEpsilon);
    assertEquals(0.0, solution.fieldBearing().getDegrees(), kEpsilon);
    assertEquals(0.0, solution.turretRateRadPerSec(), kEpsilon);
  }

  @Test
  void drivingAtTheGoalShortensTheShotWithoutChangingTheBearing() {
    var solution =
        ShotSolver.solve(
            kOrigin, new Translation2d(3.0, 0.0), 0.0, kGoal, constantFlightTable(), 1.0);

    assertEquals(7.0, solution.effectiveDistanceMeters(), kEpsilon);
    assertEquals(0.0, solution.fieldBearing().getDegrees(), kEpsilon);
    assertEquals(0.0, solution.turretRateRadPerSec(), kEpsilon);
  }

  @Test
  void drivingAwayFromTheGoalLengthensTheShot() {
    var solution =
        ShotSolver.solve(
            kOrigin, new Translation2d(-3.0, 0.0), 0.0, kGoal, constantFlightTable(), 1.0);

    assertEquals(13.0, solution.effectiveDistanceMeters(), kEpsilon);
    assertEquals(0.0, solution.fieldBearing().getDegrees(), kEpsilon);
  }

  @Test
  void drivingSidewaysLeadsTheShotAgainstTheDirectionOfTravel() {
    var solution =
        ShotSolver.solve(
            kOrigin, new Translation2d(0.0, 2.0), 0.0, kGoal, constantFlightTable(), 1.0);

    assertEquals(10.0, solution.virtualGoal().getX(), kEpsilon);
    assertEquals(-2.0, solution.virtualGoal().getY(), kEpsilon);
    assertTrue(
        solution.fieldBearing().getDegrees() < 0.0,
        "expected the shot to lead against travel, got " + solution.fieldBearing().getDegrees());
  }

  @Test
  void theTurretRateTracksTheGoalSweepingAcrossTheRobot() {
    var solution =
        ShotSolver.solve(
            kOrigin, new Translation2d(0.0, 2.0), 0.0, kGoal, constantFlightTable(), 1.0);

    assertEquals(-20.0 / 104.0, solution.turretRateRadPerSec(), kEpsilon);
  }

  @Test
  void chassisRotationIsRemovedFromTheTurretRate() {
    var moving =
        ShotSolver.solve(
            kOrigin, new Translation2d(0.0, 2.0), 0.0, kGoal, constantFlightTable(), 1.0);
    var spinning =
        ShotSolver.solve(
            kOrigin, new Translation2d(0.0, 2.0), 0.5, kGoal, constantFlightTable(), 1.0);

    assertEquals(moving.turretRateRadPerSec() - 0.5, spinning.turretRateRadPerSec(), kEpsilon);
  }

  @Test
  void aSpinningStationaryRobotStillNeedsTheTurretToCounterRotate() {
    var solution = ShotSolver.solve(kOrigin, kStill, 1.5, kGoal, constantFlightTable(), 1.0);
    assertEquals(-1.5, solution.turretRateRadPerSec(), kEpsilon);
  }

  @Test
  void theSolutionIsAGenuineFixedPointOfTheFlightTimeLookup() {
    ShotTable table = proportionalFlightTable();
    var solution =
        ShotSolver.solve(kOrigin, new Translation2d(2.5, 1.5), 0.0, kGoal, table, 0.5);

    assertTrue(solution.converged(), "solver did not converge");
    assertEquals(
        table.get(solution.effectiveDistanceMeters()).timeOfFlightSecs(),
        solution.parameters().timeOfFlightSecs(),
        1e-4);
  }

  @Test
  void warmStartingWithTheAnswerConvergesImmediatelyToTheSameSolution() {
    ShotTable table = proportionalFlightTable();
    Translation2d velocity = new Translation2d(2.5, 1.5);

    var cold = ShotSolver.solve(kOrigin, velocity, 0.0, kGoal, table, 0.5);
    var warm =
        ShotSolver.solve(
            kOrigin, velocity, 0.0, kGoal, table, cold.parameters().timeOfFlightSecs());

    assertTrue(warm.converged());
    assertEquals(cold.effectiveDistanceMeters(), warm.effectiveDistanceMeters(), 1e-4);
    assertEquals(cold.fieldBearing().getRadians(), warm.fieldBearing().getRadians(), 1e-4);
  }

  @Test
  void flagsAShotOutsideTheMeasuredRangeInsteadOfSilentlyClamping() {
    ShotTable table = constantFlightTable();
    var farAway =
        ShotSolver.solve(kOrigin, kStill, 0.0, new Translation2d(50.0, 0.0), table, 1.0);

    assertFalse(farAway.withinTableRange(), "a 50m shot is well outside the measured table");
  }

  @Test
  void interpolatesHoodAngleAndRpmBetweenMeasuredPoints() {
    ShotTable table = new ShotTable();
    table.put(2.0, new ShotParameters(2000.0, 20.0, 0.5));
    table.put(4.0, new ShotParameters(3000.0, 30.0, 0.9));

    ShotParameters midpoint = table.get(3.0);
    assertEquals(2500.0, midpoint.rpm(), kEpsilon);
    assertEquals(25.0, midpoint.hoodDegrees(), kEpsilon);
    assertEquals(0.7, midpoint.timeOfFlightSecs(), kEpsilon);
  }
}
