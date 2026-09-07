// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;

class TurretMathTest {
  private static final double kMin = -200.0;
  private static final double kMax = 200.0;
  private static final double kEpsilon = 1e-9;

  private static double resolve(double desired, double current) {
    OptionalDouble result = TurretMath.resolveSetpoint(desired, current, kMin, kMax);
    assertTrue(result.isPresent(), "expected " + desired + " to be reachable from " + current);
    return result.getAsDouble();
  }

  @Test
  void takesTheDirectPathWhenTheTargetIsComfortablyInRange() {
    assertEquals(90.0, resolve(90.0, 0.0), kEpsilon);
    assertEquals(-45.0, resolve(-45.0, 0.0), kEpsilon);
  }

  @Test
  void unwrapsBackwardsRatherThanDrivingPastTheUpperLimit() {
    assertEquals(-155.0, resolve(205.0, 195.0), kEpsilon);
  }

  @Test
  void unwrapsForwardsRatherThanDrivingPastTheLowerLimit() {
    assertEquals(155.0, resolve(-205.0, -195.0), kEpsilon);
  }

  @Test
  void prefersTheShorterRouteWhenABearingIsReachableTwoWays() {
    assertEquals(-170.0, resolve(190.0, 0.0), kEpsilon);
  }

  @Test
  void doesNotUnwrapBackAgainOnceCommitted() {
    assertEquals(-161.0, resolve(199.0, -155.0), kEpsilon);
  }

  @Test
  void reportsBearingsInTheDeadZoneAsUnreachable() {
    assertTrue(TurretMath.resolveSetpoint(180.0, 0.0, -90.0, 90.0).isEmpty());
    assertTrue(TurretMath.resolveSetpoint(135.0, 0.0, -90.0, 90.0).isEmpty());
  }

  @Test
  void acceptsADesiredBearingGivenInAnyRevolution() {
    assertEquals(resolve(90.0, 0.0), resolve(450.0, 0.0), kEpsilon);
    assertEquals(resolve(90.0, 0.0), resolve(-270.0, 0.0), kEpsilon);
  }

  @Test
  void neverReturnsAnAngleOutsideTheTravelLimits() {
    for (double desired = -720.0; desired <= 720.0; desired += 3.0) {
      for (double current = kMin; current <= kMax; current += 5.0) {
        OptionalDouble result = TurretMath.resolveSetpoint(desired, current, kMin, kMax);
        if (result.isPresent()) {
          double setpoint = result.getAsDouble();
          assertTrue(
              setpoint >= kMin - kEpsilon && setpoint <= kMax + kEpsilon,
              "setpoint " + setpoint + " escaped limits for desired=" + desired);
        }
      }
    }
  }

  @Test
  void alwaysFindsASolutionWhenTravelCoversAFullRevolution() {
    for (double desired = -720.0; desired <= 720.0; desired += 3.0) {
      for (double current = kMin; current <= kMax; current += 5.0) {
        assertTrue(
            TurretMath.resolveSetpoint(desired, current, kMin, kMax).isPresent(),
            "no solution for desired=" + desired + " from current=" + current);
      }
    }
  }

  // --- Hall effect cross-check ---

  @Test
  void reportsNoErrorWhenTheEncoderAgreesWithTheMagnet() {
    assertEquals(0.0, TurretMath.referenceError(42.0, 42.0), kEpsilon);
  }

  @Test
  void reportsSignedErrorWhenTheEncoderHasDrifted() {
    assertEquals(2.5, TurretMath.referenceError(44.5, 42.0), kEpsilon);
    assertEquals(-2.5, TurretMath.referenceError(39.5, 42.0), kEpsilon);
  }

  @Test
  void treatsAMagnetCrossingAFullRevolutionAwayAsAgreement() {
    assertEquals(0.0, TurretMath.referenceError(402.0, 42.0), kEpsilon);
    assertEquals(0.0, TurretMath.referenceError(-318.0, 42.0), kEpsilon);
  }

  @Test
  void wrapsErrorAcrossTheDiscontinuityRatherThanReportingNearlyAFullTurn() {
    assertEquals(-2.0, TurretMath.referenceError(179.0, -179.0), kEpsilon);
    assertEquals(2.0, TurretMath.referenceError(-179.0, 179.0), kEpsilon);
  }

  // --- Dead-wedge fallback ---

  private static final double kBotMin = -236.0;
  private static final double kBotMax = 68.5;

  @Test
  void parksAtTheNearerEdgeOfTheDeadWedge() {
    assertEquals(kBotMax, TurretMath.clampToNearestLimit(75.0, kBotMin, kBotMax), kEpsilon);

    assertEquals(kBotMin, TurretMath.clampToNearestLimit(120.0, kBotMin, kBotMax), kEpsilon);
  }

  @Test
  void comparesEdgesByBearingRatherThanRawDifference() {
    assertEquals(kBotMin, TurretMath.clampToNearestLimit(100.0, kBotMin, kBotMax), kEpsilon);
  }

  @Test
  void alwaysReturnsOneOfTheTwoLimits() {
    for (double desired = -720.0; desired <= 720.0; desired += 1.0) {
      double result = TurretMath.clampToNearestLimit(desired, kBotMin, kBotMax);
      assertTrue(
          result == kBotMin || result == kBotMax,
          "clamp returned " + result + ", which is not a travel limit");
    }
  }
}
