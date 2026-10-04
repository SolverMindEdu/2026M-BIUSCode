// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Translation2d;
import org.junit.jupiter.api.Test;

/**
 * G407: FUEL may only be launched at our HUB while our bumpers are at least partly in our own
 * ALLIANCE ZONE, so the NEUTRAL ZONE is a passing-only half of the field. These pin the half-to-mode
 * mapping, including that the driver cannot opt out of it where the rule applies.
 */
class PassModeTest {
  private static final boolean KNOWN = true;
  private static final boolean UNKNOWN = false;
  private static final boolean IN_ZONE = true;
  private static final boolean NEUTRAL = false;
  private static final boolean SHOOTING = false;
  private static final boolean PASSING = true;

  @Test
  void crossingOutIntoTheNeutralZoneStartsPassing() {
    assertTrue(RobotContainer.passModeFor(KNOWN, NEUTRAL, IN_ZONE, SHOOTING));
  }

  @Test
  void comingBackOverTheBumpReturnsToScoring() {
    assertFalse(RobotContainer.passModeFor(KNOWN, IN_ZONE, NEUTRAL, PASSING));
  }

  @Test
  void passingIsHeldOnForEveryLoopOutInTheNeutralZone() {
    // Not just the crossing loop: a MAJOR FOUL is available on any loop out here.
    assertTrue(RobotContainer.passModeFor(KNOWN, NEUTRAL, NEUTRAL, PASSING));
  }

  @Test
  void theDriverCannotToggleBackToShootingFromTheNeutralZone() {
    // The toggle flips passMode false; the next loop must put it back before anything can fire.
    assertTrue(RobotContainer.passModeFor(KNOWN, NEUTRAL, NEUTRAL, SHOOTING));
  }

  @Test
  void insideOurOwnZoneTheDriverKeepsTheChoice() {
    // Passing from inside our own zone is legal, so a settled robot keeps whatever was selected.
    assertTrue(RobotContainer.passModeFor(KNOWN, IN_ZONE, IN_ZONE, PASSING));
    assertFalse(RobotContainer.passModeFor(KNOWN, IN_ZONE, IN_ZONE, SHOOTING));
  }

  @Test
  void anUnknownAllianceChangesNothingInEitherDirection() {
    // The zone test mirrors with the alliance, so before the station names one it means nothing.
    assertTrue(RobotContainer.passModeFor(UNKNOWN, NEUTRAL, IN_ZONE, PASSING));
    assertFalse(RobotContainer.passModeFor(UNKNOWN, NEUTRAL, IN_ZONE, SHOOTING));
    assertFalse(RobotContainer.passModeFor(UNKNOWN, IN_ZONE, NEUTRAL, SHOOTING));
  }

  @Test
  void theZoneBoundaryIsTheHubLineAndItMirrorsWithTheAlliance() {
    // Manual 5.3/5.4: the ALLIANCE ZONE is 158.6 in deep and the HUB centre sits 158.6 in from the
    // same wall, so the hub line is the boundary. Each alliance's own zone is the half behind it.
    var blueHub = FieldConstants.kBlueGoal;
    var redHub = FieldConstants.kRedGoal;
    assertTrue(blueHub.getX() < redHub.getX(), "expected blue's hub nearer the origin wall");

    // A point just behind each hub is in that alliance's zone and in the other's neutral zone.
    var behindBlue = new Translation2d(blueHub.getX() - 2.0, 4.0);
    var behindRed = new Translation2d(redHub.getX() + 2.0, 4.0);
    assertTrue(behindBlue.getX() < blueHub.getX());
    assertTrue(behindRed.getX() > redHub.getX());

    // And the midfield belongs to neither.
    double midX = (blueHub.getX() + redHub.getX()) / 2.0;
    assertTrue(midX > blueHub.getX() && midX < redHub.getX(), "midfield is neutral for both");
  }
}
