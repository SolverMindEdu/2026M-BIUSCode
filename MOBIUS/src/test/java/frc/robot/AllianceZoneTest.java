// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.FieldConstants.PassSide;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/**
 * Everything that mirrors with the alliance, checked at absolute field positions on both sides.
 *
 * <p>Relative scenarios cannot catch a mirror that is wrong, because they are the same geometry
 * translated. These use real coordinates, and each assertion checks that a point belonging to one
 * alliance does NOT belong to the other.
 */
class AllianceZoneTest {
  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.setDsAttached(true);
  }

  @AfterEach
  void tearDown() {
    use(DriverStation.Alliance.Blue);
  }

  private static void use(DriverStation.Alliance alliance) {
    DriverStationSim.setAllianceStationId(
        alliance == DriverStation.Alliance.Red ? AllianceStationID.Red1 : AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    assertEquals(alliance, FieldConstants.alliance());
  }

  /** Deep in blue's half, and its 180-degree partner deep in red's. */
  private static final Translation2d kDeepInBlue = new Translation2d(1.5, 3.0);

  @Test
  void eachAllianceOwnsItsOwnEndAndNotTheOther() {
    var deepInRed = new Translation2d(0.0, 0.0);
    use(DriverStation.Alliance.Red);
    deepInRed = FieldConstants.mirrorForAlliance(kDeepInBlue);

    use(DriverStation.Alliance.Blue);
    assertTrue(FieldConstants.inOurAllianceZone(kDeepInBlue), "blue does not own blue's end");
    assertFalse(FieldConstants.inOurAllianceZone(deepInRed), "blue thinks it owns red's end");

    use(DriverStation.Alliance.Red);
    assertTrue(FieldConstants.inOurAllianceZone(deepInRed), "red does not own red's end");
    assertFalse(FieldConstants.inOurAllianceZone(kDeepInBlue), "red thinks it owns blue's end");
  }

  @Test
  void midfieldBelongsToNeither() {
    var mid =
        new Translation2d(
            (FieldConstants.kBlueGoal.getX() + FieldConstants.kRedGoal.getX()) / 2.0, 4.0);
    use(DriverStation.Alliance.Blue);
    assertFalse(FieldConstants.inOurAllianceZone(mid), "blue claims midfield");
    use(DriverStation.Alliance.Red);
    assertFalse(FieldConstants.inOurAllianceZone(mid), "red claims midfield");
  }

  @Test
  void theHubEachAllianceScoresInIsTheOneAtItsOwnEnd() {
    use(DriverStation.Alliance.Blue);
    assertEquals(FieldConstants.kBlueGoal, FieldConstants.ourGoal());
    assertTrue(FieldConstants.inOurAllianceZone(nudgeTowardOwnWall(FieldConstants.ourGoal())));
    use(DriverStation.Alliance.Red);
    assertEquals(FieldConstants.kRedGoal, FieldConstants.ourGoal());
    assertTrue(FieldConstants.inOurAllianceZone(nudgeTowardOwnWall(FieldConstants.ourGoal())));
  }

  /** A metre behind the hub line, on whichever side of it our own wall is. */
  private static Translation2d nudgeTowardOwnWall(Translation2d hub) {
    boolean red = FieldConstants.alliance() == DriverStation.Alliance.Red;
    return new Translation2d(hub.getX() + (red ? 1.0 : -1.0), hub.getY());
  }

  @Test
  void aPassLandsInOurOwnZoneOnBothAlliances() {
    for (var alliance : DriverStation.Alliance.values()) {
      use(alliance);
      for (PassSide side : PassSide.values()) {
        var target = FieldConstants.passTarget(new Translation2d(8.0, 4.0), side);
        assertTrue(
            FieldConstants.inOurAllianceZone(target),
            alliance + " " + side + " pass landed outside its own zone at " + target);
      }
    }
  }

  @Test
  void theNeutralZoneForcesPassingAndOurOwnZoneRestoresScoringOnBothAlliances() {
    var mid =
        new Translation2d(
            (FieldConstants.kBlueGoal.getX() + FieldConstants.kRedGoal.getX()) / 2.0, 4.0);
    for (var alliance : DriverStation.Alliance.values()) {
      use(alliance);
      var home = nudgeTowardOwnWall(FieldConstants.ourGoal());

      // Out in the middle, G407 says the hub is not a legal target, so passing is forced on.
      boolean inZone = FieldConstants.inOurAllianceZone(mid);
      assertFalse(inZone, alliance + " thinks midfield is its own zone");
      assertTrue(
          RobotContainer.passModeFor(true, inZone, true, false),
          alliance + " did not switch to passing in the neutral zone");

      // Back over the bump, the choice is the driver's again and scoring is restored on entry.
      boolean homeInZone = FieldConstants.inOurAllianceZone(home);
      assertTrue(homeInZone, alliance + " does not recognise its own zone");
      assertFalse(
          RobotContainer.passModeFor(true, homeInZone, false, true),
          alliance + " stayed in passing after re-entering its own zone");
    }
  }
}
