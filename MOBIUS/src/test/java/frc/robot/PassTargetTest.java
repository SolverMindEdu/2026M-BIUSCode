// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.FieldConstants.PassSide;
import frc.robot.subsystems.vision.VisionConstants;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

/** Where a pass lands, and that red gets the same placement turned around rather than a new one. */
class PassTargetTest {
  private static final double kEpsilon = 1e-6;
  private static final double L = VisionConstants.aprilTagLayout.getFieldLength();
  private static final double W = VisionConstants.aprilTagLayout.getFieldWidth();

  @BeforeEach
  void setUp() {
    assertTrue(HAL.initialize(500, 0), "HAL failed to initialize");
    DriverStationSim.setDsAttached(true);
  }

  @AfterEach
  void tearDown() {
    setAlliance(DriverStation.Alliance.Blue);
  }

  private static void setAlliance(DriverStation.Alliance alliance) {
    DriverStationSim.setAllianceStationId(
        alliance == DriverStation.Alliance.Red
            ? edu.wpi.first.hal.AllianceStationID.Red1
            : edu.wpi.first.hal.AllianceStationID.Blue1);
    DriverStationSim.notifyNewData();
    DriverStation.refreshData();
    // FieldConstants caches on read; make it take the new value.
    assertEquals(alliance, FieldConstants.alliance());
  }

  private static void assertAt(double x, double y, Translation2d actual) {
    assertEquals(x, actual.getX(), kEpsilon);
    assertEquals(y, actual.getY(), kEpsilon);
  }

  private static final Translation2d kAnywhere = new Translation2d(8.0, 4.0);

  @Test
  void blueDropsThePassAMetreOffItsOwnWallOnTheNamedSide() {
    setAlliance(DriverStation.Alliance.Blue);
    assertAt(1.0, 7.0, FieldConstants.passTarget(kAnywhere, PassSide.LEFT));
    assertAt(1.0, 1.0, FieldConstants.passTarget(kAnywhere, PassSide.RIGHT));
  }

  @Test
  void redGetsTheSamePlacementTurnedOneHundredAndEightyDegrees() {
    setAlliance(DriverStation.Alliance.Red);
    assertAt(L - 1.0, W - 7.0, FieldConstants.passTarget(kAnywhere, PassSide.LEFT));
    assertAt(L - 1.0, W - 1.0, FieldConstants.passTarget(kAnywhere, PassSide.RIGHT));
  }

  @Test
  void bothAlliancesPassIntoTheirOwnHalf() {
    setAlliance(DriverStation.Alliance.Blue);
    for (PassSide side : PassSide.values()) {
      var target = FieldConstants.passTarget(kAnywhere, side);
      assertTrue(
          FieldConstants.inOurAllianceZone(target), "blue " + side + " landed outside our zone");
    }
    setAlliance(DriverStation.Alliance.Red);
    for (PassSide side : PassSide.values()) {
      var target = FieldConstants.passTarget(kAnywhere, side);
      assertTrue(
          FieldConstants.inOurAllianceZone(target), "red " + side + " landed outside our zone");
    }
  }

  @Test
  void bothKeepsThePassOnTheSideTheRobotIsAlreadyOn() {
    var highY = new Translation2d(8.0, W - 1.0);
    var lowY = new Translation2d(8.0, 1.0);
    setAlliance(DriverStation.Alliance.Blue);
    assertTrue(FieldConstants.passTarget(highY, PassSide.BOTH).getY() > W / 2.0);
    assertTrue(FieldConstants.passTarget(lowY, PassSide.BOTH).getY() < W / 2.0);
    setAlliance(DriverStation.Alliance.Red);
    assertTrue(FieldConstants.passTarget(highY, PassSide.BOTH).getY() > W / 2.0);
    assertTrue(FieldConstants.passTarget(lowY, PassSide.BOTH).getY() < W / 2.0);
  }

  @Test
  void theDriversLeftIsTheDriversLeftOnEitherAlliance() {
    setAlliance(DriverStation.Alliance.Blue);
    var blueLeft = FieldConstants.passTarget(kAnywhere, PassSide.LEFT);
    assertTrue(FieldConstants.isDriverLeft(blueLeft), "blue LEFT is not on blue's left");
    setAlliance(DriverStation.Alliance.Red);
    var redLeft = FieldConstants.passTarget(kAnywhere, PassSide.LEFT);
    assertTrue(FieldConstants.isDriverLeft(redLeft), "red LEFT is not on red's left");
    // Same physical corner must not serve both alliances.
    assertTrue(blueLeft.getDistance(redLeft) > 10.0, "both alliances aimed at the same corner");
  }

  @Test
  void mirroringIsAOneHundredAndEightyDegreeTurnAndIsItsOwnInverse() {
    var point = new Translation2d(3.0, 2.0);
    setAlliance(DriverStation.Alliance.Blue);
    assertAt(3.0, 2.0, FieldConstants.mirrorForAlliance(point));
    setAlliance(DriverStation.Alliance.Red);
    var turned = FieldConstants.mirrorForAlliance(point);
    assertAt(L - 3.0, W - 2.0, turned);
    assertAt(3.0, 2.0, FieldConstants.mirrorForAlliance(turned));
  }
}
