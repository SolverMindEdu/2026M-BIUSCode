// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ballpath;

import static frc.robot.subsystems.ballpath.BallPathConstants.*;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * The singulators are slowed while the indexer is held, not stopped. Stopping them is what left
 * them restarting against a packed column and jamming, so the floor here is load-bearing.
 */
class SingulatorHoldTest {
  private static double singulatorAt(boolean feeding) {
    return kSingulatorTopPercent * (feeding ? 1.0 : kSingulatorHoldScale);
  }

  @Test
  void feedingRunsTheSingulatorsAtFullRate() {
    assertEquals(kSingulatorTopPercent, singulatorAt(true), 1e-9);
  }

  @Test
  void aHeldIndexerSlowsTheSingulatorsWithoutStoppingThem() {
    double held = singulatorAt(false);
    assertTrue(held > 0.0, "stopping them outright is what jams them");
    assertTrue(held < singulatorAt(true), "held rate must be lower than the feeding rate");
  }

  @Test
  void theHoldRateIsEnoughToKeepThemTurningAndLowEnoughToBeWorthIt() {
    // Too low and they stall against the column anyway; too high and nothing is saved.
    assertTrue(kSingulatorHoldScale >= 0.2, "too slow to keep the column moving");
    assertTrue(kSingulatorHoldScale <= 0.6, "not a big enough reduction to be worth doing");
  }

  @Test
  void theSingulatorsGetMoreHeadroomThanTheSharedRollerLimits() {
    // They are the pair that has to break a packed column loose.
    assertTrue(kSingulatorStatorAmps > kStatorAmps);
    assertTrue(kSingulatorSupplyAmps > kSupplyAmps);
    // And the sustained ceiling comes back to what everything else lives on.
    assertEquals(kSupplyAmps, kSingulatorSupplyLowerAmps, 1e-9);
    assertTrue(kSingulatorSupplyLowerLimitIsBelowUpper());
  }

  private static boolean kSingulatorSupplyLowerLimitIsBelowUpper() {
    return kSingulatorSupplyLowerAmps < kSingulatorSupplyAmps
        && kSingulatorSupplyLowerTimeSecs > 0.0;
  }
}
