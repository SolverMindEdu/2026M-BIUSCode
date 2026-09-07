// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ballpath;

import com.ctre.phoenix6.CANBus;

public final class BallPathConstants {
  private BallPathConstants() {}

  public static final CANBus kCanBus = CANBus.roboRIO();

  public static final int kIndexerId = 30;
  public static final int kVerticalRollerId = 20;
  public static final int kSingulatorTopId = 22;
  public static final int kSingulatorBottomId = 25;
  public static final int kFeedId = 0;

  // --- Direction ------------------------------------------------------------------------------

  public static final boolean kIndexerInverted = false;
  public static final boolean kVerticalRollerInverted = true;
  public static final boolean kSingulatorTopInverted = false;
  public static final boolean kSingulatorBottomInverted = true;
  public static final boolean kFeedInverted = false;

  public static final double kIndexerHighPercent = 0.40;

  public static final double kIndexerLowPercent = 0.30;

  public static final double kIndexerPulsePeriodSecs = 0.15;

  public static final double kVerticalRollerPercent = 0.85;

  public static final double kSingulatorTopPercent = 0.95;

  public static final double kSingulatorBottomPercent = 0.90;
  // The kicker runs closed-loop: its speed sets how fast the ball enters the flywheel, and open
  // loop that varied with load on every shot.

  // TODO(10015): run the ball path, read BallPath/VelocityRotPerSec index 4 while it is up to
  // speed, and put that number here. kV is then 11.76 / that value.
  public static final double kFeedRotPerSec = 100.0;

  public static final double kFeedKs = 0.15;
  public static final double kFeedKv = 0.1176;
  public static final double kFeedKp = 0.3;

  // --- Loading while intaking ------------------------------------------------------------------

  public static final double kIntakeLoadHighPercent = 0.25;

  public static final double kIntakeLoadLowPercent = 0.15;

  public static final double kIntakeLoadPulsePeriodSecs = 0.15;

  public static final double kIntakeLoadStallAmps = 15.0;

  public static final int kIntakeLoadStallLoops = 5;

  // --- Jam detection -------------------------------------------------------------------------

  public static final double kJamCurrentAmps = 45.0;

  public static final double kJamVelocityRotPerSec = 2.0;

  public static final int kJamLoopsToTrigger = 15;

  public static final double kJamClearSecs = 0.15;

  public static final double kStatorAmps = 60.0;
  public static final double kSupplyAmps = 40.0;
}
