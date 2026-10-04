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

  // 0.55, not 0.75: the tunnel is the slow stage, so fuel entering faster than it clears packs up.
  public static final double kIndexerPercent = 0.950;



  
  /** How long the singulators and tunnel spin before the indexer starts pushing fuel into them. */
  public static final double kStagePrimeSecs = 0.25;

  public static final double kVerticalRollerPercent = 1.00;

  public static final double kSingulatorTopPercent = 1.00;

  // The kicker runs closed-loop: its speed sets how fast the ball enters the flywheel, and open
  // loop that varied with load on every shot.

  // Kicker velocity target in rotations per second.
  public static final double kFeedRotPerSec = 104.0;
  
  public static final double kFeedKs = 0.15;
  public static final double kFeedKv = 0.1152;
  public static final double kFeedKp = 0.3;

  // --- Jam clearing, hopper through singulators; the kicker is never reversed --------------------

  public static final double kJamCurrentAmps = 30.0;

  public static final double kJamVelocityRotPerSec = 2.0;

  /** Consecutive loops of stall before a jam is called, at 20 ms each. */
  public static final int kJamLoopsToTrigger = 8;

  public static final double kJamClearSecs = 0.15;

  public static final double kStatorAmps = 60.0;
  public static final double kSupplyAmps = 30.0;

  // --- Singulator limits ------------------------------------------------------------------------
  // The singulators are the pair that has to break a packed column loose, so they get their own
  // limits rather than the shared roller pair above: headroom for the first moment, and a lower
  // ceiling once it is clear the motor is pushing against something that is not moving.
  //
  // The ball path is the second largest consumer on the robot after the drive -- 17.5 percent of
  // all charge and a 107.8 A peak across its five motors in the 26-09-20 logs -- and unlike the
  // indexer these two run for the whole trigger hold whether or not fuel is being fed.

  /** Brief ceiling, for breaking a stalled column loose. */
  public static final double kSingulatorSupplyAmps = 50.0;

  /** Sustained ceiling, once the brief one has been held for kSingulatorSupplyLowerTimeSecs. */
  public static final double kSingulatorSupplyLowerAmps = 30.0;

  /** How long the singulator may sit at the upper limit before it is backed off to the lower one. */
  public static final double kSingulatorSupplyLowerTimeSecs = 1.5;

  /** Motor-side ceiling; higher than supply because a stalled roller draws far more than it pulls. */
  public static final double kSingulatorStatorAmps = 80.0;

  /**
   * Singulator demand while the indexer is stopped, as a fraction of its normal rate.
   *
   * <p>Reduced rather than stopped. With the indexer held the singulators are pushing into a closed
   * path: all the extra speed buys is a packed column and current burnt for nothing, and the gates
   * above them now hold the indexer for a whole unwrap sweep. But stopping them outright is worse
   * -- that is exactly what left them restarting against a packed column and jamming, which is why
   * the transport was made to run for the whole trigger hold in the first place. This keeps them
   * turning, so there is no dead-stop restart, at a rate that cannot pack the column hard.
   */
  public static final double kSingulatorHoldScale = 0.35;
}
