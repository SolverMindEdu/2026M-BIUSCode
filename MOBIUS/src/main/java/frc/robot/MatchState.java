// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * REBUILT match structure: 20 s auto, a 10 s transition off the clock, four 25 s shifts, then a
 * 30 s endgame. FMS sends one character naming the alliance whose goal goes inactive first; that
 * alliance is active in shifts 2 and 4, and the other alliance is active in 1 and 3.
 */
public final class MatchState {
  private MatchState() {}

  public static final double kShiftSecs = 25.0;
  public static final int kShiftCount = 4;
  public static final double kEndgameSecs = 30.0;

  /** The transition is between periods, so the teleop clock starts at the shifts. */
  public static final double kTeleopSecs = kShiftCount * kShiftSecs + kEndgameSecs;

  /** TODO(10015): the spec says nothing about endgame, so this is an assumption. */
  public static final boolean kEndgameGoalsActive = true;

  public enum Phase {
    UNKNOWN,
    AUTO,
    SHIFT,
    ENDGAME
  }

  /** shiftNumber is 1-4 during SHIFT and 0 otherwise. */
  public record State(
      Phase phase,
      int shiftNumber,
      double shiftSecsRemaining,
      double matchSecsRemaining,
      boolean ourGoalActive,
      boolean dataValid) {}

  public static State of(
      boolean autonomous, double matchTimeSecs, String gameData, Alliance alliance) {
    boolean dataValid = gameData != null && (gameData.equals("R") || gameData.equals("B"));

    if (matchTimeSecs < 0.0) {
      return new State(Phase.UNKNOWN, 0, 0.0, 0.0, true, dataValid);
    }
    if (autonomous) {
      // Auto counts down its own clock, so the rest of the match is still ahead of it.
      return new State(
          Phase.AUTO, 0, matchTimeSecs, matchTimeSecs + kTeleopSecs, true, dataValid);
    }

    double elapsed = kTeleopSecs - matchTimeSecs;
    if (elapsed >= kShiftCount * kShiftSecs) {
      return new State(
          Phase.ENDGAME, 0, matchTimeSecs, matchTimeSecs, kEndgameGoalsActive, dataValid);
    }

    int shiftNumber = (int) Math.floor(Math.max(elapsed, 0.0) / kShiftSecs) + 1;
    double remaining = kShiftSecs - (Math.max(elapsed, 0.0) % kShiftSecs);

    // The named alliance sits out shift 1, so it is active on the even shifts.
    boolean namedActive = shiftNumber % 2 == 0;
    boolean weAreNamed =
        dataValid && (gameData.equals("R") ? alliance == Alliance.Red : alliance == Alliance.Blue);
    boolean active = !dataValid || (weAreNamed == namedActive);

    return new State(Phase.SHIFT, shiftNumber, remaining, matchTimeSecs, active, dataValid);
  }
}
