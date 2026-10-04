package frc.robot;

import static org.junit.jupiter.api.Assertions.*;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.MatchState.Phase;
import org.junit.jupiter.api.Test;

class MatchStateTest {
  private static MatchState.State teleop(double secsRemaining, String data, Alliance alliance) {
    return MatchState.of(false, secsRemaining, data, alliance);
  }

  @Test
  void namedAllianceSitsOutTheOddShifts() {
    // "R" names red, so red is inactive in shift 1 and active in shift 2.
    assertFalse(teleop(MatchState.kTeleopSecs - 1, "R", Alliance.Red).ourGoalActive());
    assertTrue(teleop(MatchState.kTeleopSecs - 1, "R", Alliance.Blue).ourGoalActive());
    assertTrue(teleop(MatchState.kTeleopSecs - 26, "R", Alliance.Red).ourGoalActive());
    assertFalse(teleop(MatchState.kTeleopSecs - 26, "R", Alliance.Blue).ourGoalActive());
  }

  @Test
  void shiftsAreNumberedAndCountDownIndividually() {
    var first = teleop(MatchState.kTeleopSecs - 10, "B", Alliance.Blue);
    assertEquals(Phase.SHIFT, first.phase());
    assertEquals(1, first.shiftNumber());
    assertEquals(15.0, first.shiftSecsRemaining(), 1e-9);

    var fourth = teleop(MatchState.kTeleopSecs - 99, "B", Alliance.Blue);
    assertEquals(4, fourth.shiftNumber());
    assertEquals(1.0, fourth.shiftSecsRemaining(), 1e-9);
  }

  @Test
  void matchTimeSpansBothPeriods() {
    // Auto still has the whole teleop clock ahead of it; teleop is just its own remainder.
    assertEquals(
        15.0 + MatchState.kTeleopSecs,
        MatchState.of(true, 15.0, "B", Alliance.Blue).matchSecsRemaining(),
        1e-9);
    assertEquals(42.0, teleop(42.0, "B", Alliance.Blue).matchSecsRemaining(), 1e-9);
  }

  @Test
  void lastThirtySecondsAreEndgame() {
    var state = teleop(29.0, "B", Alliance.Blue);
    assertEquals(Phase.ENDGAME, state.phase());
    assertEquals(0, state.shiftNumber());
  }

  @Test
  void missingGameDataNeverClaimsWeAreShutOut() {
    var state = teleop(MatchState.kTeleopSecs - 1, "", Alliance.Red);
    assertFalse(state.dataValid());
    assertTrue(state.ourGoalActive());
  }

  @Test
  void autoAndPracticeModeReportNoShift() {
    assertEquals(Phase.AUTO, MatchState.of(true, 12.0, "", Alliance.Blue).phase());
    assertEquals(Phase.UNKNOWN, MatchState.of(false, -1.0, "", Alliance.Blue).phase());
  }
}
