package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

class ShooterBangBangTest {
  @Test
  void startupContactRecoveryAndNextBall() {
    var control = new ShooterBangBang();
    assertFalse(control.useTorque(1000, 0, 0, 0));
    assertTrue(control.useTorque(1000, 990, 995, 1));
    assertTrue(control.useTorque(1000, 600, 610, 2));
    double hold = ShooterConstants.kTorqueControlHoldSecs;
    assertTrue(control.useTorque(1000, 600, 610, 2.0 + hold / 2.0));
    assertFalse(control.useTorque(1000, 600, 610, 2.0 + hold));
    assertTrue(control.useTorque(1000, 990, 995, 3));
    assertTrue(control.useTorque(1000, 600, 610, 4));
  }

  @Test
  void briefDipClearsTimerAndEitherWheelCanTriggerRecovery() {
    var control = new ShooterBangBang();
    assertTrue(control.useTorque(1000, 1000, 1000, 0));
    assertTrue(control.useTorque(1000, 1000, 600, 1));
    assertTrue(control.useTorque(1000, 1000, 1000, 1.020));
    assertTrue(control.useTorque(1000, 600, 1000, 2));
    assertFalse(
        control.useTorque(1000, 600, 1000, 2.0 + ShooterConstants.kTorqueControlHoldSecs));
  }

  @Test
  void stopAndInvalidTargetsResetMode() {
    var control = new ShooterBangBang();
    assertTrue(control.useTorque(1000, 1000, 1000, 0));
    control.reset();
    assertFalse(control.useTorque(1000, 600, 600, 1));
    assertTrue(control.useTorque(1000, 1000, 1000, 2));
    assertFalse(control.useTorque(0, 1000, 1000, 3));
    assertFalse(control.useTorque(Double.NaN, 1000, 1000, 4));
    assertFalse(control.useTorque(1000, 600, 600, 5));
  }

  @Test
  void targetChangesUseCurrentSetpointAndOverspeedStaysBounded() {
    var control = new ShooterBangBang();
    assertTrue(control.useTorque(1000, 1000, 1000, 0));
    assertTrue(control.useTorque(1500, 1000, 1000, 1));
    assertFalse(control.useTorque(1500, 1000, 1000, 1.040));
    assertTrue(control.useTorque(1500, 1500, 1500, 2));
    assertTrue(control.useTorque(1000, 1500, 1500, 3));
    assertFalse(control.useTorque(1000, 1500, 1500, 3.040));
  }
}
