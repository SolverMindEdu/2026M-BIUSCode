package frc.robot.subsystems.shooter;

/** Selects control mode; the Talon itself performs the fast bang-bang velocity loop. */
final class ShooterBangBang {
  private boolean torqueControl;
  private double outsideSince = Double.NaN;

  boolean useTorque(double targetRpm, double leftRpm, double rightRpm, double now) {
    if (!Double.isFinite(targetRpm) || targetRpm <= 0.0) {
      reset();
      return false;
    }
    boolean nearTarget =
        Math.abs(leftRpm - targetRpm) <= ShooterConstants.kTorqueControlToleranceRpm
            && Math.abs(rightRpm - targetRpm) <= ShooterConstants.kTorqueControlToleranceRpm;
    if (nearTarget) {
      torqueControl = true;
      outsideSince = Double.NaN;
    } else if (torqueControl) {
      if (Double.isNaN(outsideSince)) outsideSince = now;
      if (now - outsideSince >= ShooterConstants.kTorqueControlHoldSecs) {
        torqueControl = false;
      }
    }
    return torqueControl;
  }

  void reset() {
    torqueControl = false;
    outsideSince = Double.NaN;
  }
}
