// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  /** A goal already in the turret's frame: where to point, and how fast that point is moving. */
  public record Goal(double angleDegrees, double velocityDegPerSec) {}

  /** Only the first step's guess, before there are two timestamps to subtract. */
  private static final double kNominalLoopPeriodSecs = 0.02;

  private static final TrapezoidProfile profile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              kCruiseVelocityRotPerSec * 360.0, kAccelerationRotPerSecSq * 360.0));

  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  private final SysIdRoutine sysId;

  private boolean homed = false;
  private boolean visionConfident = true;
  private boolean recentre = false;
  private double lastGoalDegrees = Double.NaN;
  private double heldGoalDegrees = Double.NaN;
  private TrapezoidProfile.State setpoint = null;
  private boolean unwrapping = false;
  private double unwrapStartSecs = Double.NaN;
  private double unwrapSettledSinceSecs = Double.NaN;
  private double lastProfileSecs = Double.NaN;

  public Turret(TurretIO io) {
    this.io = io;
    io.setSoftLimitsEnabled(false);

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(0.5).per(Second),
                Volts.of(2.0),
                Seconds.of(6.0),
                state -> Logger.recordOutput("Turret/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(output -> io.setVoltage(output.in(Volts)), null, this));

    setDefaultCommand(run(io::stop).withName("TurretHold"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);

    if (!homed && DriverStation.isEnabled()) {
      io.seedPosition(kHomeAngleDegrees);
      io.setSoftLimitsEnabled(kTravelMeasured);
      homed = true;
    }

    Logger.recordOutput("Turret/Homed", homed);
    Logger.recordOutput("Turret/SoftGains", !visionConfident);
    Logger.recordOutput("Turret/Recentring", recentre);
    Logger.recordOutput("Turret/Unwrapping", unwrapping);
    Logger.recordOutput("Turret/Settled", isSettled());
    Logger.recordOutput("Turret/AngleDegrees", homed ? inputs.positionDegrees : Double.NaN);
    Logger.recordOutput(
        "Turret/ErrorDegrees",
        Double.isNaN(lastGoalDegrees) ? Double.NaN : lastGoalDegrees - inputs.positionDegrees);
    Logger.recordOutput("Turret/AtGoal", atGoal());
  }

  /**
   * 6328's tracking loop: pick the wrap nearest the last goal inside the active band, then run one
   * trapezoid profile toward it carrying the goal velocity, and command position plus velocity.
   */
  public Command track(Supplier<Goal> goal, BooleanSupplier enabled) {
    return run(() -> {
          if (!enabled.getAsBoolean() || !homed || !kTravelMeasured) {
            io.stop();
            lastGoalDegrees = Double.NaN;
            heldGoalDegrees = Double.NaN;
            setpoint = null;
            unwrapping = false;
            unwrapSettledSinceSecs = Double.NaN;
            lastProfileSecs = Double.NaN;
            return;
          }

          Goal g = goal.get();
          double reference =
              Double.isNaN(lastGoalDegrees) ? inputs.positionDegrees : lastGoalDegrees;
          var resolved =
              TurretMath.resolveSetpoint(
                  g.angleDegrees(),
                  inputs.positionDegrees,
                  kMinAngleDegrees,
                  kMaxAngleDegrees,
                  reference);
          double best =
              resolved.isPresent()
                  ? resolved.getAsDouble()
                  : TurretMath.clampToNearestLimit(
                      g.angleDegrees(), kMinAngleDegrees, kMaxAngleDegrees);

          // Away from the goal and not shooting, take the unwrap now rather than on arrival, but
          // only when the middle of the travel is enough better to be worth the sweep.
          if (recentre) {
            var centred =
                TurretMath.resolveSetpoint(
                    g.angleDegrees(),
                    inputs.positionDegrees,
                    kMinAngleDegrees,
                    kMaxAngleDegrees,
                    kCentreAngleDegrees);
            if (centred.isPresent()
                && Math.abs(centred.getAsDouble() - kCentreAngleDegrees) + kRecentreGainDegrees
                    < Math.abs(best - kCentreAngleDegrees)) {
              best = centred.getAsDouble();
            }
          }
          // Hold the goal until it moves enough to be worth moving for; see kGoalDeadbandDegrees.
          double rawGoal = best;
          if (!Double.isNaN(heldGoalDegrees)
              && Math.abs(best - heldGoalDegrees) < kGoalDeadbandDegrees) {
            best = heldGoalDegrees;
          }
          // A goal that moves most of a turn in one loop is the wrap changing under the turret, not
          // the target moving. Hold the feed off until the sweep has landed on the new wrap.
          if (!Double.isNaN(lastGoalDegrees)
              && Math.abs(best - lastGoalDegrees) > kUnwrapDetectDegrees) {
            unwrapping = true;
            unwrapSettledSinceSecs = Double.NaN;
            unwrapStartSecs = Timer.getFPGATimestamp();
          }
          heldGoalDegrees = best;

          if (unwrapping) {
            double now = Timer.getFPGATimestamp();
            // Against the PREVIOUS goal, not this loop's. periodic() samples the position before
            // this body runs, so the turret can only ever have reached the goal it was given last
            // loop; comparing it against this loop's goal builds in a fixed lag of one loop of
            // travel -- 3.6 degrees at a 180 deg/s chase, more when faster -- and a settle band
            // tighter than that could never be met however well the turret was tracking.
            double reached = Double.isNaN(lastGoalDegrees) ? best : lastGoalDegrees;
            if (Math.abs(inputs.positionDegrees - reached) <= kUnwrapSettledDegrees) {
              if (Double.isNaN(unwrapSettledSinceSecs)) {
                unwrapSettledSinceSecs = now;
              } else if (now - unwrapSettledSinceSecs >= kUnwrapSettledSecs) {
                unwrapping = false;
              }
            } else {
              unwrapSettledSinceSecs = Double.NaN;
            }
            // Never let a turret that will not quite settle hold the feed off indefinitely. Both
            // this and the settle window are wall-clock: a loop-counted timeout stretches with
            // loop time, so an overrunning robot would hold the feed off longest.
            if (now - unwrapStartSecs >= kUnwrapTimeoutSecs) {
              unwrapping = false;
            }
          }

          lastGoalDegrees = best;

          driveTo(best, g.velocityDegPerSec());

          Logger.recordOutput("Turret/GoalDegrees", best);
          Logger.recordOutput("Turret/GoalRawDegrees", rawGoal);
          Logger.recordOutput("Turret/GoalDegPerSec", g.velocityDegPerSec());
        })
        .withName("TurretTrack");
  }

  /**
   * Drive back to the home angle and finish there, which hands the turret straight back to the
   * tracking default command. The wrap reference is left at home, so tracking then resumes on the
   * wrap nearest zero rather than the one it was on before.
   */
  public Command goToZero() {
    return run(() -> {
          if (!homed || !kTravelMeasured) {
            io.stop();
            return;
          }
          lastGoalDegrees = kHomeAngleDegrees;
          heldGoalDegrees = kHomeAngleDegrees;
          // A deliberate recall is not an unwrap, and tracking resumes from home afterwards.
          unwrapping = false;
          unwrapSettledSinceSecs = Double.NaN;
          driveTo(kHomeAngleDegrees, 0.0);
          Logger.recordOutput("Turret/GoalDegrees", kHomeAngleDegrees);
          Logger.recordOutput("Turret/GoalRawDegrees", kHomeAngleDegrees);
          Logger.recordOutput("Turret/GoalDegPerSec", 0.0);
        })
        .until(
            () ->
                !homed
                    || !kTravelMeasured
                    || Math.abs(inputs.positionDegrees - kHomeAngleDegrees)
                        <= kZeroToleranceDegrees)
        .withTimeout(kZeroTimeoutSecs)
        .withName("TurretGoToZero");
  }

  /** One step of the shared trapezoid toward a goal, with the RIO-side kS deadband. */
  private void driveTo(double goalDegrees, double goalVelocityDegPerSec) {
    if (setpoint == null) {
      setpoint = new TrapezoidProfile.State(inputs.positionDegrees, inputs.velocityDegreesPerSec);
    }
    // Real elapsed time, not the nominal period. The RIO loop runs well over 20 ms under load, and
    // a profile fed a constant 20 ms advances slower than the clock -- at a 35 ms loop it plans at
    // 57 percent of real speed, so every move takes almost twice as long as designed. Clamped so a
    // stall or a first step cannot jump the setpoint somewhere the turret was never asked to go.
    double now = Timer.getFPGATimestamp();
    double dt =
        Double.isNaN(lastProfileSecs)
            ? kNominalLoopPeriodSecs
            : MathUtil.clamp(now - lastProfileSecs, kMinProfileStepSecs, kMaxProfileStepSecs);
    lastProfileSecs = now;
    Logger.recordOutput("Turret/ProfileStepSecs", dt);
    setpoint =
        profile.calculate(
            dt,
            setpoint,
            new TrapezoidProfile.State(goalDegrees, goalVelocityDegPerSec));
    // The goal carries the goal's own velocity, so the profile plans to arrive still moving and
    // sails past a goal pinned to a limit. Hold it at the travel the soft limits enforce, and drop
    // the velocity that was driving it out, so the setpoint cannot wind up somewhere the mechanism
    // is not allowed to follow.
    if (setpoint.position > kMaxAngleDegrees) {
      setpoint = new TrapezoidProfile.State(kMaxAngleDegrees, Math.min(0.0, setpoint.velocity));
    } else if (setpoint.position < kMinAngleDegrees) {
      setpoint = new TrapezoidProfile.State(kMinAngleDegrees, Math.max(0.0, setpoint.velocity));
    }
    double error = setpoint.position - inputs.positionDegrees;
    double staticVolts = Math.abs(error) > kStaticDeadbandDegrees ? Math.copySign(kS, error) : 0.0;
    io.setPositionSetpoint(setpoint.position, setpoint.velocity, !visionConfident, staticVolts);
    Logger.recordOutput("Turret/StaticVolts", staticVolts);
    Logger.recordOutput("Turret/SetpointDegrees", setpoint.position);
    Logger.recordOutput("Turret/SetpointDegPerSec", setpoint.velocity);
  }

  /** True while driving back, when an unwrap costs nothing; false once a shot is in progress. */
  public void setRecentre(boolean recentre) {
    this.recentre = recentre;
  }

  /** False in a blind spot, which drops the turret to its softer gain slot. */
  public void setVisionConfident(boolean visionConfident) {
    this.visionConfident = visionConfident;
  }

  /**
   * False while the turret is sweeping to a new wrap and until it has arrived on it. Fuel fed
   * during that sweep is thrown away, so the feed gate holds off while this is false and resumes
   * once the turret is back on target.
   */
  public boolean isSettled() {
    return !unwrapping;
  }

  /** Physically within the feeding tolerance of the wrap being chased. */
  public boolean atGoal() {
    return atGoal(kAngleToleranceDegrees);
  }

  public boolean atGoal(double toleranceDegrees) {
    return homed
        && !Double.isNaN(lastGoalDegrees)
        && Math.abs(inputs.positionDegrees - lastGoalDegrees) <= toleranceDegrees;
  }

  public boolean isConnected() {
    return inputs.motorConnected && inputs.encoderConnected;
  }

  public boolean isHomed() {
    return homed;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction).onlyIf(() -> kTravelMeasured);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction).onlyIf(() -> kTravelMeasured);
  }
  /** What the battery pays for this mechanism, for PowerMonitor. */
  public double supplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

}
