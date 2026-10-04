// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeDeployConstants.*;

import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class IntakeDeploy extends SubsystemBase {
  private final IntakeDeployIO io;
  private final IntakeDeployIOInputsAutoLogged inputs = new IntakeDeployIOInputsAutoLogged();

  private double commandedRotations = Double.NaN;

  private int stallLoops = 0;
  private boolean settled = false;
  private double settledRotations = 0.0;
  private double lastCommandedRotations = Double.NaN;
  private boolean atHome = false;

  public IntakeDeploy(IntakeDeployIO io) {
    this.io = io;
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                Volts.of(1.0).per(Second),
                Volts.of(5.0),
                null,
                state -> Logger.recordOutput("IntakeDeploy/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(output -> io.setVoltage(output.in(Volts)), null, this));

    io.setSoftLimitsEnabled(isTravelMeasured());

    setDefaultCommand(
        isTravelMeasured()
            ? retract().withName("IntakeRetractHold")
            : run(io::stop).withName("IntakeHoldForCalibration"));
  }

  private static final LoggedTunableNumber tunableP =
      new LoggedTunableNumber("IntakeDeploy/kP", kP);
  private static final LoggedTunableNumber tunableD =
      new LoggedTunableNumber("IntakeDeploy/kD", kD);
  private static final LoggedTunableNumber tunableG =
      new LoggedTunableNumber("IntakeDeploy/kG", kG);
  private static final LoggedTunableNumber tunableA =
      new LoggedTunableNumber("IntakeDeploy/kA", kA);

  @Override
  public void periodic() {
    LoggedTunableNumber.ifChanged(
        hashCode(),
        v -> io.setGains(v[0], v[1], v[2], v[3]),
        tunableP,
        tunableD,
        tunableG,
        tunableA);
    io.updateInputs(inputs);
    Logger.processInputs("IntakeDeploy", inputs);

    updateSettle();
    Logger.recordOutput(
        "IntakeDeploy/ErrorRotations", inputs.setpointRotations - inputs.positionRotations);
    Logger.recordOutput("IntakeDeploy/AtGoal", atGoal());

    Logger.recordOutput(
        "IntakeDeploy/PivotDegrees", inputs.positionRotations / kGearRatio * 360.0);
  }

  public static boolean isTravelMeasured() {
    return kDeployRotations != kRetractRotations;
  }

  private void updateSettle() {
    // Both directions push hard into a hard stop, and neither may lean on it forever: at kP = 25
    // a tenth of a rotation of standing error is 2.5 V into a motor that cannot move. After a
    // stall the setpoint becomes where it actually got to, which drops the error, and the output
    // with it, to zero. A ball knocking it off that position still gets pushed back.
    double shortfall = Math.abs(commandedRotations - inputs.positionRotations);
    boolean deploying = commandedRotations > inputs.positionRotations;
    double window = deploying ? kDeploySettleWindowRotations : kSettleWindowRotations;
    boolean stalled =
        !Double.isNaN(commandedRotations)
            && !settled
            && inputs.statorCurrentAmps > kCollisionCurrentAmps
            && Math.abs(inputs.velocityRotPerSec) < kCollisionVelocityRotPerSec
            && shortfall < window;
    stallLoops = stalled ? stallLoops + 1 : 0;
    if (stallLoops >= kSettleLoops) {
      settledRotations = inputs.positionRotations;
      settled = true;
      stallLoops = 0;
      // How far short it gave up. If a stall is not settling, this is the number the window has
      // to clear.
      Logger.recordOutput("IntakeDeploy/SettledShortfallRotations", shortfall);
    }
    // Clear on any new command: otherwise one stall latches the intake at wherever it stopped and
    // every later move aims there instead of at the setpoint.
    if (commandedRotations != lastCommandedRotations) {
      settled = false;
    }
    lastCommandedRotations = commandedRotations;

    Logger.recordOutput("IntakeDeploy/Rebooted", inputs.rebooted);
    Logger.recordOutput("IntakeDeploy/AtHome", atHome);
    Logger.recordOutput("IntakeDeploy/Settled", settled);
    Logger.recordOutput("IntakeDeploy/StallLoops", stallLoops);
    Logger.recordOutput("IntakeDeploy/ShortfallRotations", shortfall);
    Logger.recordOutput("IntakeDeploy/CommandedRotations", commandedRotations);
  }

  private double effectiveTarget(double target) {
    commandedRotations = target;
    return settled ? settledRotations : target;
  }

  public Command deploy() {
    return runOnce(() -> atHome = false)
        .andThen(run(() ->
            io.setPositionSetpoint(
                effectiveTarget(kDeployRotations), kCruiseRotPerSec, kAccelRotPerSecSq)))
        .withName("IntakeDeploy");
  }

  public Command retract() {
    return run(() -> {
          commandedRotations = kRetractRotations;
          if (Math.abs(inputs.positionRotations - kRetractRotations) <= kRetractHomeRotations) {
            atHome = true;
          }
          if (atHome) {
            // Home on a linear intake means resting on its stop. Holding a setpoint there just
            // grinds; the brake neutral mode keeps it put without drawing current.
            io.stop();
          } else {
            io.setPositionSetpoint(
                effectiveTarget(kRetractRotations),
                kRetractCruiseRotPerSec,
                kRetractAccelRotPerSecSq);
          }
        })
        .withName("IntakeRetract");
  }

  /** Walks the intake home over about 3 s while shooting, keeping fuel moving without bobbing. */
  public Command shootStow() {
    return run(() ->
            io.setPositionSetpoint(
                effectiveTarget(kRetractRotations),
                kShootStowCruiseRotPerSec,
                kShootStowAccelRotPerSecSq))
        .beforeStarting(() -> io.setStatorLimit(kShootStowStatorAmps))
        .finallyDo(() -> io.setStatorLimit(kStatorAmps))
        .withName("IntakeShootStow");
  }

  public boolean isConnected() {
    return inputs.connected;
  }

  public boolean atGoal() {
    return !Double.isNaN(commandedRotations)
        && Math.abs(commandedRotations - inputs.positionRotations) <= kToleranceRotations;
  }

  private final SysIdRoutine sysId;

  /** Ramps slowly across the travel to identify kS, kV and kA. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return runOnce(() -> commandedRotations = Double.NaN).andThen(sysId.quasistatic(direction));
  }

  /** Steps voltage to separate kA from the rest. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return runOnce(() -> commandedRotations = Double.NaN).andThen(sysId.dynamic(direction));
  }

  public Command jog(DoubleSupplier demand) {
    return runOnce(() -> commandedRotations = Double.NaN)
        .andThen(run(() ->
            io.setVoltage(
                MathUtil.clamp(
                    MathUtil.applyDeadband(demand.getAsDouble(), 0.08) * kJogVolts,
                    -kJogVolts,
                    kJogVolts))))
        .finallyDo(io::stop)
        .withName("IntakeJog");
  }
  /** What the battery pays for this mechanism, for PowerMonitor. */
  public double supplyCurrentAmps() {
    return inputs.supplyCurrentAmps;
  }

}
