// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeDeployConstants.*;

import edu.wpi.first.math.MathUtil;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class IntakeDeploy extends SubsystemBase {
  private final IntakeDeployIO io;
  private final IntakeDeployIOInputsAutoLogged inputs = new IntakeDeployIOInputsAutoLogged();

  private double commandedRotations = Double.NaN;

  private int retractStallLoops = 0;
  private boolean retractSettled = false;
  private double retractSettledRotations = 0.0;
  private double lastCommandedRotations = Double.NaN;

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

    updateRetractSettle();
    Logger.recordOutput(
        "IntakeDeploy/ErrorRotations", inputs.setpointRotations - inputs.positionRotations);
    Logger.recordOutput("IntakeDeploy/AtGoal", atGoal());

    Logger.recordOutput(
        "IntakeDeploy/PivotDegrees", inputs.positionRotations / kGearRatio * 360.0);
  }

  public static boolean isTravelMeasured() {
    return kDeployRotations != kRetractRotations;
  }

  private void updateRetractSettle() {
    boolean deploying = commandedRotations > inputs.positionRotations;

    // Retracting pushes hard, but must not lean on a stop forever. After a stall it accepts where
    // it got to, which drops the error to zero and the output with it.
    boolean stalled =
        !deploying
            && !Double.isNaN(commandedRotations)
            && !retractSettled
            && inputs.statorCurrentAmps > kCollisionCurrentAmps
            && Math.abs(inputs.velocityRotPerSec) < kCollisionVelocityRotPerSec
            && Math.abs(commandedRotations - inputs.positionRotations)
                < kRetractSettleWindowRotations;
    retractStallLoops = stalled ? retractStallLoops + 1 : 0;
    if (retractStallLoops >= kRetractSettleLoops) {
      retractSettledRotations = inputs.positionRotations;
      retractSettled = true;
      retractStallLoops = 0;
    }
    // Clear on any NEW command, not just a deploy: otherwise one stall latches the intake at
    // wherever it stopped and every later retract aims there instead of at the setpoint.
    if (deploying || commandedRotations != lastCommandedRotations) {
      retractSettled = false;
    }
    lastCommandedRotations = commandedRotations;

    Logger.recordOutput("IntakeDeploy/Rebooted", inputs.rebooted);
    Logger.recordOutput("IntakeDeploy/RetractSettled", retractSettled);
    Logger.recordOutput("IntakeDeploy/CommandedRotations", commandedRotations);
  }

  private double effectiveTarget(double target) {
    commandedRotations = target;
    return retractSettled ? retractSettledRotations : target;
  }

  public Command deploy() {
    return run(() ->
            io.setPositionSetpoint(
                effectiveTarget(kDeployRotations), kCruiseRotPerSec, kAccelRotPerSecSq))
        .withName("IntakeDeploy");
  }

  public Command retract() {
    return run(() ->
            io.setPositionSetpoint(
                effectiveTarget(kRetractRotations),
                kRetractCruiseRotPerSec,
                kRetractAccelRotPerSecSq))
        .withName("IntakeRetract");
  }

  private Command pulseIn() {
    return run(() ->
            io.setPositionSetpoint(
                effectiveTarget(kShootPulseLowRotations), kCruiseRotPerSec, kAccelRotPerSecSq))
        .withName("IntakeShootPulseIn");
  }

  /** Pulses just off the deployed position to keep fuel moving, from the first loop of a shot. */
  public Command shootPulse() {
    return Commands.repeatingSequence(
            deploy().withTimeout(kShootPulsePeriodSecs),
            pulseIn().withTimeout(kShootPulsePeriodSecs))
        .withName("IntakeShootPulse");
  }

  public boolean atGoal() {
    return Math.abs(inputs.setpointRotations - inputs.positionRotations) <= kToleranceRotations;
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
}
