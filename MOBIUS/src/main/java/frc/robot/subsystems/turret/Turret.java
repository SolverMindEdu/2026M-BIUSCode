// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Turret extends SubsystemBase {
  private final TurretIO io;
  private final TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();

  private boolean homed = false;

  private double lastBearingDegrees = 0.0;

  private final SysIdRoutine sysId;

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
      io.seedPosition(0.0);
      io.setSoftLimitsEnabled(kTravelMeasured);
      homed = true;
    }

    Logger.recordOutput("Turret/Homed", homed);
    Logger.recordOutput("Turret/AngleDegrees", homed ? inputs.positionDegrees : Double.NaN);
    Logger.recordOutput("Turret/ErrorDegrees", inputs.setpointDegrees - inputs.positionDegrees);
    Logger.recordOutput("Turret/AtGoal", atGoal());

    Logger.recordOutput(
        "Turret/EncoderAbsoluteDegrees", inputs.encoderAbsoluteRotations * 360.0);
  }

  public Command setAngle(DoubleSupplier bearingDegrees) {
    return setAngle(bearingDegrees, () -> 0.0);
  }

  public Command setAngle(DoubleSupplier bearingDegrees, DoubleSupplier bearingRateDegPerSec) {
    return run(() -> {
          if (!homed || !kTravelMeasured) {
            io.stop();
            return;
          }

          double desired = bearingDegrees.getAsDouble();
          var resolved =
              TurretMath.resolveSetpoint(
                  desired, inputs.positionDegrees, kMinAngleDegrees, kMaxAngleDegrees);

          boolean reachable = resolved.isPresent();
          double target =
              reachable
                  ? resolved.getAsDouble()
                  : TurretMath.clampToNearestLimit(desired, kMinAngleDegrees, kMaxAngleDegrees);

          double travel = Math.abs(target - inputs.positionDegrees);
          boolean unwrapping = travel > kProfileThresholdDegrees;
          if (unwrapping) {
            io.setProfiledSetpoint(target);
          } else {
            io.setPositionSetpoint(target, bearingRateDegPerSec.getAsDouble());
          }

          Logger.recordOutput("Turret/TargetBearingDegrees", desired);
          Logger.recordOutput("Turret/TargetAngleDegrees", target);
          Logger.recordOutput("Turret/TargetReachable", reachable);
          Logger.recordOutput("Turret/Unwrapping", unwrapping);
          Logger.recordOutput("Turret/FeedforwardDegPerSec", bearingRateDegPerSec.getAsDouble());

          Logger.recordOutput(
              "Turret/TargetBearingRateDegPerSec",
              (desired - lastBearingDegrees) / 0.02);
          lastBearingDegrees = desired;
        })
        .withName("TurretSetAngle");
  }

  public boolean atGoal() {
    return homed
        && Math.abs(inputs.positionDegrees - inputs.setpointDegrees) <= kAngleToleranceDegrees;
  }

  public Command declareZero() {
    return runOnce(
            () -> {
              io.seedPosition(0.0);
              io.setSoftLimitsEnabled(true);
              homed = true;
            })
        .ignoringDisable(true)
        .withName("TurretDeclareZero");
  }

  public boolean isHomed() {
    return homed;
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction).onlyIf(Turret::isReadyForClosedLoop);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction).onlyIf(Turret::isReadyForClosedLoop);
  }

  private static boolean isReadyForClosedLoop() {
    return kTravelMeasured;
  }

  public Command jog(DoubleSupplier voltsSupplier) {
    return run(() -> {
          double commanded =
              MathUtil.applyDeadband(voltsSupplier.getAsDouble(), 0.08) * kCalibrationMaxVolts;
          io.setVoltage(MathUtil.clamp(commanded, -kCalibrationMaxVolts, kCalibrationMaxVolts));
        })
        .finallyDo(io::stop)
        .withName("TurretJog");
  }
}
