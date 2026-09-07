// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static frc.robot.subsystems.hood.HoodConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  private final HoodIO io;
  private final HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  public Hood(HoodIO io) {
    this.io = io;
    setDefaultCommand(setAngle(kStowDegrees).withName("HoodStow"));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
    Logger.recordOutput("Hood/AtGoal", atGoal(kDeployDegrees));
    Logger.recordOutput("Hood/TravelMeasured", kTravelMeasured);
  }

  public Command setAngle(double degrees) {
    return setAngle(() -> degrees);
  }

  public Command setAngle(DoubleSupplier degreesSupplier) {
    return run(() -> {
          if (kTravelMeasured) {
            io.setAngle(degreesSupplier.getAsDouble());
          } else {
            io.stop();
          }
        })
        .withName("HoodSetAngle");
  }

  public Command jog(DoubleSupplier demand) {
    return run(() ->
            io.setVoltage(
                MathUtil.clamp(
                    MathUtil.applyDeadband(demand.getAsDouble(), 0.08) * kJogVolts,
                    -kJogVolts,
                    kJogVolts)))
        .finallyDo(io::stop)
        .withName("HoodJog");
  }

  public boolean atGoal(double degrees) {
    return Math.abs(inputs.positionDegrees - degrees) <= kToleranceDegrees;
  }
}
