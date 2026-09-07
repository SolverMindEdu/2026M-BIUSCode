// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeRollerConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.PhoenixUtil;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX motor =
      PhoenixUtil.configureRoller(kMotorId, kCanBus, kInverted, kStatorAmps, kSupplyAmps);

  private final StatusSignal<?> appliedVolts = motor.getMotorVoltage();
  private final StatusSignal<?> statorCurrent = motor.getStatorCurrent();
  private final StatusSignal<?> velocity = motor.getVelocity();

  private final DutyCycleOut request = new DutyCycleOut(0.0);

  public IntakeRollerIOTalonFX() {
    PhoenixUtil.publishRollerSignals(motor, appliedVolts, statorCurrent, velocity);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(appliedVolts, statorCurrent, velocity).isOK();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.velocityRotPerSec = velocity.getValueAsDouble();
  }

  @Override
  public void setPercent(double percent) {
    motor.setControl(request.withOutput(percent));
  }

  @Override
  public void stop() {
    motor.setControl(request.withOutput(0.0));
  }
}
