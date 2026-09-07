// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import static frc.robot.subsystems.intake.IntakeDeployConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.PhoenixUtil;

public class IntakeDeployIOTalonFX implements IntakeDeployIO {
  private final TalonFX motor = new TalonFX(kMotorId, kCanBus);

  private final StatusSignal<?> position = motor.getPosition();
  private final StatusSignal<?> velocity = motor.getVelocity();
  private final StatusSignal<?> appliedVolts = motor.getMotorVoltage();
  private final StatusSignal<?> statorCurrent = motor.getStatorCurrent();
  private final StatusSignal<Boolean> rebooted = motor.getStickyFault_BootDuringEnable();
  private final StatusSignal<?> closedLoopReference = motor.getClosedLoopReference();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final DynamicMotionMagicVoltage positionRequest =
      new DynamicMotionMagicVoltage(0.0, kCruiseRotPerSec, kAccelRotPerSecSq);
  private final NeutralOut neutralRequest = new NeutralOut();

  public IntakeDeployIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = kStatorAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = kSupplyAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = PhoenixUtil.direction(kInverted);

    config.MotorOutput.PeakForwardDutyCycle = kPeakDeployDutyCycle;
    config.MotorOutput.PeakReverseDutyCycle = -kPeakRetractDutyCycle;

    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kG = kG;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kP = kP;
    config.Slot0.kD = kD;

    config.MotionMagic.MotionMagicCruiseVelocity = kCruiseRotPerSec;
    config.MotionMagic.MotionMagicAcceleration = kAccelRotPerSecSq;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Math.max(kRetractRotations, kDeployRotations) + kLimitMarginRotations;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Math.min(kRetractRotations, kDeployRotations) - kLimitMarginRotations;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    motor.getConfigurator().apply(config);

    motor.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, statorCurrent, closedLoopReference);
    rebooted.setUpdateFrequency(4.0);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(IntakeDeployIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(
                position, velocity, appliedVolts, statorCurrent, closedLoopReference)
            .isOK();
    inputs.positionRotations = position.getValueAsDouble();
    inputs.velocityRotPerSec = velocity.getValueAsDouble();
    inputs.setpointRotations = closedLoopReference.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.rebooted = rebooted.getValue();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPositionSetpoint(
      double rotations, double cruiseRotPerSec, double accelRotPerSecSq) {
    motor.setControl(
        positionRequest
            .withPosition(rotations)
            .withVelocity(cruiseRotPerSec)
            .withAcceleration(accelRotPerSecSq));
  }

  @Override
  public void setSoftLimitsEnabled(boolean enabled) {
    motor
        .getConfigurator()
        .apply(
            new SoftwareLimitSwitchConfigs()
                .withForwardSoftLimitThreshold(
                    Math.max(kRetractRotations, kDeployRotations) + kLimitMarginRotations)
                .withReverseSoftLimitThreshold(
                    Math.min(kRetractRotations, kDeployRotations) - kLimitMarginRotations)
                .withForwardSoftLimitEnable(enabled)
                .withReverseSoftLimitEnable(enabled));
  }

  @Override
  public void setGains(double kP, double kD, double kG, double kA) {
    var slot = new Slot0Configs();
    slot.kS = kS;
    slot.kV = kV;
    slot.kA = kA;
    slot.kG = kG;
    slot.GravityType = GravityTypeValue.Elevator_Static;
    slot.kP = kP;
    slot.kD = kD;
    motor.getConfigurator().apply(slot);
  }

  @Override
  public void stop() {
    motor.setControl(neutralRequest);
  }
}
