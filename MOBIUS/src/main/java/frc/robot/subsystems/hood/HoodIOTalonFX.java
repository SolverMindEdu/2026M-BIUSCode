// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import static frc.robot.subsystems.hood.HoodConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.PhoenixUtil;

public class HoodIOTalonFX implements HoodIO {
  private final TalonFX motor = new TalonFX(kMotorId, kCanBus);

  private final StatusSignal<?> position = motor.getPosition();
  private final StatusSignal<?> velocity = motor.getVelocity();
  private final StatusSignal<?> appliedVolts = motor.getMotorVoltage();
  private final StatusSignal<?> statorCurrent = motor.getStatorCurrent();

  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();
  private final VoltageOut voltageRequest = new VoltageOut(0.0);

  public HoodIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = kStatorAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = kSupplyAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = PhoenixUtil.direction(kInverted);

    config.Feedback.SensorToMechanismRatio = kGearRatio;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        (kMaxDegrees - kLimitMarginDegrees) / 360.0;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        (kMinDegrees - kLimitMarginDegrees) / 360.0;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = kTravelMeasured;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = kTravelMeasured;

    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kA = kA;
    config.Slot0.kG = kG;
    config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
    config.Slot0.kP = kP;
    config.MotionMagic.MotionMagicCruiseVelocity = kCruiseRotPerSec;
    config.MotionMagic.MotionMagicAcceleration = kAccelRotPerSecSq;
    config.Slot0.kD = kD;

    motor.getConfigurator().apply(config);

    motor.setPosition(0.0);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, statorCurrent);
    motor.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    inputs.connected =
        BaseStatusSignal.refreshAll(position, velocity, appliedVolts, statorCurrent).isOK();
    inputs.positionDegrees = position.getValueAsDouble() * 360.0;
    inputs.velocityDegreesPerSec = velocity.getValueAsDouble() * 360.0;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
  }

  @Override
  public void setAngle(double degrees) {
    motor.setControl(positionRequest.withPosition(degrees / 360.0));
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void stop() {
    motor.setControl(neutralRequest);
  }
}
