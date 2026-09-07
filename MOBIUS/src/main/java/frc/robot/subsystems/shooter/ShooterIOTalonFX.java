// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX left = new TalonFX(kLeftId, kCanBus);
  private final TalonFX right = new TalonFX(kRightId, kCanBus);

  private final StatusSignal<?> leftVelocity = left.getVelocity();
  private final StatusSignal<?> rightVelocity = right.getVelocity();
  private final StatusSignal<?> leftVolts = left.getMotorVoltage();
  private final StatusSignal<?> rightVolts = right.getMotorVoltage();
  private final StatusSignal<?> leftSupply = left.getSupplyVoltage();
  private final StatusSignal<?> leftCurrent = left.getStatorCurrent();
  private final StatusSignal<?> rightCurrent = right.getStatorCurrent();

  private final VelocityTorqueCurrentFOC velocityRequest = new VelocityTorqueCurrentFOC(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();

  public ShooterIOTalonFX() {
    var config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = kStatorAmps;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = kSupplyAmps;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Feedback.SensorToMechanismRatio = kGearRatio;

    config.Slot0.kS = kS;
    config.Slot0.kV = kV;
    config.Slot0.kP = kP;

    config.TorqueCurrent.PeakForwardTorqueCurrent = kPeakTorqueAmps;
    config.TorqueCurrent.PeakReverseTorqueCurrent = 0.0;
    config.MotorOutput.PeakReverseDutyCycle = 0.0;

    config.MotorOutput.Inverted = PhoenixUtil.direction(kLeftInverted);
    left.getConfigurator().apply(config);

    config.MotorOutput.Inverted = PhoenixUtil.direction(kRightInverted);
    right.getConfigurator().apply(config);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftVelocity,
        rightVelocity,
        leftVolts,
        rightVolts,
        leftCurrent,
        rightCurrent,
        leftSupply);
    left.optimizeBusUtilization();
    right.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.leftConnected =
        BaseStatusSignal.refreshAll(
                leftVelocity, leftVolts, leftCurrent, leftSupply)
            .isOK();
    inputs.rightConnected =
        BaseStatusSignal.refreshAll(rightVelocity, rightVolts, rightCurrent).isOK();

    inputs.leftRpm = leftVelocity.getValueAsDouble() * 60.0;
    inputs.rightRpm = rightVelocity.getValueAsDouble() * 60.0;
    inputs.leftAppliedVolts = leftVolts.getValueAsDouble();
    inputs.rightAppliedVolts = rightVolts.getValueAsDouble();
    inputs.leftSupplyVolts = leftSupply.getValueAsDouble();
    inputs.leftStatorCurrentAmps = leftCurrent.getValueAsDouble();
    inputs.rightStatorCurrentAmps = rightCurrent.getValueAsDouble();
  }

  @Override
  public void setRpm(double rpm) {
    var request = velocityRequest.withVelocity(rpm / 60.0);
    left.setControl(request);
    right.setControl(request);
  }

  @Override
  public void setGains(double kP, double peakTorqueAmps) {
    var slot = new Slot0Configs();
    slot.kS = kS;
    slot.kV = kV;
    slot.kP = kP;
    left.getConfigurator().apply(slot);
    right.getConfigurator().apply(slot);
    var torque = new TorqueCurrentConfigs();
    torque.PeakForwardTorqueCurrent = peakTorqueAmps;
    torque.PeakReverseTorqueCurrent = 0.0;
    left.getConfigurator().apply(torque);
    right.getConfigurator().apply(torque);
  }

  @Override
  public void stop() {
    left.setControl(neutralRequest);
    right.setControl(neutralRequest);
  }
}
