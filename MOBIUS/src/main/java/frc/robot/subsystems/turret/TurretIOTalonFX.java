// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import static frc.robot.subsystems.turret.TurretConstants.*;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.util.PhoenixUtil;

public class TurretIOTalonFX implements TurretIO {
  private final TalonFX motor = new TalonFX(kMotorId, kCanBus);
  private final CANcoder encoder = new CANcoder(kEncoderId, kCanBus);

  private final StatusSignal<?> rotorPosition = motor.getPosition();
  private final StatusSignal<?> rotorVelocity = motor.getVelocity();
  private final StatusSignal<?> appliedVolts = motor.getMotorVoltage();
  private final StatusSignal<?> statorCurrent = motor.getStatorCurrent();
  private final StatusSignal<?> supplyCurrent = motor.getSupplyCurrent();

  private final StatusSignal<?> encoderAbsolute = encoder.getAbsolutePosition();
  private final StatusSignal<?> encoderPosition = encoder.getPosition();
  private final StatusSignal<?> encoderVelocity = encoder.getVelocity();
  private final StatusSignal<?> closedLoopReference = motor.getClosedLoopReference();

  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final MotionMagicVoltage profiledRequest = new MotionMagicVoltage(0.0);
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private final NeutralOut neutralRequest = new NeutralOut();

  public TurretIOTalonFX() {
    var encoderConfig = new CANcoderConfiguration();

    encoderConfig.MagnetSensor.MagnetOffset = 0.0;

    encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    encoder.getConfigurator().apply(encoderConfig);

    var motorConfig = new TalonFXConfiguration();

    motorConfig.CurrentLimits.StatorCurrentLimit = kCalibrationStatorAmps;
    motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfig.CurrentLimits.SupplyCurrentLimit = kCalibrationStatorAmps;
    motorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.MotorOutput.Inverted = PhoenixUtil.direction(kInverted);

    motorConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    motorConfig.Feedback.SensorToMechanismRatio = kTotalReduction;

    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = kMaxAngleDegrees / 360.0;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = kMinAngleDegrees / 360.0;
    motorConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    motorConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;

    motorConfig.Slot0.kS = kS;
    motorConfig.Slot0.kV = kV;
    motorConfig.Slot0.kA = kA;
    motorConfig.Slot0.kP = kP;
    motorConfig.Slot0.kI = kI;
    motorConfig.Slot0.kD = kD;
    motorConfig.Slot0.kG = kG;

    motorConfig.MotionMagic.MotionMagicCruiseVelocity = kCruiseVelocityRotPerSec;
    motorConfig.MotionMagic.MotionMagicAcceleration = kAccelerationRotPerSecSq;
    motorConfig.MotionMagic.MotionMagicJerk = kJerkRotPerSecCubed;

    motor.getConfigurator().apply(motorConfig);

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        rotorPosition,
        rotorVelocity,
        appliedVolts,
        statorCurrent,
        supplyCurrent,
        encoderAbsolute,
        encoderPosition,
        encoderVelocity,
        closedLoopReference);
    motor.optimizeBusUtilization();
    encoder.optimizeBusUtilization();
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    boolean motorOk =
        BaseStatusSignal.refreshAll(
                rotorPosition,
                rotorVelocity,
                appliedVolts,
                statorCurrent,
                supplyCurrent,
                closedLoopReference)
            .isOK();
    boolean encoderOk =
        BaseStatusSignal.refreshAll(encoderAbsolute, encoderPosition, encoderVelocity).isOK();

    inputs.motorConnected = motorOk;
    inputs.encoderConnected = encoderOk;

    inputs.positionDegrees = rotorPosition.getValueAsDouble() * 360.0;
    inputs.velocityDegreesPerSec = rotorVelocity.getValueAsDouble() * 360.0;
    inputs.setpointDegrees = closedLoopReference.getValueAsDouble() * 360.0;
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();

    inputs.encoderAbsoluteRotations = encoderAbsolute.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setPositionSetpoint(double degrees, double velocityDegPerSec) {
    motor.setControl(
        positionRequest
            .withPosition(degrees / 360.0)
            .withVelocity(velocityDegPerSec / 360.0));
  }

  @Override
  public void setProfiledSetpoint(double degrees) {
    motor.setControl(profiledRequest.withPosition(degrees / 360.0));
  }

  @Override
  public void stop() {
    motor.setControl(neutralRequest);
  }

  @Override
  public void seedPosition(double degrees) {
    motor.setPosition(degrees / 360.0);
  }

  @Override
  public void setSoftLimitsEnabled(boolean enabled) {
    var limits =
        new SoftwareLimitSwitchConfigs()
            .withForwardSoftLimitThreshold(kMaxAngleDegrees / 360.0)
            .withReverseSoftLimitThreshold(kMinAngleDegrees / 360.0)
            .withForwardSoftLimitEnable(enabled)
            .withReverseSoftLimitEnable(enabled);
    motor.getConfigurator().apply(limits);
  }
}
