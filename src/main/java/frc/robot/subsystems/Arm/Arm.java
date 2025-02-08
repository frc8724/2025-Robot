// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  TalonFX shoulderLeft = new TalonFX(Constants.DriveConstants.kShoulderLeftMotor);
  TalonFX shoulderRight = new TalonFX(Constants.DriveConstants.kShoulderRightMotor);
  TalonFX elbow = new TalonFX(Constants.DriveConstants.kElbowMotor);

  AnalogInput shoulderEncoder = new AnalogInput(5);
  AnalogInput elbowEncoder = new AnalogInput(4);

  DutyCycleOut output = new DutyCycleOut(0);

  PIDController elbowPid = new PIDController(0.0009, 0, 0);
  PIDController shoulderPid = new PIDController(0.001, 0, 0);

  boolean elbowPosMode = false;
  boolean shoulderPosMode = false;

  double elbowSetpoint;
  double shoulderSetpoint;

  final double ElbowEncoderOffset = 0;
  final double ShoulderEncoderOffset = 0;

  final double ElbowEncoderTicksPerRevolution = 4096;
  final double ElbowEncoderTickeAtVertical = 4011;
  final double ShoulderEncoderTickeAtVertical = 4030;

  /** Creates a new Arm. */
  public Arm() {
    elbowPid.setTolerance(30, 100);
    shoulderPid.setTolerance(30, 100);

    elbowPid.enableContinuousInput(0, 4095);
    shoulderPid.enableContinuousInput(0, 4095);

    shoulderLeft.setControl(new Follower(Constants.DriveConstants.kShoulderRightMotor, true));

    var shoulderConfig = new TalonFXConfiguration();
    shoulderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    shoulderLeft.getConfigurator().apply(shoulderConfig);
    shoulderRight.getConfigurator().apply(shoulderConfig);

    var elbowConfig = new TalonFXConfiguration();
    elbowConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    elbowConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    elbow.getConfigurator().apply(elbowConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    double elbowPower = elbowPid.calculate(elbowEncoder.getValue(), elbowSetpoint);
    double shoulderPower = shoulderPid.calculate(shoulderEncoder.getValue(), shoulderSetpoint);

    elbowPower = MathUtil.clamp(elbowPower, -.5, .5);
    shoulderPower = MathUtil.clamp(shoulderPower, -.5, .5);

    if (elbowPosMode) {
      elbow.setControl(output.withOutput(elbowPower));
    }

    if (shoulderPosMode) {
      shoulderRight.setControl(output.withOutput(shoulderPower));
    }

    SmartDashboard.putNumber("Shoulder Encoder", shoulderEncoder.getValue());
    SmartDashboard.putNumber("Elbow Encoder", elbowEncoder.getValue());
    SmartDashboard.putNumber("Elbow Position Power", elbowPower);
    SmartDashboard.putNumber("Elbow Position Setpoint", elbowSetpoint);
    SmartDashboard.putNumber("Shoulder Position Power", shoulderPower);
    SmartDashboard.putNumber("Shoulder Position Setpoint", shoulderSetpoint);
  }

  public void setShoulderSpeed(double d) {
    shoulderPosMode = false;
    shoulderRight.setControl(output.withOutput(d));
  }

  public Command setShoulderSpeedCmd(double d) {
    return runOnce(() -> {
      setShoulderSpeed(d);
    });
  }

  public void setElbowSpeed(double d) {
    elbowPosMode = false;
    elbow.setControl(output.withOutput(d));
  }

  public Command setElbowSpeedCmd(double d) {
    return runOnce(() -> {
      setElbowSpeed(d);
    });
  }

  public void setElbowPosition(double d) {
    elbowPosMode = true;
    elbowSetpoint = d;
  }

  public Command setElbowPositionCmd(double d) {
    return runOnce(() -> {
      setElbowPosition(d);
    });
  }

  public void setShoulderPosition(double d) {
    shoulderPosMode = true;

    if (d > 130 && d < 2048) {
      d = 130;
    }
    if (d < 3800 && d > 2048) {
      d = 3800;
    }
    shoulderSetpoint = d;
  }

  public Command setShoulderPositionCmd(double d) {
    return runOnce(() -> {
      setShoulderPosition(d);
    });
  }

  double convertRadianToEncoder(double x, double offset) {
    return x / (2 * Math.PI) * 4095 + offset;
  }

  public Command setArmPositionCmd(double x, double y) {
    var angles = InverseKinematics.getPreferredArmAngles(x, y);

    double elbowEncoderCounts = convertRadianToEncoder(angles.elbowAngleRads, ElbowEncoderOffset);
    double shoulderEncoderCounts = convertRadianToEncoder(angles.shoulderAngleRads, ShoulderEncoderOffset);

    return runOnce(() -> {
      setShoulderPosition(shoulderEncoderCounts);
      setElbowPosition(elbowEncoderCounts);
    });
  }
}
