// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  TalonFX motorFx = new TalonFX(Constants.DriveConstants.kWristMotor);
  DutyCycleOut output = new DutyCycleOut(0);
  Encoder encoder = new Encoder(1, 2);

  PIDController pid = new PIDController(0.01, 0, 0);
  boolean positionMode;
  double positionSetpoint;

  final double Plus_90_Degrees = 233;
  final double Minus_90_Degrees = -802;
  final double EncoderTicksPerRevolution = 2048;

  /** Creates a new Wrist. */
  public Wrist() {
    pid.setTolerance(5, 10);

    var config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    motorFx.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    double wristPower = pid.calculate(encoder.get(), positionSetpoint);

    wristPower = MathUtil.clamp(wristPower, -.5, .5);

    if (positionMode) {
      motorFx.setControl(output.withOutput(wristPower));
    }

    SmartDashboard.putNumber("Wrist Encoder", encoder.get());
    SmartDashboard.putNumber("Wrist Position Power", wristPower);
  }

  public void setSpeed(double d) {
    positionMode = false;
    motorFx.setControl(output.withOutput(d));
  }

  public Command setSpeedCmd(double d) {
    return runOnce(() -> {
      setSpeed(d);
    });
  }

  public void setPosition(double d) {
    positionMode = true;
    positionSetpoint = d;
  }

  public Command setPositionCmd(double d) {
    return runOnce(() -> {
      setPosition(d);
    });
  }
}
