// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Wrist extends SubsystemBase {
  TalonFX motorFx = new TalonFX(Constants.DriveConstants.kWristMotor);
  DutyCycleOut output = new DutyCycleOut(0);
  Encoder encoder = new Encoder(1, 2);

  /** Creates a new Wrist. */
  public Wrist() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Wrist Encoder", encoder.get());
  }

  public void setSpeed(double d) {
    motorFx.setControl(output.withOutput(d));
  }

  public Command setSpeedCmd(double d) {
    return runOnce(() -> {
      setSpeed(d);
    });
  }

  public void setPosition(double d) {

  }

  public Command setPositionCmd(double d) {
    return runOnce(() -> {
      setPosition(d);
    });
  }
}
