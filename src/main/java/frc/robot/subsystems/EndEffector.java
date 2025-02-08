// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {
  TalonFX motorFx = new TalonFX(Constants.DriveConstants.kEndEffector);
  DutyCycleOut output = new DutyCycleOut(0);
  DigitalInput limitSwitch = new DigitalInput(0);

  /** Creates a new EndEffector. */
  public EndEffector() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // if the limit switch is pressed, stop the motor.
    if (!limitSwitch.get()) {
      setSpeed(0);
    }

    SmartDashboard.putBoolean("EndEffector Limit Switch", limitSwitch.get());
  }

  public void setSpeed(double d) {
    motorFx.setControl(output.withOutput(d));
  }

  public Command setSpeedCmd(double d) {
    return runOnce(() -> {
      setSpeed(d);
    });
  }
}
