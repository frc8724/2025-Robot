// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  TalonFX shoulderLeft = new TalonFX(11);
  TalonFX shoulderRight = new TalonFX(12);
  TalonFX elbow = new TalonFX(13);
  TalonFX wrist = new TalonFX(14);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
