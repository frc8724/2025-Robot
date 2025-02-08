// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  TalonFX shoulderLeft = new TalonFX(11);
  TalonFX shoulderRight = new TalonFX(12);
  TalonFX elbow = new TalonFX(13);
  TalonFX wrist = new TalonFX(14);

  final MotionMagicVoltage shoulderPosition = new MotionMagicVoltage(0);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    // configure shoulder left to follow shoulder right

    // configure PID and motion magic parameters

        
    // robot init
    var talonFXConfigs = new TalonFXConfiguration();

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 4.8;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

    shoulderRight.getConfigurator().apply(talonFXConfigs, 0.050);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPosition(double distance, double height){
    InverseKinematics.DoubleJointedArmAngles angles = InverseKinematics.getPreferredArmAngles(distance, height);

    shoulderPosition.Slot = 0;
    shoulderRight.setControl(shoulderPosition.withPosition(angles.shoulderAngleRads));
  }
}
