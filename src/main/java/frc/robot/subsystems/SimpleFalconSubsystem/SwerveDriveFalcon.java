// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SimpleFalconSubsystem;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

// import frc.robot.subsystems.MayhemTalonFX.CurrentLimit;

public class SwerveDriveFalcon extends SubsystemBase {
  private TalonFX motor;
  StatusSignal<Double> rotorVelSignal;
  StatusSignal<Double> rotorPosSignal;
  VelocityVoltage velVolt = new VelocityVoltage(0);

  private String name;

  private final double Drive1rotationTicks = 13824.0;
  final double WheelDiameterMeters = 0.102;
  final double WheelCircumferenceMeters = WheelDiameterMeters * Math.PI;

  /** Creates a new SimpleFalconSubsystem. */
  public SwerveDriveFalcon(String name, int id, boolean invert) {
    motor = new TalonFX(id);
    rotorVelSignal = motor.getRotorVelocity();
    rotorPosSignal = motor.getRotorPosition();

    motor.setInverted(invert);
    this.name = name;
    // motor.setSelectedSensorPosition(0);
    motor.setPosition(0);

    var talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // set slot 0 gains and leave every other config factory-default
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = 0.05;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
    slot0Configs.kV = 0.0;
  }

  double m_set;

  double convertMpsToTicksPer100ms(double mps) {
    return mps * Drive1rotationTicks / WheelCircumferenceMeters / 10.0;
  }

  /**
   * 
   * @param value meters per second
   */
  public void set(double value) {
    double ticksPer100ms = convertMpsToTicksPer100ms(value);
    motor.setControl(velVolt.withVelocity(ticksPer100ms));
    m_set = value;
  }

  /**
   * get meters per second of the drive wheel
   */
  public double getRotationalVelocity() {
    var ticksPerSecond = rotorVelSignal.getValue() / 0.100;
    var metersPerSecond = ticksPerSecond / Drive1rotationTicks * Math.PI * WheelDiameterMeters;
    return metersPerSecond;
  }

  // distance in meters
  public double getDistance() {
    return rotorPosSignal.getValue() / Drive1rotationTicks * WheelDiameterMeters * Math.PI;
  }

  public void reset() {
    set(0.0);
    motor.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    // SmartDashboard.putNumber(this.name + " velocity",
    // motor.getSelectedSensorVelocity());
    // SmartDashboard.putNumber(this.name + " position",
    // motor.getSelectedSensorPosition());
    // SmartDashboard.putNumber(this.name + " m_set", m_set);

  }
}
