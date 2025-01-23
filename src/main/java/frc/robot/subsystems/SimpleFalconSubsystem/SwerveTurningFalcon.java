// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SimpleFalconSubsystem;

import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class SwerveTurningFalcon extends SubsystemBase {
  private TalonFX motor;
  StatusSignal<Angle> rotorPosSignal;
  PositionVoltage posVolt = new PositionVoltage(0);

  private String name;

  final double MOTOR_TICKS_PER_ROTATION = 2048.0;
  final double MOTOR_RATIO_TO_WHEEL = 150.0 / 7.0;
  final double MOTOR_TICKS_PER_WHEEL_ROTATION = MOTOR_TICKS_PER_ROTATION *
      MOTOR_RATIO_TO_WHEEL;

  /** Creates a new SimpleFalconSubsystem. */
  public SwerveTurningFalcon(String name, int id, boolean invert) {
    motor = new TalonFX(id, "canivore");
    rotorPosSignal = motor.getRotorPosition();

    this.name = name;
    motor.setPosition(0);

    var talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    talonFXConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // set slot 0 gains and leave every other config factory-default
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = 1.0;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 10.0;
    slot0Configs.kV = 0.0;

    talonFXConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    talonFXConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;

    motor.getConfigurator().apply(talonFXConfigs);
  }

  // double m_set;

  double convertRadiansToTicks(double rads) {
    return rads * MOTOR_TICKS_PER_WHEEL_ROTATION / (2 * Math.PI);
  }

  final double twoPi = Math.PI * 2.0;

  /**
   * Give a source radian and a target radian, return the radian difference to
   * add to the source to get to the target, possibly passing through pi/-pi.
   * 
   * @param source
   * @param target
   * @return
   */
  public double shortestRotation(double source, double target) {
    // double sourceMod = source < 0 ? twoPi + (source % twoPi) : (source % twoPi);
    // double targetMod = target < 0 ? twoPi + (target % twoPi) : (target % twoPi);

    double sourceMod = source % twoPi;
    double targetMod = target % twoPi;

    if (sourceMod < 0.0) {
      sourceMod += twoPi;
    }
    if (targetMod < 0.0) {
      targetMod += twoPi;
    }

    if (sourceMod > twoPi) {
      sourceMod -= twoPi;
    }
    if (targetMod > twoPi) {
      targetMod -= twoPi;
    }

    double rotation = targetMod - sourceMod;

    if (rotation > Math.PI) {
      return rotation - twoPi;
    } else if (rotation < -Math.PI) {
      return twoPi + rotation;
    } else {
      return rotation;
    }
  }

  /**
   * value is from -pi to +pi. In order to ensure
   * smoother rotation we check if we are crossing pi
   * and do the math to find the shortest path
   * 
   * @param value radians
   */
  public void set(double value) {
    double currentRotation = this.getRotationRadians();
    double rotation = this.shortestRotation(currentRotation, value);
    double finalRotation = currentRotation + rotation;
    // double e = convertRadiansToTicks(finalRotation);

    // double position = rotorPosSignal.getValue().magnitude();
    // double s = position % MOTOR_TICKS_PER_WHEEL_ROTATION;

    if (this.name == "frontLeftTurningMotor") {
      // SmartDashboard.putNumber(this.name + " desired radians", value);
      // SmartDashboard.putNumber(this.name + " current radians", currentRotation);
      // SmartDashboard.putNumber(this.name + " rotation mod", rotation);
      // SmartDashboard.putNumber(this.name + " shortest radians", finalRotation);
      // System.out.println("rotation: " + rotation);
      // System.out.println("final rot: " + finalRotation);
      // System.out.println("final Tick: " + e);
      // System.out.println("motor curr ticks: " + motor.getSelectedSensorPosition());
      // System.out.println("==============================");
    // }
    // double ticks;
    // if (e - s + MOTOR_TICKS_PER_WHEEL_ROTATION < s - e) {
      // ticks = position + e - s + MOTOR_TICKS_PER_WHEEL_ROTATION;
    // } else {
      // ticks = position - (s - e);
      System.out.println("" + name + " :" + finalRotation + " " + currentRotation);
    }
    motor.setControl(posVolt.withPosition(finalRotation));
    // m_set = e;
  }

  public void setMotorPositionTick(double ticks) {
    motor.setPosition(ticks);
    // motor.setControl(posVolt.withPosition(ticks));
    // m_set = ticks;
  }

  public double getRotationRadians() {
    return rotorPosSignal.getValue().in(Radians);
    // double limitedSensorPosition = rotorPosSignal.getValue().magnitude() % (MOTOR_TICKS_PER_WHEEL_ROTATION);
    // return limitedSensorPosition / MOTOR_TICKS_PER_WHEEL_ROTATION * 2 * Math.PI;
  }

  // public double getRotationTicks() {
  //   return rotorPosSignal.getValue().in(Degrees) / 360 * MOTOR_TICKS_PER_WHEEL_ROTATION;
  //   // return rotorPosSignal.getValue().magnitude() / 360 * MOTOR_TICKS_PER_WHEEL_ROTATION;
  // }

  public void reset() {
    this.reset(0);
  }

  public void reset(double tick) {
    set(0.0);
    motor.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // // This method will be called once per scheduler run
    // SmartDashboard.putNumber(this.name + " position",
    // motor.getSelectedSensorPosition());
    // SmartDashboard.putNumber(this.name + " error", motor.getClosedLoopError());
    // SmartDashboard.putNumber(this.name + " MOTOR_TICKS_PER_WHEEL_ROTATION",
    // MOTOR_TICKS_PER_WHEEL_ROTATION);
    // SmartDashboard.putNumber(this.name + " rads", this.getRotationRadians());
    // SmartDashboard.putNumber(this.name + " m_set", m_set);
    // SmartDashboard.putNumber("test shortestRotation 3/4*PI to -3/4*PI",
    // shortestRotation(Math.PI * 3.0 / 4.0, -Math.PI * 3.0 / 4.0)); // should be
    // +Pi/2 = 1.57

    // SmartDashboard.putNumber("test shortestRotation -3/4*PI to 3/4*PI",
    // shortestRotation(-Math.PI * 3.0 / 4.0, Math.PI * 3.0 / 4.0)); // should be
    // -Pi/2 = 1.57

    rotorPosSignal.refresh();
  }
}
