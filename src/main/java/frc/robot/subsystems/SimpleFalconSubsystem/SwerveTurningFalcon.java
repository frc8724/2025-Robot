// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SimpleFalconSubsystem;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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

    var talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    talonFXConfigs.MotorOutput.Inverted = invert ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    // set slot 0 gains and leave every other config factory-default
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kP = 4.1;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
    slot0Configs.kV = 0.0;

    talonFXConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    talonFXConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;

    motor.getConfigurator().apply(talonFXConfigs);

    motor.setPosition(0, 1.0);
  }

  double convertRadiansToTicks(double rads) {
    return rads * MOTOR_TICKS_PER_WHEEL_ROTATION / (2 * Math.PI);
  }

  final double twoPi = Math.PI * 2.0;

  /**
   * Given a starting radian and an ending radian, return a 
   * equivalent radian to the ending that is the shortest 
   * distance from the starting.
   * e.g. staring rad = pi (180 deg)
   *      ending rad = -pi/2 (-90 deg)
   *      shortest rad = 3pi/2 (270 deg)
   * @param starting
   * @param ending
   * @return
   */
  public double shortestRotation(double starting, double ending) {
    // convert radians to rotations to get whole numbers
    var startingRot = starting * 2 * Math.PI;
    var endingRot = ending * 2 * Math.PI;

    long startingInt = (long)startingRot;
    long endingint = (long)endingRot;

    var startingFrac = startingRot - startingInt;
    var endingFrac = endingRot - endingint;

    // example
    // starting = .5
    // ending = -.25
    // opt1: starting - end = .75
    // opt2: end - start = .25



    var opt1 = startingFrac - endingFrac;  // 0 - .25 = -.25
    var opt2 = endingFrac - startingFrac;  // .25 - 0 

    if( opt1 < 0) opt1 += 1.0; // .75
    if( opt2 < 0) opt2 += 1.0; // .25

    if( Math.abs(opt1) < Math.abs(opt2)){
      return starting - opt1;
    }
    else{
      return starting + opt2;
    }

    // double sourceMod = starting % twoPi;
    // double targetMod = ending % twoPi;

    // if (sourceMod < 0.0) {
    //   sourceMod += twoPi;
    // }
    // if (targetMod < 0.0) {
    //   targetMod += twoPi;
    // }

    // if (sourceMod > twoPi) {
    //   sourceMod -= twoPi;
    // }
    // if (targetMod > twoPi) {
    //   targetMod -= twoPi;
    // }

    // double rotation = targetMod - sourceMod;

    // if (rotation > Math.PI) {
    //   return rotation - twoPi;
    // } else if (rotation < -Math.PI) {
    //   return twoPi + rotation;
    // } else {
    //   return rotation;
    // }
  }

  /**
   * value is from -pi to +pi. In order to ensure
   * smoother rotation we check if we are crossing pi
   * and do the math to find the shortest path
   * 
   * @param value radians
   */
  public void set(double value) {
    double currentRotation = this.getRotationRadians() / (150.0/7.0); // convert motor radians to wheel radians
    double rotation = this.shortestRotation(currentRotation, value);
    // double finalRotation = currentRotation + rotation;

    if (this.name == "frontLeftTurningMotor") {
      System.out.println("" + name + " :" + value + " " + currentRotation + " " + rotation);
      // motor.setControl(posVolt.withPosition(value * 3.5));
      // SmartDashboard.putNumber(this.name + " target", value * 3.5);
    }

    motor.setControl(posVolt.withPosition(rotation * 3.5));
  }

  public void setMotorPositionTick(double ticks) {
    motor.setPosition(ticks);
  }

  public double getRotationRadians() {
    return rotorPosSignal.getValue().in(Radians);
  }

  public void reset() {
    this.reset(0);
  }

  public void reset(double tick) {
    set(0.0);
    motor.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    rotorPosSignal.refresh();

    SmartDashboard.putNumber(this.name + " rads", this.getRotationRadians() / (150.0/7.0));
  }
}
