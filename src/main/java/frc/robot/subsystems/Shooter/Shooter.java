// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Shooter;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.IMayhemTalonFX;

public class Shooter extends SubsystemBase {
  IMayhemTalonFX topMotor;
  IMayhemTalonFX bottomMotor;

  final double kWheelP = 0.08; // 0.015;
  final double kWheelI = 0.000;
  final double kWheelD = 0.000;
  final double kWheelF = 0.000;

  final double CLOSED_LOOP_RAMP_RATE = 0.01; // time from neutral to full in seconds

  /** Creates a new Shooter. */
  public Shooter(IMayhemTalonFX top, IMayhemTalonFX bottom) {
    this.topMotor = top;
    this.bottomMotor = bottom;

    this.topMotor.setInverted(false);
    this.bottomMotor.setInverted(false);

    this.bottomMotor.follow(this.topMotor);
    int slot = 0;
    int timeout = 0;

    this.topMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, slot, timeout);
    this.topMotor.setSensorPhase(false);

    this.topMotor.config_kP(slot, kWheelP, timeout);
    this.topMotor.config_kI(slot, kWheelI, timeout);
    this.topMotor.config_kD(slot, kWheelD, timeout);
    this.topMotor.config_kF(slot, kWheelF, timeout);

    this.topMotor.configPeakOutputForward(1.0);
    this.topMotor.configPeakOutputReverse(-1.0);
    this.topMotor.configNominalOutputForward(0.0);
    this.topMotor.configNominalOutputReverse(0.0);

    this.topMotor.configClosedloopRamp(CLOSED_LOOP_RAMP_RATE); // specify minimum time for neutral to full in seconds
    this.topMotor.selectProfileSlot(slot, timeout);
    this.topMotor.configForwardSoftLimitEnable(false);
    this.topMotor.configReverseSoftLimitEnable(false);
    this.topMotor.configAllowableClosedloopError(slot, 100, timeout);

    this.topMotor.configClosedLoopPeakOutput(slot, 0.5);

    this.setPower(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double d) {
    this.topMotor.set(ControlMode.PercentOutput, d);
  }
}
