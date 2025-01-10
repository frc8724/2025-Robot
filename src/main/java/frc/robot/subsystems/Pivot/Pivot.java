// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Pivot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.MayhemCANSparkMax;
import frc.robot.subsystems.Intake.Intake;

public class Pivot extends SubsystemBase {
  public static final double ZERO = 0;
  public static final double SHORT_SHOT = 4;
  public static final double AMP_SHOT = 12.35;

  MayhemCANSparkMax m_left;
  MayhemCANSparkMax m_right;

  SparkPIDController pid;
  RelativeEncoder encoder;

  ShuffleboardTab tab = Shuffleboard.getTab("Pivot");
  GenericEntry entryP;
  GenericEntry entryI;
  GenericEntry entryD;
  GenericEntry entryF;

  double setPoint;

  /** Creates a new Pivot. */
  public Pivot(MayhemCANSparkMax left, MayhemCANSparkMax right) {
    this.m_left = left;
    this.m_right = right;

    m_right.setInverted(true);

    m_left.getMotor().follow(m_right.getMotor(), true);

    pid = m_right.getMotor().getPIDController();

    pid.setP(0.1);
    pid.setI(0);
    pid.setD(4.0);
    pid.setFF(0);

    pid.setOutputRange(-.1, .3);

    encoder = m_right.getMotor().getEncoder();
    // encoder.setInverted(true);
    setZero();
    setPower(0);

    entryP = tab
        .add("P", 0.1)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .getEntry();
    entryI = tab
        .add("I", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .getEntry();
    entryD = tab
        .add("D", 4.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .getEntry();
    entryF = tab
        .add("F", 0.0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .getEntry();
  }

  public void setZero() {
    m_right.getMotor().getEncoder().setPosition(0.0);
    setPoint = 0;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // pid.setP(entryP.getDouble(0));
    // pid.setI(entryI.getDouble(0));
    // pid.setD(entryD.getDouble(0));
    // pid.setFF(entryF.getDouble(0));

    SmartDashboard.putNumber("pivot", encoder.getPosition());
    SmartDashboard.putNumber("pivot P", pid.getP());
    SmartDashboard.putNumber("pivot setpoint", setPoint);
  }

  public void setPower(double d) {
    m_right.setVBusPower(d);
  }

  public void setPosition(double pos) {
    setPoint = pos;
    pid.setReference(pos, ControlType.kPosition);
  }
}
