// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.MayhemCANSparkMax;

public class Intake extends SubsystemBase {
  MayhemCANSparkMax m_motorA;
  MayhemCANSparkMax m_motorB;

  /** Creates a new Intake. */
  public Intake(MayhemCANSparkMax motorA, MayhemCANSparkMax motorB) {
    m_motorA = motorA;
    m_motorB = motorB;

    m_motorA.getMotor().setSmartCurrentLimit(40);
    m_motorB.getMotor().setSmartCurrentLimit(40);

    m_motorA.setInverted(true);

    m_motorB.follow(m_motorA);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setPower(double d) {
    m_motorA.setVBusPower(d);
  }
}
