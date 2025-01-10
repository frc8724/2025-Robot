// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Magazine;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motors.MayhemCANSparkMax;

public class Magazine extends SubsystemBase {
  public enum NotePosition {
    TOO_FAR_IN,
    JUST_RIGHT,
    TOO_FAR_OUT,
  };

  MayhemCANSparkMax m_motor;

  DigitalInput beamBreakSensorInner = new DigitalInput(0);
  DigitalInput beamBreakSensorOuter = new DigitalInput(1);

  /** Creates a new Magazine. */
  public Magazine(MayhemCANSparkMax motor) {
    m_motor = motor;
    m_motor.getMotor().setSmartCurrentLimit(20);

    setPower(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("Mag Beam Break Inner", getBeamBreakSensorInner());
    SmartDashboard.putBoolean("Mag Beam Break Outer", getBeamBreakSensorOuter());
  }

  public void setPower(double d) {
    m_motor.setVBusPower(d);
  }

  public boolean getBeamBreakSensorInner() {
    return beamBreakSensorInner.get();
  }

  public boolean getBeamBreakSensorOuter() {
    return beamBreakSensorOuter.get();
  }

  public NotePosition getNotePosition() {
    if (getBeamBreakSensorInner()) {
      if (getBeamBreakSensorOuter()) {
        return NotePosition.TOO_FAR_OUT;
      } else {
        return NotePosition.JUST_RIGHT;
      }
    } else { // inner not broken
      if (getBeamBreakSensorOuter()) {
        return NotePosition.TOO_FAR_OUT;
      } else {
        return NotePosition.TOO_FAR_IN;
      }
    }
  }
}
