// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.System;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Magazine.Magazine;
import frc.robot.subsystems.Magazine.Magazine.NotePosition;

public class SystemCenterNote extends Command {
  /** Creates a new SystemCenterNote. */
  public SystemCenterNote() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(RobotContainer.m_intake);
    addRequirements(RobotContainer.m_magazine);
    addRequirements(RobotContainer.m_shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (RobotContainer.m_magazine.getNotePosition()) {
      case JUST_RIGHT:
        RobotContainer.m_intake.setPower(0);
        RobotContainer.m_magazine.setPower(0);
        RobotContainer.m_shooter.setPower(0);
        break;
      case TOO_FAR_IN:
        RobotContainer.m_intake.setPower(0.2);
        RobotContainer.m_magazine.setPower(0.2);
        RobotContainer.m_shooter.setPower(-0.2);
        break;
      case TOO_FAR_OUT:
        RobotContainer.m_intake.setPower(-.2);
        RobotContainer.m_magazine.setPower(-.2);
        RobotContainer.m_shooter.setPower(.2);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_intake.setPower(0);
    RobotContainer.m_magazine.setPower(0);
    RobotContainer.m_shooter.setPower(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.m_magazine.getNotePosition() == Magazine.NotePosition.JUST_RIGHT;
  }
}
