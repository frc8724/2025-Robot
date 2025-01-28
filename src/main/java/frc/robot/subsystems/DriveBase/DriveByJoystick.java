// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.DriveBase;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.controls.MayhemExtreme3dPro;

public class DriveByJoystick extends Command {
  MayhemExtreme3dPro m_joystick;

  /** Creates a new DriveByJoystick. */
  public DriveByJoystick(MayhemExtreme3dPro joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(RobotContainer.m_robotDrive);
    m_joystick = joystick;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // RobotContainer.m_robotDrive.drive(
    //     -m_joystick.DeadbandAxis(MayhemExtreme3dPro.Axis.Y,
    //         0.10)
    //         * DriveConstants.kMaxSpeedMetersPerSecond
    //         * (m_joystick.Button(11).getAsBoolean()
    //             ? DriveConstants.kSlowDriveModifier
    //             : DriveConstants.kFullDriveModifier),
    //     -m_joystick.DeadbandAxis(MayhemExtreme3dPro.Axis.X,
    //         0.10)
    //         * DriveConstants.kMaxSpeedMetersPerSecond
    //         * (m_joystick.Button(11).getAsBoolean()
    //             ? DriveConstants.kSlowDriveModifier
    //             : DriveConstants.kFullDriveModifier),
    //     -m_joystick.DeadbandAxis(MayhemExtreme3dPro.Axis.Z,
    //         0.40)
    //         * ModuleConstants.kMaxModuleAngularSpeedRadiansPerSecond
    //         * (m_joystick.Button(11).getAsBoolean()
    //             ? DriveConstants.kSlowDriveModifier
    //             : DriveConstants.kFullDriveModifier),
    //     true);
  }
}
