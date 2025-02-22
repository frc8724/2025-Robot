// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLight.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTarget extends Command {
  LimeLightSubsystem limelight;
  SwerveSubsystem swerve;

  double rz;

  /** Creates a new AlignToTarget. */
  public AlignToTarget(LimeLightSubsystem l, SwerveSubsystem s) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(l);
    addRequirements(s);

    limelight = l;
    swerve = s;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerve.resetOdometry((new Pose2d()));
    rz = limelight.getTargetRZ();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d pose = swerve.getSwerveDrive().getPose();
    double robotRz = pose.getRotation().getDegrees() - rz;

    double driveRot = robotRz > 2 ? .5 : robotRz < -2 ? -.5 : 0;
    swerve.drive(new ChassisSpeeds(0.0, 0.0, driveRot));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Pose2d pose = swerve.getSwerveDrive().getPose();
    double robotRz = pose.getRotation().getDegrees() - rz;
    // return Math.abs(robotRz) < 5;
    boolean b = robotRz > 2 ? false : robotRz < -2 ? false : true;
    return b;
  }
}
