// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LimeLight.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignToTarget2 extends InstantCommand {
  SwerveSubsystem swerve;
  LimeLightSubsystem limelight;

  double endY;
  double endX;

  public AlignToTarget2(SwerveSubsystem s, LimeLightSubsystem l, double y, double x) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerve = s;
    limelight = l;

    endY = y;
    endX = x;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double targetZ = limelight.getTargetRZ();
    double targetX = limelight.getTargetX();
    double targetY = limelight.getTargetY();

    double robotX = -targetY - endY;
    double robotY = targetX + endX;
    double robotRz = -(targetZ);

    // swerve.resetOdometry((new Pose2d()));
    SmartDashboard.putNumber("Align To Target robotRz", robotRz);
    SmartDashboard.putNumber("Align To Target robotx", robotX);
    SmartDashboard.putNumber("Align To Target roboty", robotY);

    Pose2d robotPose = swerve.getPose();
    Pose2d p = // new Pose2d(robotX, robotY, Rotation2d.fromDegrees(robotRz));
        robotPose.transformBy(new Transform2d(new Translation2d(Meter.of(1), // robotX
            Meter.of(0)), // 0
            Rotation2d.fromDegrees(robotRz))); // robotY

    // Command cmd = swerve.driveToPose(p);
    // cmd.addRequirements(swerve);
    // cmd.schedule();
    swerve.driveToPose(p).until(() -> false).schedule();
  }
}
