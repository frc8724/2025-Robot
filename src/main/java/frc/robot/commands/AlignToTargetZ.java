// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLight.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTargetZ extends Command {
  LimeLightSubsystem limelight;
  SwerveSubsystem swerve;

  // double targetX;
  // double targetY;
  double targetZ;

  double endY;
  double endX;

  // PIDController yPid = new PIDController(4.5, 0.0, 0.0);
  // PIDController xPid = new PIDController(4.5, 0.0, 0.0);
  PIDController rotPid = new PIDController(0.5, 0.0, 0.0);

  /** Creates a new AlignToTarget. */
  public AlignToTargetZ(LimeLightSubsystem l, SwerveSubsystem s, double y, double x) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(l);
    addRequirements(s);

    limelight = l;
    swerve = s;

    endY = y;
    endX = x;

    rotPid.setTolerance(15.0);
  }

  Pose2d initialPose;

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    getTargetCoords();
  }

  void getTargetCoords() {
    if (limelight.getTv() == 1) {
      initialPose = swerve.getPose();

      // swerve.resetOdometry((new Pose2d()));
      targetZ = limelight.getTargetRZ();
    }
  }

  double getRobotRot() {
    Transform2d pose = swerve.getSwerveDrive().getPose().minus(initialPose);
    // Pose2d pose = swerve.getSwerveDrive().getPose();
    double robotRz = pose.getRotation().getDegrees() - targetZ;
    return robotRz;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double robotRz = getRobotRot();
    double driveRot = rotPid.calculate(robotRz);

    // limit driveRot to [-.5, .5]
    driveRot = Math.min(0.35, driveRot);
    driveRot = Math.max(-0.35, driveRot);

    SmartDashboard.putNumber("Align To Target rotZ", driveRot);

    swerve.drive(new ChassisSpeeds(0, 0, driveRot));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double robotRz = getRobotRot();

    boolean rotDone = robotRz > 2 ? false : robotRz < -2 ? false : true;

    SmartDashboard.putNumber("Align To Target robotRz", robotRz);
    SmartDashboard.putNumber("Align To Target pose X", swerve.getSwerveDrive().getPose().getTranslation().getX());
    SmartDashboard.putNumber("Align To Target endY", endY);

    SmartDashboard.putBoolean("Align To Target rot done", rotDone);
    return rotDone;// && xDone && yDone;
  }
}
