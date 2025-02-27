// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLight.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem.SwerveSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlignToTarget extends Command {
  LimeLightSubsystem limelight;
  SwerveSubsystem swerve;

  double targetX;
  double targetY;
  double targetZ;

  double endY;
  double endX;

  PIDController yPid = new PIDController(4.5, 0.0, 0.0);
  PIDController xPid = new PIDController(4.5, 0.0, 0.0);
  PIDController rotPid = new PIDController(0.5, 0.0, 0.0);

  /** Creates a new AlignToTarget. */
  public AlignToTarget(LimeLightSubsystem l, SwerveSubsystem s, double y, double x) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(l);
    addRequirements(s);

    limelight = l;
    swerve = s;

    endY = y;
    endX = x;

    rotPid.setTolerance(15.0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    getTargetCoords();
  }

  void getTargetCoords() {
    if (limelight.getTv() == 1) {
      swerve.resetOdometry((new Pose2d()));
      targetZ = limelight.getTargetRZ();
      targetX = limelight.getTargetX();
      targetY = limelight.getTargetY();
    }
  }

  double getRobotXToEnd() {
    Pose2d pose = swerve.getSwerveDrive().getPose();
    double robotX = -targetY - endY - pose.getTranslation().getX(); // target Y is Robot X
    return robotX;
  }

  double getRobotYToEnd() {
    Pose2d pose = swerve.getSwerveDrive().getPose();
    double robotY = targetX + endX - pose.getTranslation().getY(); // target X is -Robot Y
    return robotY;
  }

  double getRobotRot() {
    Pose2d pose = swerve.getSwerveDrive().getPose();
    double robotRz = pose.getRotation().getDegrees() - targetZ;
    return robotRz;
  }

  int loop;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // if ((loop % 50) == 0) {
    // getTargetCoords(false);
    // }
    loop++;

    double robotRz = getRobotRot();
    double robotX = getRobotXToEnd();
    double robotY = getRobotYToEnd();

    // double driveRot = robotRz > 2 ? .5 : robotRz < -2 ? -.5 : 0;
    // double driveX = robotX > 0 ? .3 : -.3;

    double driveY = yPid.calculate(robotY);
    double driveX = xPid.calculate(robotX);
    double driveRot = rotPid.calculate(robotRz);

    // limit driveX to [-.5, .5]
    driveX = Math.min(0.75, driveX);
    driveX = Math.max(-0.75, driveX);

    // limit driveY to [-.5, .5]
    driveY = Math.min(0.5, driveY);
    driveY = Math.max(-0.5, driveY);

    // limit driveRot to [-.5, .5]
    driveRot = Math.min(0.35, driveRot);
    driveRot = Math.max(-0.35, driveRot);

    SmartDashboard.putNumber("Align To Target rotZ", driveRot);
    SmartDashboard.putNumber("Align To Target drive x", driveX);
    SmartDashboard.putNumber("Align To Target drive y", driveY);

    driveX = 0;
    driveY = 0;
    swerve.drive(new ChassisSpeeds(-driveX, -driveY, driveRot));
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
    double robotX = getRobotXToEnd();
    double robotY = getRobotYToEnd();

    boolean rotDone = robotRz > 2 ? false : robotRz < -2 ? false : true;
    boolean xDone = Math.abs(robotX) < .05;
    boolean yDone = Math.abs(robotY) < .05;

    SmartDashboard.putNumber("Align To Target robotRz", robotRz);
    SmartDashboard.putNumber("Align To Target robotx", robotX);
    SmartDashboard.putNumber("Align To Target roboty", robotY);
    SmartDashboard.putNumber("Align To Target pose X", swerve.getSwerveDrive().getPose().getTranslation().getX());
    SmartDashboard.putNumber("Align To Target targetY", targetY);
    SmartDashboard.putNumber("Align To Target endY", endY);

    SmartDashboard.putBoolean("Align To Target rot done", rotDone);
    SmartDashboard.putBoolean("Align To Target x done", xDone);
    SmartDashboard.putBoolean("Align To Target y done", yDone);
    return false;// rotDone;// && xDone && yDone;
    // return fa se;

  }
}
