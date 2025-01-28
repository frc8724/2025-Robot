// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.VisionSubsystems.Vision;
import swervelib.SwerveInputStream;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.controls.MayhemExtreme3dPro;
import frc.robot.controls.MayhemLogitechAttack3;
import frc.robot.controls.MayhemOperatorPad;
import frc.robot.subsystems.Autonomous.*;
import frc.robot.subsystems.Autonomous.test.AutoTestSquare;
import frc.robot.subsystems.DriveBase.DriveBaseSubsystem;
import frc.robot.subsystems.DriveBase.DriveByJoystick;
import frc.robot.subsystems.DriveBase.DriveZeroGyro;
import frc.robot.subsystems.LimeLight.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem.SwerveSubsystem;
import frc.robot.subsystems.System.SystemStopAllMotors;
import frc.robot.subsystems.Targeting.Targeting;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.File;
import java.lang.management.MemoryType;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
        private static final MayhemExtreme3dPro m_driverStick = new MayhemExtreme3dPro(0);
        // private static final MayhemOperatorPad m_operatorPad = new
        // MayhemOperatorPad();

        // public static final DriveBaseSubsystem m_robotDrive = new
        // DriveBaseSubsystem();
        // public static final
        // The robot's subsystems and commands are defined here...
        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"));

        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -m_driverStick.Axis(MayhemExtreme3dPro.Axis.Y).getAsDouble(),
                        () -> -m_driverStick.Axis(MayhemExtreme3dPro.Axis.X).getAsDouble())
                        .withControllerRotationAxis(m_driverStick.Axis(MayhemExtreme3dPro.Axis.Z))
                        .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8).allianceRelativeControl(true);

        /**
         * Clone the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                        .withControllerHeadingAxis(m_driverStick.Axis(MayhemExtreme3dPro.Axis.X),
                                        m_driverStick.Axis(MayhemExtreme3dPro.Axis.Y))
                        .headingWhile(true);

        /**
         * Clone the angular velocity input stream and converts it to a robotRelative
         * input stream.
         */
        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                        .allianceRelativeControl(false);

        SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> -m_driverStick.Axis(MayhemExtreme3dPro.Axis.Y).getAsDouble(),
                        () -> -m_driverStick.Axis(MayhemExtreme3dPro.Axis.X).getAsDouble())
                        .withControllerRotationAxis(
                                        () -> m_driverStick.Axis(MayhemExtreme3dPro.Axis.Z).getAsDouble())
                        .deadband(OperatorConstants.DEADBAND)
                        .scaleTranslation(0.8).allianceRelativeControl(true);

        // Derive the heading axis with math!
        SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                        .withControllerHeadingAxis(() -> Math.sin(
                                        m_driverStick.getRawAxis(MayhemExtreme3dPro.Axis.X) * Math.PI)
                                        * (Math.PI * 2),
                                        () -> Math.cos(m_driverStick.getRawAxis(MayhemExtreme3dPro.Axis.X) * Math.PI)
                                                        * (Math.PI * 2))
                        .headingWhile(true);

        // public static final Targeting m_targets = new Targeting();
        // private static final MayhemLogitechAttack3 operatorStick = new
        // MayhemLogitechAttack3(2);
        private static final AutoChooser m_auto = new AutoChooser();
        // public static final Vision vision = null; // = new Vision(0);
        public static final LimeLightSubsystem m_limelight = new LimeLightSubsystem();

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // Configure the trigger bindings
                configureBindings();
                configureNamedCommands();
                // m_robotDrive.setDefaultCommand(new DriveByJoystick(m_driverStick));

                // m_driverStick.Button(9).onTrue(new DriveZeroGyro(0));
                // m_driverStick.Button(11).onTrue(m_robotDrive.ResetTurning());
                // m_driverStick.Button(1).onTrue(m_robotDrive.print());

                // m_driverStick.Button(7).onTrue(m_robotDrive.SetWheesAtCmd(0));
                // m_driverStick.Button(8).onTrue(m_robotDrive.SetWheesAtCmd(1.5));
                // m_driverStick.Button(9).onTrue(m_robotDrive.SetWheesAtCmd(-1.5));
                // m_driverStick.Button(10).onTrue(m_robotDrive.SetWheesAtCmd(3.14));

                m_auto.addAuto(new WaitCommand(5));
                m_auto.addAuto(new AutoDriveOut());

                // m_robotDrive.ResetTurning();
        }

        /**
         * Use this method to define your trigger->command mappings. Triggers can be
         * created via the
         * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
         * an arbitrary predicate, or via the
         * named factories in
         * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
         * for
         * {@link CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
         * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick
         * Flight joysticks}.
         */
        private void configureBindings() {

                Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
                Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
                Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
                Command driveFieldOrientedAnglularVelocityKeyboard = drivebase
                                .driveFieldOriented(driveAngularVelocityKeyboard);
                Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
                                driveDirectAngleKeyboard);

                if (RobotBase.isSimulation()) {
                        drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
                } else {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
                }

                if (Robot.isSimulation()) {
                        m_driverStick.Button(12).onTrue(Commands
                                        .runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
                        m_driverStick.Button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

                }
                if (DriverStation.isTest()) {
                        drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command
                                                                                         // above!

                        m_driverStick.Button(1).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
                        m_driverStick.Button(2).whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
                        m_driverStick.Button(3).onTrue((Commands.runOnce(drivebase::zeroGyro)));
                        m_driverStick.Button(4).whileTrue(drivebase.centerModulesCommand());
                        m_driverStick.Button(5).onTrue(Commands.none());
                        m_driverStick.Button(6).onTrue(Commands.none());
                } else {
                        m_driverStick.Button(1).onTrue((Commands.runOnce(drivebase::zeroGyro)));
                        m_driverStick.Button(2).onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
                        m_driverStick.Button(3).whileTrue(
                                        drivebase.driveToPose(
                                                        new Pose2d(new Translation2d(4, 4),
                                                                        Rotation2d.fromDegrees(0))));
                        m_driverStick.Button(4).whileTrue(Commands.none());
                        m_driverStick.Button(5).whileTrue(Commands.none());
                        m_driverStick.Button(6).whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
                        m_driverStick.Button(7).onTrue(Commands.none());
                }
        }

        private void configureNamedCommands() {
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                // An example command will be run in autonomous
                // return new PathPlannerAuto("StartCenterScore3");
                return new SequentialCommandGroup(
                                new SystemStopAllMotors(),
                                m_auto.getAutoCommand());
                // new PathPlannerAuto("StartCenterShortShoot3Left"));
        }
}
