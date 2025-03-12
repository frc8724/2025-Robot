// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.VisionSubsystems.Vision;
import swervelib.SwerveInputStream;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlignToTarget;
// import frc.robot.commands.AlignToTarget2;
import frc.robot.commands.AlignToTargetXY;
import frc.robot.commands.AlignToTargetZ;
import frc.robot.controls.JoystickPOVButton;
import frc.robot.controls.MayhemDriverPad;
import frc.robot.controls.MayhemExtreme3dPro;
import frc.robot.controls.MayhemLogitechAttack3;
import frc.robot.controls.MayhemOperatorPad;
import frc.robot.subsystems.EndEffector;
import frc.robot.subsystems.LEDLights;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Autonomous.*;
import frc.robot.subsystems.Autonomous.test.AutoTestSquare;
import frc.robot.subsystems.LimeLight.LimeLightSubsystem;
import frc.robot.subsystems.SwerveSubsystem.SwerveSubsystem;
import frc.robot.subsystems.System.SystemStopAllMotors;
import frc.robot.subsystems.Targeting.Targeting;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.lang.management.MemoryType;
import java.util.Map;
import java.util.Set;

import com.fasterxml.jackson.core.sym.Name;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.controls.JoystickAxisButton.Direction;

/**
 * 
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
        private static final MayhemDriverPad m_operatorPad = new MayhemDriverPad(1);

        private final SwerveSubsystem drivebase = new SwerveSubsystem(
                        new File(Filesystem.getDeployDirectory(), "swerve"));
        private final Wrist wrist = new Wrist();
        private final EndEffector endEffector = new EndEffector(wrist);
        private final Arm arm = new Arm();
        private final LEDLights lights = new LEDLights();

        /**
         * Converts driver input into a field-relative ChassisSpeeds that is controlled
         * by angular velocity.
         */
        // SwerveInputStream driveAngularVelocity =
        // SwerveInputStream.of(drivebase.getSwerveDrive(),
        // () -> {
        // double speedMod = m_driverStick.Button(1).getAsBoolean() ? 1.0 : .25;
        // return m_driverStick.Axis(MayhemExtreme3dPro.Axis.Y).getAsDouble() *
        // speedMod;
        // },
        // () -> {
        // double speedMod = m_driverStick.Button(1).getAsBoolean() ? 1.0 : .25;
        // return m_driverStick.Axis(MayhemExtreme3dPro.Axis.X).getAsDouble() *
        // speedMod;
        // })
        // .withControllerRotationAxis(
        // () -> m_driverStick.Axis(MayhemExtreme3dPro.Axis.Z).getAsDouble() * .6)
        // .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.2).allianceRelativeControl(true);
        SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                        () -> {
                                var multiplier = m_driverStick.Axis(MayhemExtreme3dPro.Axis.Flapper).getAsDouble();
                                multiplier = ((multiplier * -1) + 1.0) / 2; // rescale from [-1,1] to [0,1]
                                multiplier = multiplier + .25;

                                return -m_driverStick.Axis(MayhemExtreme3dPro.Axis.Y).getAsDouble() * multiplier;
                        },
                        () -> {
                                var multiplier = m_driverStick.Axis(MayhemExtreme3dPro.Axis.Flapper).getAsDouble();
                                multiplier = ((multiplier * -1) + 1.0) / 2; // rescale from [-1,1] to [0,1]
                                multiplier = multiplier + .25;

                                return -m_driverStick.Axis(MayhemExtreme3dPro.Axis.X).getAsDouble() * multiplier;
                        })
                        .withControllerRotationAxis(
                                        () -> {
                                                var multiplier = m_driverStick.Axis(MayhemExtreme3dPro.Axis.Flapper)
                                                                .getAsDouble();
                                                multiplier = ((multiplier * -1) + 1.0) / 2; // rescale from [-1,1] to
                                                                                            // [0,1]
                                                multiplier = multiplier + .25;

                                                var stick = m_driverStick.Axis(MayhemExtreme3dPro.Axis.Z).getAsDouble();
                                                return -(Math.abs(stick) * stick) * multiplier;
                                        })
                        .deadband(OperatorConstants.DEADBAND).scaleTranslation(0.8).allianceRelativeControl(false);

        /**
         * Clone the angular velocity input stream and converts it to a fieldRelative
         * input stream.
         */
        // SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
        // .withControllerHeadingAxis(m_driverStick.Axis(MayhemExtreme3dPro.Axis.X),
        // m_driverStick.Axis(MayhemExtreme3dPro.Axis.Y))
        // .headingWhile(true);

        /**
         * Clone the angular velocity input stream and converts it to a robotRelative
         * input stream.
         */
        SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                        .allianceRelativeControl(false);

        // SwerveInputStream driveAngularVelocityKeyboard =
        // SwerveInputStream.of(drivebase.getSwerveDrive(),
        // () -> -m_driverStick.Axis(MayhemExtreme3dPro.Axis.Y).getAsDouble(),
        // () -> -m_driverStick.Axis(MayhemExtreme3dPro.Axis.X).getAsDouble())
        // .withControllerRotationAxis(
        // () -> m_driverStick.Axis(MayhemExtreme3dPro.Axis.Z).getAsDouble())
        // .deadband(OperatorConstants.DEADBAND)
        // .scaleTranslation(0.8).allianceRelativeControl(true);

        // Derive the heading axis with math!
        // SwerveInputStream driveDirectAngleKeyboard =
        // driveAngularVelocityKeyboard.copy()
        // .withControllerHeadingAxis(() -> Math.sin(
        // m_driverStick.getRawAxis(MayhemExtreme3dPro.Axis.X) * Math.PI)
        // * (Math.PI * 2),
        // () -> Math.cos(m_driverStick.getRawAxis(MayhemExtreme3dPro.Axis.X) * Math.PI)
        // * (Math.PI * 2))
        // .headingWhile(true);

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
                configureBindings();
                configureNamedCommands();

                m_auto.addAuto(new WaitCommand(5));
                m_auto.addAuto("drive", new PathPlannerAuto("drive"));
                m_auto.addAuto("start left l1 l1", new PathPlannerAuto("start left l1 l1"));
                m_auto.addAuto("start left l1", new PathPlannerAuto("start left l1"));
                m_auto.addAuto("start left l1 bump", new PathPlannerAuto("start left l1 bump"));
                m_auto.addAuto("start left l1 algae", new PathPlannerAuto("start left l1 algae"));
                m_auto.addAuto("start left l1 algae bump", new PathPlannerAuto("start left l1 algae bump"));
                m_auto.addAuto("start right l1 to HP", new PathPlannerAuto("start right l1 to HP"));

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

                // Command driveFieldOrientedDirectAngle =
                // drivebase.driveFieldOriented(driveDirectAngle);
                Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
                // Command driveRobotOrientedAngularVelocity =
                // drivebase.driveFieldOriented(driveRobotOriented);
                // Command driveSetpointGen =
                // drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
                // Command driveFieldOrientedDirectAngleKeyboard =
                // drivebase.driveFieldOriented(driveDirectAngleKeyboard);
                // Command driveFieldOrientedAnglularVelocityKeyboard = drivebase
                // .driveFieldOriented(driveAngularVelocityKeyboard);
                // Command driveSetpointGenKeyboard =
                // drivebase.driveWithSetpointGeneratorFieldRelative(
                // driveDirectAngleKeyboard);

                // if (RobotBase.isSimulation()) {
                // drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
                // } else {

                drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);

                // }

                // if (Robot.isSimulation()) {
                // m_driverStick.Button(12).onTrue(Commands
                // .runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
                // m_driverStick.Button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

                // }
                // if (DriverStation.isTest()) {
                // drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides
                // drive command
                // // above!

                // m_driverStick.Button(1).whileTrue(Commands.runOnce(drivebase::lock,
                // drivebase).repeatedly());
                // m_driverStick.Button(2).whileTrue(drivebase.driveToDistanceCommand(1.0,
                // 0.2));
                // m_driverStick.Button(3).onTrue((Commands.runOnce(drivebase::zeroGyro)));
                // m_driverStick.Button(4).whileTrue(drivebase.centerModulesCommand());
                // m_driverStick.Button(5).onTrue(Commands.none());
                // m_driverStick.Button(6).onTrue(Commands.none());
                // } else {
                // m_driverStick.Button(1).onTrue((Commands.runOnce(drivebase::zeroGyro)));
                // m_driverStick.Button(2).onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
                // m_driverStick.Button(3).whileTrue(
                // drivebase.driveToPose(
                // new Pose2d(new Translation2d(4, 4),
                // Rotation2d.fromDegrees(0))));

                m_driverStick.Button(11).onTrue(arm.setShoulderSpeedCmd(-.2));
                m_driverStick.Button(11).onFalse(arm.setShoulderSpeedCmd(0));
                m_driverStick.Button(12).onTrue(arm.setShoulderSpeedCmd(.2));
                m_driverStick.Button(12).onFalse(arm.setShoulderSpeedCmd(0));

                m_driverStick.Button(9).onTrue(arm.setElbowSpeedCmd(-.2));
                m_driverStick.Button(9).onFalse(arm.setElbowSpeedCmd(0));
                m_driverStick.Button(10).onTrue(arm.setElbowSpeedCmd(.2));
                m_driverStick.Button(10).onFalse(arm.setElbowSpeedCmd(0));

                m_driverStick.Button(7).onTrue(wrist.setSpeedCmd(-.2));
                m_driverStick.Button(7).onFalse(wrist.setSpeedCmd(0));
                m_driverStick.Button(8).onTrue(wrist.setSpeedCmd(.2));
                m_driverStick.Button(8).onFalse(wrist.setSpeedCmd(0));

                m_driverStick.Button(6).onTrue(drivebase.zerCommand());

                m_driverStick.Button(3).whileTrue(
                                new SequentialCommandGroup(
                                                new AlignToTargetZ(m_limelight, drivebase, .75, 0),
                                                new AlignToTargetXY(m_limelight, drivebase, .75, 0.0),
                                                new AlignToTargetZ(m_limelight, drivebase, 751, 0)));
                m_driverStick.Button(4).whileTrue(
                                new SequentialCommandGroup(
                                                new AlignToTargetZ(m_limelight, drivebase, .9, -.39),
                                                new AlignToTargetXY(m_limelight, drivebase, .9, -.39),
                                                new AlignToTargetZ(m_limelight, drivebase, .9, -.39),
                                                new AlignToTargetXY(m_limelight, drivebase, .9, -0.39)));

                ChassisSpeeds slowFwd = new ChassisSpeeds(.3, 0, 0);
                ChassisSpeeds slowBack = new ChassisSpeeds(-.3, 0, 0);
                ChassisSpeeds slowLeft = new ChassisSpeeds(0, 0.3, 0);
                ChassisSpeeds slowRight = new ChassisSpeeds(0, -0.3, 0);

                m_driverStick.PovButton(JoystickPOVButton.NORTH).whileTrue(drivebase.driveCmd(slowFwd));
                m_driverStick.PovButton(JoystickPOVButton.SOUTH).whileTrue(drivebase.driveCmd(slowBack));
                m_driverStick.PovButton(JoystickPOVButton.WEST).whileTrue(drivebase.driveCmd(slowLeft));
                m_driverStick.PovButton(JoystickPOVButton.EAST).whileTrue(drivebase.driveCmd(slowRight));

                // swerve.getPose();
                // Pose2d p = // new Pose2d(robotX, robotY, Rotation2d.fromDegrees(robotRz));
                // robotPose.transformBy(new Transform2d(new Translation2d(Meter.of(1), //
                // robotX
                // Meter.of(0)), // 0
                // Rotation2d.fromDegrees(robotRz)));

                // Stow - dpad down
                m_operatorPad.PovButton(JoystickPOVButton.SOUTH).onTrue(ArmPositionCmd(0, -1700, 0));
                m_operatorPad.PovButton(JoystickPOVButton.SOUTH).onTrue(endEffector.setSpeedCmd(0.0));
                // Capture Algae - dpad-left
                m_operatorPad.PovButton(JoystickPOVButton.WEST).onTrue(ArmPositionCmd(-549, -1357, -55));
                // Hold Algae - dpad right
                m_operatorPad.PovButton(JoystickPOVButton.EAST).onTrue(ArmPositionCmd(-540, -1322, -32));
                // Score Algae - dpad up
                m_operatorPad.PovButton(JoystickPOVButton.NORTH).onTrue(ArmPositionCmd(-444, -1200, 39));
                m_operatorPad.PovButton(JoystickPOVButton.NORTH).onTrue(endEffector.setSpeedCmd(0.4));

                // Score L1 - Green
                m_operatorPad.Button(MayhemDriverPad.GAMEPAD_F310_A_BUTTON).onTrue(ArmPositionCmd(-531, 1380, -278));
                // Gather from HP - Blue
                m_operatorPad.Button(MayhemDriverPad.GAMEPAD_F310_X_BUTTON).onTrue(ArmPositionCmd(-608, 1145, -342));
                // Score L2 - Red
                m_operatorPad.Button(MayhemDriverPad.GAMEPAD_F310_B_BUTTON).onTrue(ArmPositionCmd(-188, 937, -286));
                // Score L3 - Yellow
                m_operatorPad.Button(MayhemDriverPad.GAMEPAD_F310_Y_BUTTON).onTrue(ArmPositionCmd(-115, 315, -126));

                // Intake - Right Trigger Top
                m_operatorPad.Button(MayhemDriverPad.GAMEPAD_F310_RIGHT_BUTTON).onTrue(endEffector.setSpeedCmd(.3));
                m_operatorPad.Button(MayhemDriverPad.GAMEPAD_F310_RIGHT_BUTTON).onFalse(endEffector.setSpeedCmd(0.0));
                // Out-take - Right Trigger Bottom
                m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_RIGHT_TRIGGER, Direction.POSITIVE_ONLY)
                                .onTrue(endEffector.setSpeedCmd(-0.8));
                m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_RIGHT_TRIGGER, Direction.POSITIVE_ONLY)
                                .onFalse(endEffector.setSpeedCmd(0.0));

                // Knock Off Algae L2 - back
                m_operatorPad.Button(MayhemDriverPad.GAMEPAD_F310_BACK_BUTTON).onTrue(ArmPositionCmd(-668, 1460, -270));
                m_operatorPad.Button(MayhemDriverPad.GAMEPAD_F310_BACK_BUTTON)
                                .onFalse(
                                                new SequentialCommandGroup(
                                                                endEffector.setSpeedCmd(-.5),
                                                                arm.setElbowPositionCmd(1288),
                                                                arm.setShoulderPositionCmd(-270),
                                                                new WaitCommand(1.0),
                                                                endEffector.setSpeedCmd(0)));

                // Knock off algae L3 - start
                m_operatorPad.Button(MayhemDriverPad.GAMEPAD_F310_START_BUTTON)
                                .onTrue(ArmPositionCmd(-547, 1033, -177));
                m_operatorPad.Button(MayhemDriverPad.GAMEPAD_F310_START_BUTTON)
                                .onFalse(
                                                new SequentialCommandGroup(
                                                                endEffector.setSpeedCmd(-.5),
                                                                // ArmPositionCmd(-399, 760, -177),

                                                                arm.setElbowPositionCmd(760),
                                                                arm.setShoulderPositionCmd(-177),
                                                                new WaitCommand(1.0),
                                                                endEffector.setSpeedCmd(0)));

                m_operatorPad.Button(9).onTrue(arm.shoulderZeroCmd());
                m_operatorPad.Button(10).onTrue(ArmPositionCmd(-70, 240, -126));

                // Manual Elbow up-down - left y-axis up/down
                // m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_RIGHT_Y_AXIS,
                // Direction.POSITIVE_ONLY)
                // .onTrue(arm.setElbowPositionOffset(20));
                // m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_RIGHT_Y_AXIS,
                // Direction.NEGATIVE_ONLY)
                // .onTrue(arm.setElbowPositionOffset(-20));
                m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_LEFT_Y_AXIS, Direction.POSITIVE_ONLY)
                                .onTrue(arm.setElbowSpeedCmd(-.2));
                m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_LEFT_Y_AXIS, Direction.POSITIVE_ONLY)
                                .onFalse(arm.setElbowSpeedCmd(0.0));
                m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_LEFT_Y_AXIS, Direction.NEGATIVE_ONLY)
                                .onTrue(arm.setElbowSpeedCmd(.2));
                m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_LEFT_Y_AXIS, Direction.NEGATIVE_ONLY)
                                .onFalse(arm.setElbowSpeedCmd(0.0));

                // Manual Shoulder in-out - right x-axis left/right
                m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_LEFT_X_AXIS, Direction.POSITIVE_ONLY)
                                .onTrue(arm.setShoulderSpeedCmd(-.2));
                m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_LEFT_X_AXIS, Direction.POSITIVE_ONLY)
                                .onFalse(arm.setShoulderSpeedCmd(0.0));
                m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_LEFT_X_AXIS, Direction.NEGATIVE_ONLY)
                                .onTrue(arm.setShoulderSpeedCmd(.2));
                m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_LEFT_X_AXIS, Direction.NEGATIVE_ONLY)
                                .onFalse(arm.setShoulderSpeedCmd(0.0));

                // Manual wrist up-down - right y-axis up/down
                m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_RIGHT_X_AXIS, Direction.POSITIVE_ONLY)
                                .onTrue(wrist.setSpeedCmd(.2));
                m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_RIGHT_X_AXIS, Direction.POSITIVE_ONLY)
                                .onFalse(wrist.setSpeedCmd(0.0));
                m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_RIGHT_X_AXIS, Direction.NEGATIVE_ONLY)
                                .onTrue(wrist.setSpeedCmd(-.20));
                m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_RIGHT_X_AXIS, Direction.NEGATIVE_ONLY)
                                .onFalse(wrist.setSpeedCmd(00));

                // Zero Wrist
                m_operatorPad.Button(MayhemDriverPad.GAMEPAD_F310_LEFT_BUTTON).onTrue(wrist.zeroCmd());
                // All Stop
                m_operatorPad.AxisButton(MayhemDriverPad.GAMEPAD_F310_LEFT_TRIGGER, Direction.POSITIVE_ONLY)
                                .onTrue(allStopCmd());

                // LED Lights
                // default blue / white
                // Coral = red
                // April Tag = Green
                new Trigger(endEffector::hasCoral).whileTrue(lights.hasCoral());
                new Trigger(m_limelight::hasSingleAprilTag).whileTrue(lights.hasAprilTag());
        }

        private Command allStopCmd() {
                return new SequentialCommandGroup(
                                arm.setElbowSpeedCmd(0.0),
                                arm.setShoulderSpeedCmd(0.0),
                                wrist.setSpeedCmd(0.0),
                                endEffector.setSpeedCmd(0.0));
        }

        private void configureNamedCommands() {
                NamedCommands.registerCommand("AutoL1", ArmPositionAutoCmd(-525, -1388, 0));
                NamedCommands.registerCommand("Outtake", endEffector.setSpeedCmd(-.8));
                NamedCommands.registerCommand("IntakeStart", endEffector.setSpeedCmd(0.8));
                NamedCommands.registerCommand("Algae High", ArmPositionAutoCmd(-750, -929, -13));
                NamedCommands.registerCommand("Algae High Remove",
                                ArmPositionAutoCmd(-485, -900, -13).withTimeout(2.0));

                NamedCommands.registerCommand("IntakeStop", endEffector.setSpeedCmd(0.0));
                NamedCommands.registerCommand("AutoHP", ArmPositionAutoCmd(-480, -1200, 0));
                NamedCommands.registerCommand("TeleHP", ArmPositionCmd(-693, 1103, -219));
                NamedCommands.registerCommand("Stow", ArmPositionAutoCmd(0, -1764, 0));

        }

        private Command ArmPositionCmd(double wristPos, double elbowPos, double shoulderPos) {
                return new SequentialCommandGroup(
                                new DeferredCommand(() -> {
                                        double currentSetPoint = arm.getShoulderSetpoint() + 60;
                                        double newNewSetPoint = shoulderPos + 60;

                                        boolean curr = currentSetPoint > 0;
                                        boolean setp = newNewSetPoint > 0;

                                        // if starting on front and going to back, go to L3
                                        if (curr && !setp) {
                                                return ArmPositionDirectCmd(-200, -670, 0);
                                        }
                                        if (!curr && setp) {
                                                return ArmPositionDirectCmd(-200, 495, -117);
                                        }
                                        return new WaitCommand(0.1);
                                }, Set.of(arm)),
                                ArmPositionDirectCmd(wristPos, elbowPos, shoulderPos));
        }

        private Command ArmPositionDirectCmd(double wristPos, double elbowPos, double shoulderPos) {
                return new SequentialCommandGroup(
                                arm.setPositionCmd(elbowPos, shoulderPos), // elbow
                                wrist.setPositionCmd(-215),
                                arm.isAtPositionCmd(),
                                wrist.setPositionCmd(wristPos));
        }

        private Command ArmPositionAutoCmd(double wristPos, double elbowPos, double shoulderPos) {
                return new SequentialCommandGroup(
                                arm.setPositionCmd(elbowPos, shoulderPos), // elbow
                                wrist.setPositionCmd(wristPos),
                                arm.isAtPositionCmd());
        }

        private Command BringArmToVerticalIfNeeded(double newSetPoint) {
                return new SelectCommand<Boolean>(
                                Map.ofEntries(
                                                Map.entry(false, new WaitCommand(0.1)),
                                                Map.entry(true, new SequentialCommandGroup(
                                                                // arm.setPositionCmd(0, 0),
                                                                arm.setElbowPositionCmd(0),
                                                                // arm.isAtPositionCmd(),
                                                                new WaitCommand(1.0),
                                                                arm.setShoulderPositionCmd(newSetPoint),
                                                                arm.isAtPositionCmd()))),
                                () -> {
                                        double currentSetPoint = arm.getShoulderSetpoint() + 60;
                                        double newNewSetPoint = newSetPoint + 60;
                                        // boolean startingIsCloseToVert = Math.abs(currentSetPoint) < 100;
                                        // boolean endingIsCloseToVert = Math.abs(newNewSetPoint) < 100;

                                        return (newNewSetPoint * currentSetPoint) < 0;
                                        // && !startingIsCloseToVert
                                        // && !endingIsCloseToVert;
                                });

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                return m_auto.getAutoCommand();
        }

        public void SwerveBrakeMode(boolean b) {
                drivebase.getSwerveDrive().setMotorIdleMode(b);
        }
}
