// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.VisionSubsystems.Vision;
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
import frc.robot.subsystems.System.SystemStopAllMotors;
import frc.robot.subsystems.Targeting.Targeting;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
        // public static final DriveBaseSubsystem m_robotDrive = new DriveBaseSubsystem();
        // public static final 

        // public static final Targeting m_targets = new Targeting();
        private static final MayhemExtreme3dPro m_driverStick = new MayhemExtreme3dPro(0);
        private static final MayhemOperatorPad m_operatorPad = new MayhemOperatorPad();
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

                m_driverStick.Button(9).onTrue(new DriveZeroGyro(0));
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
         * an arbitrary
         * predicate, or via the named factories in {@link
         * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
         * {@link
         * CommandXboxController
         * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
         * PS4} controllers or
         * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
         * joysticks}.
         */
        private void configureBindings() {

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
