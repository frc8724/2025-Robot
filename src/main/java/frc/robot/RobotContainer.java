// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.VisionSubsystems.Vision;
import frc.robot.subsystems.old.ArmSubsystem.ArmIsAtPosition;
import frc.robot.subsystems.old.ArmSubsystem.ArmSet;
import frc.robot.subsystems.old.ArmSubsystem.ArmSetPower;
import frc.robot.subsystems.old.ArmSubsystem.ArmSubsystem;
import frc.robot.subsystems.old.ClimberSubsystem.ClimberSetPower;
import frc.robot.subsystems.old.ClimberSubsystem.ClimberSetPowerLeft;
import frc.robot.subsystems.old.ClimberSubsystem.ClimberSetPowerRight;
import frc.robot.subsystems.old.ClimberSubsystem.ClimberSubsystem;
import frc.robot.subsystems.old.IntakeRollers.IntakeRollers;
import frc.robot.subsystems.old.IntakeRollers.IntakeRollersSet;
import frc.robot.subsystems.old.ShooterSubsystem.ShootNote;
import frc.robot.subsystems.old.ShooterSubsystem.ShootNotePost;
import frc.robot.subsystems.old.ShooterSubsystem.ShootNotePre;
import frc.robot.subsystems.old.ShooterSubsystem.ShooterMag;
import frc.robot.subsystems.old.ShooterSubsystem.ShooterMagSet;
import frc.robot.subsystems.old.ShooterSubsystem.ShooterWheelBrake;
import frc.robot.subsystems.old.ShooterSubsystem.ShooterWheels;
import frc.robot.subsystems.old.ShooterSubsystem.ShooterWheelsSet;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.controls.MayhemExtreme3dPro;
import frc.robot.controls.MayhemLogitechAttack3;
import frc.robot.controls.MayhemOperatorPad;
import frc.robot.motors.FakeFalconFX;
import frc.robot.motors.FakeMayhemCANSparkMax;
import frc.robot.motors.IMayhemCANSparkMax;
import frc.robot.motors.IMayhemTalonFX;
import frc.robot.motors.MayhemCANSparkMax;
import frc.robot.motors.MayhemTalonFX;
import frc.robot.motors.MayhemTalonFX.CurrentLimit;
import frc.robot.subsystems.Autonomous.*;
import frc.robot.subsystems.Autonomous.test.AutoTestSquare;
import frc.robot.subsystems.DriveBase.DriveBaseSubsystem;
import frc.robot.subsystems.DriveBase.DriveByJoystick;
import frc.robot.subsystems.DriveBase.DriveZeroGyro;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeSetPower;
import frc.robot.subsystems.LimeLight.LimeLightSubsystem;
import frc.robot.subsystems.Magazine.Magazine;
import frc.robot.subsystems.Magazine.MagazineSetPower;
import frc.robot.subsystems.Pivot.Pivot;
import frc.robot.subsystems.Pivot.PivotByJoystick;
import frc.robot.subsystems.Pivot.PivotSetPosition;
import frc.robot.subsystems.Pivot.PivotSetPower;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Shooter.ShooterSetPower;
import frc.robot.subsystems.System.SystemArmZero;
import frc.robot.subsystems.System.SystemAutoShootShort;
import frc.robot.subsystems.System.SystemCenterNote;
import frc.robot.subsystems.System.SystemIntakeNote;
import frc.robot.subsystems.System.SystemScoreAmp;
import frc.robot.subsystems.System.SystemScoreAmpStop;
import frc.robot.subsystems.System.SystemShootNote;
import frc.robot.subsystems.System.SystemShootNoteAndStorePivot;
import frc.robot.subsystems.System.SystemShootWarmUp;
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
import com.revrobotics.CANSparkLowLevel.MotorType;

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
        private static final IMayhemTalonFX intakeTop = new FakeFalconFX(Constants.DriveConstants.kIntakeRollerId,
                        CurrentLimit.HIGH_CURRENT);
        private static final IMayhemTalonFX armLeft = new FakeFalconFX(Constants.DriveConstants.kArmLeftId,
                        CurrentLimit.HIGH_CURRENT);
        private static final IMayhemTalonFX armRight = new FakeFalconFX(Constants.DriveConstants.kArmRightId,
                        CurrentLimit.HIGH_CURRENT);
        private static final IMayhemTalonFX shooterLeft = new FakeFalconFX(Constants.DriveConstants.kShooterLeftId,
                        CurrentLimit.HIGH_CURRENT);
        private static final IMayhemTalonFX shooterRight = new FakeFalconFX(Constants.DriveConstants.kShooterRightId,
                        CurrentLimit.HIGH_CURRENT);
        private static final IMayhemCANSparkMax magLeft = new FakeMayhemCANSparkMax(Constants.DriveConstants.kMagLeftId,
                        MotorType.kBrushless);
        private static final IMayhemCANSparkMax magRight = new FakeMayhemCANSparkMax(
                        Constants.DriveConstants.kMagRightId,
                        MotorType.kBrushless);
        private static final IMayhemTalonFX climberLeft = new FakeFalconFX(Constants.DriveConstants.kClimberLeftId,
                        CurrentLimit.HIGH_CURRENT);
        private static final IMayhemTalonFX climberRight = new FakeFalconFX(Constants.DriveConstants.kClimberRightId,
                        CurrentLimit.HIGH_CURRENT);

        private static final IMayhemTalonFX shooterTopMotor = new MayhemTalonFX(10, CurrentLimit.HIGH_CURRENT);
        private static final IMayhemTalonFX shooterBottomMotor = new MayhemTalonFX(11, CurrentLimit.HIGH_CURRENT);
        private static final MayhemCANSparkMax intakeMotorA = new MayhemCANSparkMax(15, MotorType.kBrushless);
        private static final MayhemCANSparkMax intakeMotorB = new MayhemCANSparkMax(20, MotorType.kBrushless);
        private static final MayhemCANSparkMax magMotor = new MayhemCANSparkMax(14, MotorType.kBrushless);
        private static final MayhemCANSparkMax pivotLeft = new MayhemCANSparkMax(19, MotorType.kBrushless);
        private static final MayhemCANSparkMax pivotRight = new MayhemCANSparkMax(13, MotorType.kBrushless);

        public static final DriveBaseSubsystem m_robotDrive = new DriveBaseSubsystem();
        public static final IntakeRollers m_rollers = new IntakeRollers(intakeTop);
        public static final ShooterMag m_mag = new ShooterMag(magLeft, magRight);
        public static final ShooterWheels m_wheels = new ShooterWheels(shooterLeft,
                        shooterRight);
        public static final ArmSubsystem m_arm = new ArmSubsystem(armLeft, armRight);
        public static final ClimberSubsystem m_climber = new ClimberSubsystem(climberLeft, climberRight);
        public static final Shooter m_shooter = new Shooter(shooterTopMotor, shooterBottomMotor);

        public static final Intake m_intake = new Intake(intakeMotorA, intakeMotorB);
        public static final Magazine m_magazine = new Magazine(magMotor);
        public static final Pivot m_pivot = new Pivot(pivotLeft, pivotRight);

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
                m_robotDrive.setDefaultCommand(new DriveByJoystick(m_driverStick));

                m_driverStick.Button(9).onTrue(new DriveZeroGyro(0));

                m_operatorPad.BUTTON_TWO.onTrue(new SystemIntakeNote(.7, .8));
                m_operatorPad.BUTTON_TWO.onFalse(new SystemIntakeNote(0));

                m_operatorPad.BUTTON_THREE.onTrue(new SystemIntakeNote(-.4, -.4));
                m_operatorPad.BUTTON_THREE.onFalse(new SystemIntakeNote(.0));

                m_operatorPad.BUTTON_FOUR.whileTrue(new SystemCenterNote());

                m_operatorPad.BUTTON_SIX.onTrue(new SystemShootWarmUp());
                m_operatorPad.BUTTON_SIX.onFalse(new SystemShootNoteAndStorePivot());

                m_operatorPad.D_PAD_DOWN.onTrue(
                                new SequentialCommandGroup(
                                                new PivotSetPosition(Pivot.ZERO),
                                                new WaitCommand(1),
                                                new PivotSetPower(0)));
                m_operatorPad.D_PAD_RIGHT.onTrue(new PivotSetPosition(Pivot.SHORT_SHOT));
                // operatorStick.Button(2).onTrue(new PivotSetPosition(Pivot.AMP_SHOT));
                // m_operatorPad.Button(7).onTrue(new SystemAutoShootShort());
                m_operatorPad.BUTTON_FIVE.onTrue(new SystemScoreAmp());
                m_operatorPad.BUTTON_FIVE.onFalse(new SystemScoreAmpStop());

                m_auto.addAuto(new WaitCommand(5));
                m_auto.addAuto(new AutoDriveOut());
                // m_auto.addAuto(new AutoTestSquare());

                m_auto.addAuto(new PathPlannerAuto("StartCenterShortShoot3Left"));
                m_auto.addAuto(new PathPlannerAuto("StartCenterShoot1Gather1"));
                m_auto.addAuto(new PathPlannerAuto("StartShortShootGather1"));
                m_auto.addAuto(new PathPlannerAuto("StartLongShoot1Gather1"));
                m_auto.addAuto(new PathPlannerAuto("StartLongShoot1Gather1GoToCenter"));
                m_auto.addAuto(new PathPlannerAuto("StartShortShoot1Gather1GoToCenter"));
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
                NamedCommands.registerCommand("SystemShootNote", new SystemShootNote());
                NamedCommands.registerCommand("PivotShortShot", new PivotSetPosition(Pivot.SHORT_SHOT));
                NamedCommands.registerCommand("PivotZero", new PivotSetPosition(Pivot.ZERO));

                NamedCommands.registerCommand("SystemShootNote", new SystemShootNote());

                NamedCommands.registerCommand("SystemAutoShootShort", new SystemAutoShootShort());
                NamedCommands.registerCommand("SystemIntakeNote2sec",
                                new SequentialCommandGroup(new SystemIntakeNote(.5),
                                                new WaitCommand(1.5),
                                                new SystemIntakeNote(0))
                                                .handleInterrupt(() -> RobotContainer.m_intake.setPower(0)));
        }

        // Command auto1 = new PathPlannerAuto("StartCenterShoot1Gather1");
        // "StartCenterShoot1Gather1"
        // StartShortShootGather1
        // StartLongShoot1Gather1
        // StartCenterShortShoot3Left
        // StartLongShoot1Gather1GoToCenter
        // StartShortShoot1Gather1GoToCenter

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {

                // An example command will be run in autonomous
                // return m_auto.getAutoCommand();
                // return auto1;
                // return new PathPlannerAuto("StartCenterScore3");
                return new SequentialCommandGroup(new SystemStopAllMotors(),
                                m_auto.getAutoCommand());
                // new PathPlannerAuto("StartCenterShortShoot3Left"));
        }
}
