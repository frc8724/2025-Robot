// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.System;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake.IntakeSetPower;
import frc.robot.subsystems.Magazine.MagazineSetPower;
import frc.robot.subsystems.Pivot.Pivot;
import frc.robot.subsystems.Pivot.PivotSetPosition;
import frc.robot.subsystems.Shooter.ShooterSetPower;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemShootNoteAndStorePivot extends SequentialCommandGroup {
  /** Creates a new SystemShootNoteAndStorePivot. */
  public SystemShootNoteAndStorePivot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new MagazineSetPower(1.0),
        new IntakeSetPower(.6),
        new WaitCommand(2),
        new IntakeSetPower(0),

        // Shutdown
        new ShooterSetPower(0),
        new MagazineSetPower(0),
        new PivotSetPosition(Pivot.ZERO)

    );
  }
}
