// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.System;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Intake.IntakeSetPower;
import frc.robot.subsystems.Magazine.MagazineSetPower;
import frc.robot.subsystems.Shooter.ShooterSetPower;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemScoreAmpStop extends SequentialCommandGroup {
  /** Creates a new SystemScoreAmpStop. */
  public SystemScoreAmpStop() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new MagazineSetPower(0),
        new ShooterSetPower(0),
        new IntakeSetPower(0)

    );
  }
}
