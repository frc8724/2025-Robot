// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoTestSpeeds extends SequentialCommandGroup {
  /** Creates a new AutoTestSpeeds. */
  public AutoTestSpeeds() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        // new DriveForDistance(0.1, 2.0, 0, 0, 0.5),
        new WaitCommand(1.0)
        // new DriveForDistance(2.0, 0.1, 0, 0, 0.5)
        );
  }
}
