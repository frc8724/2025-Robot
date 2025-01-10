// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.System;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Magazine.Magazine.NotePosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SystemCenterNoteInstant extends SequentialCommandGroup {
  /** Creates a new SystemCenterNote. */
  public SystemCenterNoteInstant() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new SelectCommand<>(
            Map.ofEntries(
                Map.entry(NotePosition.JUST_RIGHT, new SystemSlideNote(0)),
                Map.entry(NotePosition.TOO_FAR_IN, new SystemSlideNote(0.2)),
                Map.entry(NotePosition.TOO_FAR_OUT, new SystemSlideNote(-0.2))),
            () -> RobotContainer.m_magazine.getNotePosition()));
  }
}
