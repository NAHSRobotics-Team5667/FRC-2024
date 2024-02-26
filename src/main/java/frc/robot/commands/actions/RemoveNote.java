// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.index.IndexCommand;
import frc.robot.commands.shooter.OuttakeShooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

/**
 * Removes note out the front of the robot.
 */
public class RemoveNote extends ParallelRaceGroup {
    /**
     * Removes note out the front of the robot.
     */
    public RemoveNote() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new IntakeCommand(false), // run intake backwards
                new IndexCommand(false), // run index into shooter
                new OuttakeShooter()); // shoot note with very little speed
    }
}
