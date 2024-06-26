// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.index.IndexCommand;
import frc.robot.commands.index.IndexCommandAuto;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.IntakeCommandAuto;
import frc.robot.commands.shooter.OuttakeShooter;
import frc.robot.util.States.RobotState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeNoteAuto extends ParallelRaceGroup {
    /**
     * Intakes a note. Stops when note is collected.
     */
    public IntakeNoteAuto() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new IntakeCommandAuto(true),
                new IndexCommand(RobotState.INTAKE));
    }
}
