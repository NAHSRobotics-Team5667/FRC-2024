// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.actions;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.AlignShooterAuto;
import frc.robot.commands.index.IndexCommand;
import frc.robot.commands.shooter.ShooterCommandAuto;
import frc.robot.util.States.RobotState;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootNoteSpeaker extends ParallelRaceGroup {
    /** Shoots note into amp in automated routine. */
    public ShootNoteSpeaker() {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                new ShooterCommandAuto(),
                new SequentialCommandGroup(
                        new AlignShooterAuto(),
                        new IndexCommand(RobotState.SPEAKER)));
    }
}
