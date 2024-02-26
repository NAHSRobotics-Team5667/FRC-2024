// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexConstants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.util.States.ShooterState;

public class IndexCommand extends Command {
    private IndexSubsystem index;
    private StateManager states;

    private boolean shooting;

    /**
     * Creates a new IndexCommand.
     * 
     * @param shooting whether a note is being shot.
     */
    public IndexCommand(boolean shooting) {
        index = IndexSubsystem.getInstance();
        states = StateManager.getInstance(); // DO NOT add to addRequirements()

        this.shooting = shooting;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(index);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        index.set(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (shooting) {
            if (states.getShooterState().equals(ShooterState.READY) && states.armAtTarget()) {
                index.set(IndexConstants.SPEED);
            } else {
                index.set(0);
            }
        } else {
            index.set(IndexConstants.SPEED);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        index.set(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // run until interrupted
    }
}
