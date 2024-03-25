// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.index;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IndexConstants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.util.States.RobotState;
import frc.robot.util.States.ShooterState;

public class IndexCommand extends Command {
    private IndexSubsystem index;
    private StateManager states;

    private RobotState mode; // 1- intake, 2- shoot, 3- outtake

    /**
     * Creates a new IndexCommand.
     * 
     * @param mode whether a note is being shot. 1: intake. 2: shoot. 3: outtake.
     */
    public IndexCommand(RobotState mode) {
        index = IndexSubsystem.getInstance();
        states = StateManager.getInstance(); // DO NOT add to addRequirements()

        this.mode = mode;

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
        if (mode == RobotState.SPEAKER) { // shoot
            if (states.getShooterState().equals(ShooterState.READY) && states.armAtTarget()) { // only run the index if
                                                                                               // shooter is
                // revved up and arm is at target
                index.set(IndexConstants.SHOOT_SPEED);
            } else {
                index.set(0);
            }
        } else if (mode == RobotState.INTAKE) { // intake
            index.set(IndexConstants.INTAKE_SPEED); // run index without conditions if not shooting
        } else if (mode == RobotState.OUTTAKE) { // outtake
            index.set(-IndexConstants.INTAKE_SPEED);
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
