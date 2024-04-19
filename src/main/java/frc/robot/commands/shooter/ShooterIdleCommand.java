// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterIdleCommand extends Command {
    private ShooterSubsystem shooter;

    /** Creates a new ShooterIdleCommand. */
    public ShooterIdleCommand() {
        shooter = ShooterSubsystem.getInstance();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        shooter.set(0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooter.set(ShooterConstants.IDLE_SPEED);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooter.set(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
