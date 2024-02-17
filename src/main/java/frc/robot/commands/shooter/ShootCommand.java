// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.States.ShooterStates;

public class ShootCommand extends Command {

    public ShooterSubsystem shooter;
    public ShooterStates shooterState;
    public double[] shooterRPMs = shooter.getShooterRPM();

    /** Creates a new ShooterCommand. */
    public ShootCommand() {
        shooter = ShooterSubsystem.getInstance();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {
        shooter.setShooterSpeed(0.00);
    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        shooter.setShooterSpeed(100);

        if (shooter.getShooterState().equals(ShooterStates.READY)) {
            shooter.setIndexSpeed(50);
        }
    }

    // Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {
        shooter.setShooterSpeed(0.00);
    }

    // Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return false;
    }
}
