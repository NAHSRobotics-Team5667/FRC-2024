// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.util.States.ArmState;

public class ShooterCommand extends Command { // TODO: make index running manual

    private ShooterSubsystem shooter;

    private StateManager states;

    private double left, right;
    private boolean amp;

    /**
     * Creates a new ShootCommand.
     * 
     * @param amp whether shooting into amp or not.
     */
    public ShooterCommand(boolean amp) {
        this.amp = amp;

        if (amp) {
            left = ShooterConstants.AMP_LEFT_SPEED;
            right = ShooterConstants.AMP_RIGHT_SPEED;
        } else {
            left = ShooterConstants.SPEAKER_LEFT_SPEED;
            right = ShooterConstants.SPEAKER_RIGHT_SPEED;
        }

        shooter = ShooterSubsystem.getInstance();
        states = StateManager.getInstance(); // DO NOT add to addRequirements()

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {
        states.setShooterStartTime(Timer.getFPGATimestamp());
        states.setTargetArmState((amp) ? ArmState.AMP : ArmState.SPEAKER);

        shooter.set(0.00);
    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        shooter.set(left, right);
    }

    // Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {
        states.setTargetArmState(ArmState.TRANSFER);

        shooter.set(0.00);
    }

    // Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return false;
    }
}
