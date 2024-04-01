// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.util.States.RobotState;

public class ShooterCommand extends Command {

    private ShooterSubsystem shooter;

    private StateManager states;
    private RobotState mode; // 1: shoot, 2: amp, 3: feed

    private LimelightSubsystem limelight;

    /**
     * Creates a new ShootCommand.
     * 
     * @param mode shooting mode. 1: speaker, 2: amp, 3: feed
     */
    public ShooterCommand(RobotState mode) {
        this.mode = mode;

        shooter = ShooterSubsystem.getInstance();
        states = StateManager.getInstance(); // DO NOT add to addRequirements()
        limelight = LimelightSubsystem.getInstance(); // DO NOT add to addRequirements()

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {
        states.setShooterStartTime(Timer.getFPGATimestamp());

        states.setDesiredRobotState(mode);

        shooter.set(0.00);
        shooter.setFan(false);
    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        if (mode == RobotState.SPEAKER) {
            shooter.set(ShooterConstants.getSpeakerShooterSpeed(limelight.getTagTy()));
            shooter.setFan(false);

        } else if (mode == RobotState.AMP) {
            shooter.set(ShooterConstants.AMP_SPEED);
            shooter.setFan(false);

        } else if (mode == RobotState.FEED) {
            shooter.set(ShooterConstants.FEED_SPEED);
            shooter.setFan(false);

        } else if (mode == RobotState.TRAP) {
            shooter.set(ShooterConstants.TRAP_SPEED);
            shooter.setFan(true);
        }
    }

    // Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {
        states.setDesiredRobotState(RobotState.IDLE);

        shooter.set(0.00);
        shooter.setFan(false);
    }

    // Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return false;
    }
}
