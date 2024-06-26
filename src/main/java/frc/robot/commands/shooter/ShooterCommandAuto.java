// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.util.States.ArmState;
import frc.robot.util.States.RobotState;

public class ShooterCommandAuto extends Command {

    private ShooterSubsystem shooter;

    private StateManager states;
    private LimelightSubsystem limelight;

    private double speed;

    /**
     * Creates a new ShootCommand.
     * 
     * @param amp whether shooting into amp or not.
     */
    public ShooterCommandAuto() {
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
        states.setDesiredRobotState(RobotState.SPEAKER);

        ArmSubsystem.getInstance().resetMotorsToEncoders();

        shooter.set(0.00);
    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        shooter.set(ShooterConstants.getSpeakerShooterSpeed(limelight.getTagTy()));
    }

    // Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {
        states.setDesiredRobotState(RobotState.IDLE);

        shooter.set(0.00);
    }

    // Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return !IndexSubsystem.getInstance().hasGamePiece();
    }
}
