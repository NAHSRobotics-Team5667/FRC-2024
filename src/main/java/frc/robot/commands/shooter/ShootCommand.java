// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.States.ArmPosState;
import frc.robot.util.States.ShooterStates;

public class ShootCommand extends Command {

    public ShooterSubsystem shooter;
    public ShooterStates shooterState;

    private double initialTime;

    private double left, right;

    /** Creates a new ShooterCommand. */
    public ShootCommand(double left, double right) {
        this.left = left;
        this.right = right;

        shooter = ShooterSubsystem.getInstance();

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(shooter);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {
        initialTime = Timer.getFPGATimestamp();

        ArmSubsystem.getInstance().setTargetPosition(ArmPosState.SPEAKER);

        shooter.setShooterSpeed(0.00);
        shooter.setIndexSpeed(0);
    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        shooter.setShooterSpeed(left, right);

        if (ArmSubsystem.getInstance().getPositionState().equals(ArmPosState.SPEAKER)) {
            if (Timer.getFPGATimestamp() - initialTime >= 1) {
                if (shooter.getShooterState().equals(ShooterStates.READY)) {
                    shooter.setIndexSpeed(50);
                } else {
                    shooter.setIndexSpeed(0);
                }
            } else {
                shooter.setIndexSpeed(0);
            }
        }
    }

    // Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {
        ArmSubsystem.getInstance().setTargetPosition(ArmPosState.TRANSFER);
        shooter.setShooterSpeed(0.00);
        shooter.setIndexSpeed(0);
    }

    // Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return false;
    }
}
