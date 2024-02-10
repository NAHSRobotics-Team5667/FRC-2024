// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.ArmAngle;
import frc.robot.util.States.ArmState;

/**
 * SetArm.java
 * 
 * Moves the arm automatically to a set position from a list in Constants.
 */
public class SetArm extends Command {
    private ArmSubsystem armSubsystem;
    private ArmAngle target;

    /**
     * Creates a new SetArm.
     * 
     * @param target target arm position.
     */
    public SetArm(ArmAngle target) {
        armSubsystem = ArmSubsystem.getInstance();
        this.target = target;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(armSubsystem);
    }

    /**
     * Creates a new SetArm.
     * 
     * @param targetState target arm position based on pre-configured positions.
     */
    public SetArm(ArmState targetState) {
        this(ArmConstants.getGoalPosition(targetState));
    }

    /**
     * @return target pivot angles.
     */
    public ArmAngle getTarget() {
        return target;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        armSubsystem.stopAllMotors();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        armSubsystem.firstPivotToTarget(target.getFirstPivot());
        armSubsystem.secondPivotToTarget(target.getSecondPivot());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        armSubsystem.stopAllMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // TODO: finished when another command involving arm is called
    }
}
