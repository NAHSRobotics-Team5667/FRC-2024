// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.StateManager;

/**
 * SetArm.java
 * 
 * Moves the arm automatically to a set position from a list in Constants.
 */
public class SetArm extends Command {
    private ArmSubsystem arm;
    private StateManager states;

    /**
     * Creates a new SetArm.
     */
    public SetArm() {
        arm = ArmSubsystem.getInstance();
        states = StateManager.getInstance(); // DO NOT add to addRequirements()

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        arm.stopAllMotors();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // check if second pivot has priority in the maneuver
        if (ArmConstants.getSecondPivotPriority(states.getTargetArmState())) {
            // run second pivot
            arm.secondPivotToTargetPID(arm.getSecondPivotMotorDeg(),
                    states.getTargetArmAngle().getSecondPivot());

            // run first pivot after second pivot at desired location
            if (arm.secondPivotAtTarget()) {

                arm.firstPivotToTargetPID(arm.getFirstPivotMotorDeg(),
                        states.getTargetArmAngle().getFirstPivot());
            }
        } else { // if first pivot has priority
            // run first pivot
            arm.firstPivotToTargetPID(arm.getFirstPivotMotorDeg(),
                    states.getTargetArmAngle().getFirstPivot());

            // run second pivot after first pivot at desired location
            if (arm.firstPivotAtTarget()) {

                arm.secondPivotToTargetPID(arm.getSecondPivotMotorDeg(),
                        states.getTargetArmAngle().getSecondPivot());
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        arm.stopAllMotors();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false; // never stops - always running. If need to adjust arm position, set
                      // target using methods inside other commands.
    }
}
