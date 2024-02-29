// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ArmConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.util.ArmAngle;
import frc.robot.util.States.ArmState;

/**
 * SetArm.java
 * 
 * Moves the arm automatically to a set position from a list in Constants.
 */
public class SetArm extends Command {
    private ArmSubsystem arm;
    private StateManager states;
    private LimelightSubsystem limelight;

    /**
     * Creates a new SetArm.
     */
    public SetArm() {
        arm = ArmSubsystem.getInstance();

        states = StateManager.getInstance(); // DO NOT add to addRequirements()
        limelight = LimelightSubsystem.getInstance(); // DO NOT add to addReqs()

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
        // double setpoint =
        // ArmConstants.getGoalArmAngle(ArmState.SPEAKER).getFirstPivot();

        // SmartDashboard.putNumber("[ARM] TARGET ANGLE", setpoint);

        // if (RobotContainer.getDriverController().y().getAsBoolean() && setpoint < 80)
        // {
        // ArmConstants.setFirstPivotSpeaker(setpoint + 0.25);
        // } else if (RobotContainer.getDriverController().getLeftTriggerAxis() == 1 &&
        // setpoint > 20) {
        // ArmConstants.setFirstPivotSpeaker(setpoint - 0.25);
        // }

        if (states.getTargetArmState().equals(ArmState.SPEAKER)) {
            ArmConstants.setFirstPivotSpeaker(calculateSpeakerFirstPivot(limelight.getTy()));
        }

        // check if second pivot has priority in the maneuver
        if (ArmConstants.getSecondPivotPriority(states.getTargetArmState())) {
            // run second pivot
            arm.secondPivotToTargetPID(states.getTargetArmAngle().getSecondPivot());

            // run first pivot after second pivot at desired location
            if (arm.secondPivotAtTarget()) {
                arm.firstPivotToTargetPID(states.getTargetArmAngle().getFirstPivot());
            }
        } else { // if first pivot has priority
            // run first pivot
            arm.firstPivotToTargetPID(states.getTargetArmAngle().getFirstPivot());

            // run second pivot after first pivot at desired location
            if (arm.firstPivotAtTarget()) {
                arm.secondPivotToTargetPID(states.getTargetArmAngle().getSecondPivot());
            }
        }

        // arm.secondPivotToTargetPID(states.getTargetArmAngle().getSecondPivot());
        // arm.firstPivotToTargetPID(states.getTargetArmAngle().getFirstPivot());
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

    public double calculateSpeakerFirstPivot(double ty) {
        return 46.3 + (0.693 * ty) - (0.0152 * Math.pow(ty, 2));
    }
}
