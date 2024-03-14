// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.arm;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.util.States.ArmState;
import frc.robot.util.States.RobotState;

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
        // for getting setpoints ---------------------------------------

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

        // double firstSetpoint =
        // ArmConstants.getGoalArmAngle(ArmState.CLIMB).getFirstPivot();
        // double secondSetpoint =
        // ArmConstants.getGoalArmAngle(ArmState.CLIMB).getSecondPivot();

        // if (RobotContainer.getDriverController().povUp().getAsBoolean() &&
        // firstSetpoint < 108) {
        // ArmConstants.setFirstPivotClimb(firstSetpoint + 1);
        // } else if (RobotContainer.getDriverController().povDown().getAsBoolean() &&
        // firstSetpoint > 45) {
        // ArmConstants.setFirstPivotClimb(firstSetpoint - 1);
        // } else if (RobotContainer.getDriverController().povLeft().getAsBoolean() &&
        // secondSetpoint > -180) {
        // ArmConstants.setSecondPivotClimb(secondSetpoint - 1);
        // } else if (RobotContainer.getDriverController().povRight().getAsBoolean() &&
        // secondSetpoint < 10) {
        // ArmConstants.setSecondPivotClimb(secondSetpoint + 1);
        // }

        // -------------------------------------------------------------

        boolean aimingAtSpeaker = states.getTargetArmState().equals(ArmState.SPEAKER);
        boolean amp = states.getDesiredRobotState().equals(RobotState.AMP);

        if (aimingAtSpeaker) {
            ArmConstants.setFirstPivotSpeaker(calculateSpeakerFirstPivot(limelight.getTagTy()));
        }

        // // check if second pivot has priority in the maneuver
        // if (ArmConstants.getSecondPivotPriority(states.getTargetArmState())) {
        // // run second pivot
        // arm.secondPivotToTargetProfiledPID(states.getTargetArmAngle().getSecondPivot());

        // // run first pivot after second pivot at desired location
        // if (arm.secondPivotAtTarget()) {
        // arm.firstPivotToTarget(states.getTargetArmAngle().getFirstPivot());
        // }
        // } else { // if first pivot has priority
        // // run first pivot
        // if (aimingAtSpeaker && !DriverStation.isAutonomousEnabled()) {
        // arm.firstPivotToTargetSpeaker(states.getTargetArmAngle().getFirstPivot());
        // } else {
        // arm.firstPivotToTarget(states.getTargetArmAngle().getFirstPivot());
        // }

        // // run second pivot after first pivot at desired location
        // arm.secondPivotToTargetProfiledPID(states.getTargetArmAngle().getSecondPivot());
        // }

        arm.secondPivotToTargetProfiledPID(states.getTargetArmAngle().getSecondPivot());
        arm.firstPivotToTarget(states.getTargetArmAngle().getFirstPivot());
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

    /**
     * @param ty limelight crosshair vertical angle from target.
     * @return ideal first pivot angle to aim into speaker.
     */
    public double calculateSpeakerFirstPivot(double ty) {
        return 37.0 + (0.682 * ty) - (0.00609 * Math.pow(ty, 2));
    }
}
