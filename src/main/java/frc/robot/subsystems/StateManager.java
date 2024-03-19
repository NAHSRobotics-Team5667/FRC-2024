// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.ArmAngle;
import frc.robot.util.States.*;

/**
 * Handles everything relating to states of the robot across subsystems.
 */
public class StateManager extends SubsystemBase {
    // arm
    private ArmState armState, targetArmState;

    // shooter
    private ShooterState shooterState;
    private double shooterStartTime;

    // robot state
    private RobotState desiredRobotState;

    // rumble
    private double rumbleTime = 0;

    // maps of robot state to other states --------------------
    private Map<RobotState, ArmState> robotArmMap = Map.of(
            RobotState.IDLE, ArmState.TRANSFER,
            RobotState.INTAKE, ArmState.TRANSFER,
            RobotState.AMP, ArmState.AMP,
            RobotState.SPEAKER, ArmState.SPEAKER,
            RobotState.CLIMB, ArmState.CLIMB,
            RobotState.HANGING, ArmState.HANGING);

    // singleton
    private static StateManager instance = null;

    // ========================================================
    // =================== CONSTRUCTOR ========================

    /** Creates a new StateSubsystem. */
    public StateManager() {
        // Robot ----------------------------------------------
        desiredRobotState = RobotState.IDLE;

        // Arm ------------------------------------------------
        armState = ArmState.TRANSFER;
        targetArmState = ArmState.TRANSFER;

        // Shooter --------------------------------------------
        shooterState = ShooterState.STOPPED;
        shooterStartTime = 0;
    }

    // ========================================================
    // ==================== SINGLETON =========================

    /**
     * @return singleton instance of the State subsystem.
     */
    public static StateManager getInstance() {
        if (instance == null) {
            instance = new StateManager();
        }

        return instance;
    }

    // ========================================================
    // ======================= ROBOT ==========================

    /**
     * Sets the robot state.
     * 
     * @param newState state to replace old state with.
     */
    public void setDesiredRobotState(RobotState newState) {
        desiredRobotState = newState;
    }

    /**
     * @return current robot state.
     */
    public RobotState getDesiredRobotState() {
        return desiredRobotState;
    }

    // ========================================================
    // ======================== ARM ===========================

    /**
     * Periodically called to passively update the arm state.
     */
    private void updateArmState() {
        if (ArmSubsystem.getInstance().armAtTarget()) {
            armState = targetArmState;
        } else {
            armState = ArmState.INTERMEDIATE;
        }
    }

    /**
     * Updates the target arm state passively on a periodic loop.
     */
    private void updateTargetArmState() {
        targetArmState = robotArmMap.get(desiredRobotState);
    }

    // /**
    // * Sets the target arm state.
    // *
    // * @param target target arm state.
    // */
    // public void setTargetArmState(ArmState target) {
    // targetArmState = target;
    // }

    /**
     * @return target arm state.
     */
    public ArmState getTargetArmState() {
        return targetArmState;
    }

    /**
     * @return target arm angles.
     */
    public ArmAngle getTargetArmAngle() {
        return ArmConstants.getGoalArmAngle(targetArmState);
    }

    /**
     * @return current arm state.
     */
    public ArmState getArmState() {
        return armState;
    }

    /**
     * @return whether arm is at target.
     */
    public boolean armAtTarget() {
        return armState.equals(targetArmState);
    }

    // ========================================================
    // ===================== SHOOTER ==========================

    /**
     * Periodically called to passively update the shooter state.
     */
    private void updateShooterState() {
        if (Timer.getFPGATimestamp() - shooterStartTime >= ShooterConstants.SHOOTER_RAMP_TIME) {
            shooterState = ShooterState.READY;
        } else if (ShooterSubsystem.getInstance().getLeftShooterRPM() > 0
                || ShooterSubsystem.getInstance().getRightShooterRPM() > 0) {
            shooterState = ShooterState.MOVING;
        } else {
            shooterState = ShooterState.STOPPED;
        }
    }

    /**
     * @return shooter state.
     */
    public ShooterState getShooterState() {
        return shooterState;
    }

    /**
     * Set shooter start time so that ramp time can be measured.
     * 
     * @param time start shooter time.
     */
    public void setShooterStartTime(double time) {
        shooterStartTime = time;
    }

    /**
     * @return shooter start time.
     */
    public double getShooterStartTime() {
        return shooterStartTime;
    }

    /**
     * @return whether shooter is ready to shoot.
     */
    public boolean isShooterReady() {
        return shooterState.equals(ShooterState.READY);
    }

    // ========================================================
    // ====================== RUMBLE ==========================

    public void updateRumble() {
        if (IndexSubsystem.getInstance().hasGamePiece() && rumbleTime == 0) {
            rumbleTime = Timer.getFPGATimestamp();
            // start rumbling
            RobotContainer.getRumbleController().setRumble(RumbleType.kBothRumble, 0.5);
        } else if (Timer.getFPGATimestamp() - rumbleTime >= 2 || !IndexSubsystem.getInstance().hasGamePiece()) {
            // stop rumbling
            RobotContainer.getRumbleController().setRumble(RumbleType.kBothRumble, 0);
            if (!IndexSubsystem.getInstance().hasGamePiece()) { // reset rumble timer
                rumbleTime = 0;
            }
        }
    }

    // ========================================================
    // ===================== PERIODIC =========================

    @Override
    public void periodic() {
        updateTargetArmState();
        updateArmState(); // update arm state periodically
        updateShooterState(); // update shooter state periodically
        updateRumble();

        SmartDashboard.putBoolean("[STATES] Arm At Target", armAtTarget());
    }
}
