// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.ArmAngle;
import frc.robot.util.States.*;

/**
 * Handles everything relating to states of the robot across subsystems.
 */
public class StateManager extends SubsystemBase {
    // arm
    private ArmSubsystem arm;
    private ArmState armState, targetArmState;

    // shooter
    private ShooterSubsystem shooter;
    private ShooterState shooterState;
    private double shooterStartTime;

    // singleton
    private static StateManager instance = null;

    // ========================================================
    // ===================== POSITION =========================

    /** Creates a new StateSubsystem. */
    public StateManager() {
        // Arm ------------------------------------------------
        arm = ArmSubsystem.getInstance();
        targetArmState = ArmState.TRANSFER;

        // Shooter --------------------------------------------
        shooter = ShooterSubsystem.getInstance();
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
    // ======================== ARM ===========================

    /**
     * Periodically called to passively update the arm state.
     */
    private void updateArmState() {
        if (arm.armAtTarget()) {
            armState = targetArmState;
        } else {
            armState = ArmState.INTERMEDIATE;
        }
    }

    /**
     * Sets the target arm state.
     * 
     * @param target target arm state.
     */
    public void setTargetArmState(ArmState target) {
        targetArmState = target;
    }

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
        return ArmConstants.getGoalPosition(targetArmState);
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
        } else if (shooter.getLeftShooterRPM() > 0 || shooter.getRightShooterRPM() > 0) {
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

    // ========================================================
    // ===================== PERIODIC =========================

    @Override
    public void periodic() {
        updateArmState(); // update arm state periodically
        updateShooterState(); // update shooter state periodically
    }
}
