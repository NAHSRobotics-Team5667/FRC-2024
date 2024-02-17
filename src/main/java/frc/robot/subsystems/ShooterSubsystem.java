// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.util.States.IndexStates;
import frc.robot.util.States.ShooterStates;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.util.States.ShooterStates;

/**
 * ShooterSubsystem.java
 * 
 * Refers to the Shooter, which serializes and shoots rings.
 * 
 * Motors:
 * - Belt Serialization / Index - NEO 1.1 Brushless
 * - Right Shooter - Falcon 500
 * - Left Shooter - Falcon 500
 * 
 * Sensors:
 * - x2 REV Through Bore Absolute Encoder
 * (one direction is reversed)
 * - Beam Break Sensor to detect if ring is in possession.
 */
public class ShooterSubsystem extends SubsystemBase {
    private TalonFX m_leftShooter, m_rightShooter; // declaring motors and global scope
    private CANSparkMax m_index;
    private DigitalInput beamBreak;
    private ShooterStates shooterState = null; // Shooter States
    private IndexStates indexState = null; // current state of index

    private double targetLeftRPM = ShooterConstants.SHOOTER_MAX_RPM;
    private double targetRightRPM = ShooterConstants.SHOOTER_MAX_RPM;

    // ========================================================
    // ============= CLASS & SINGLETON SETUP ==================

    // SINGLETON ----------------------------------------------

    private static ShooterSubsystem instance = null;

    private ShooterSubsystem() {

        // Initialize Motors (Falcon 500s).
        m_leftShooter = new TalonFX(ShooterConstants.SHOOTER_LEFT_ID);
        m_leftShooter.setNeutralMode(NeutralModeValue.Coast);

        m_rightShooter = new TalonFX(ShooterConstants.SHOOTER_RIGHT_ID);
        m_rightShooter.setNeutralMode(NeutralModeValue.Coast);

        // Initialize Motors (NEO 1.1).
        m_index = new CANSparkMax(ShooterConstants.BELT_INDEX_ID, MotorType.kBrushless);

        // Initialize Beam Break sensor
        beamBreak = new DigitalInput(ShooterConstants.BEAM_BREAK_CHANNEL_ID);
    }

    public static ShooterSubsystem getInstance() {
        if (instance == null)
            instance = new ShooterSubsystem();

        return instance;
    }

    // PERIODIC -----------------------------------------------

    @Override
    public void periodic() {
        // adjust index state passively
        if (hasGamePiece()) {
            indexState = IndexStates.FULL;
        } else {
            indexState = IndexStates.EMPTY;
        }

        // adjust shooter state passively
        if (getLeftShooterRPM() > targetLeftRPM - ShooterConstants.RPM_ERROR_MARGIN
                && getLeftShooterRPM() < targetLeftRPM + ShooterConstants.RPM_ERROR_MARGIN) {

            if (getRightShooterRPM() > targetRightRPM - ShooterConstants.RPM_ERROR_MARGIN
                    && getRightShooterRPM() < targetRightRPM + ShooterConstants.RPM_ERROR_MARGIN) {

                shooterState = ShooterStates.READY; // shooter ready to shoot
            }
        } else if (getLeftShooterRPM() == 0 || getRightShooterRPM() == 0) {
            shooterState = ShooterStates.STOPPED; // shooter stopped
        } else {
            shooterState = ShooterStates.ADJUST_VEL; // shooter not at correct speed
        }

        // This method will be called once per scheduler run
    }

    // ========================================================
    // ================== MOTOR ACTIONS =======================

    // INDEX --------------------------------------------------

    /**
     * Sets speed of index. Positive -> into shooter. Negative -> away from shooter.
     * 0-100.
     * 
     * @param percentOutput
     */
    public void setIndexSpeed(double percentOutput) {
        double output = percentOutput / 100;
        m_index.set(output);
    }

    // SHOOTER ------------------------------------------------

    /**
     * Sets speed of the shooter for both sets of wheels. 0-100.
     * 
     * @param percentOutput % output for both motors.
     */
    public void setShooterSpeed(double percentOutput) {
        double output = percentOutput / 100;
        m_leftShooter.set(output);
        m_rightShooter.set(output);
    }

    /**
     * Sets speed of shooter wheels with separate speeds for each. Use for spin and
     * lateral adjustments.
     * 
     * @param left  % output for left motor.
     * @param right % output for right motor.
     */
    public void setShooterSpeed(double left, double right) {
        m_leftShooter.set(left);
        m_rightShooter.set(right);
    }

    // ========================================================
    // ===================== SENSORS ==========================

    // INDEX --------------------------------------------------

    /**
     * @return whether game piece is in index by consulting beam-break sensor.
     */
    public boolean hasGamePiece() {
        return !beamBreak.get(); // assuming broken beam = false and unbroken beam = true
    }

    // SHOOTER ------------------------------------------------

    /**
     * @return RPM of left set of wheels.
     */
    public double getLeftShooterRPM() {
        double leftSpeed = m_leftShooter.getVelocity().getValueAsDouble();
        return leftSpeed;
    }

    /**
     * @return RPM of right set of wheels.
     */
    public double getRightShooterRPM() {
        double rightSpeed = m_rightShooter.getVelocity().getValueAsDouble();
        return rightSpeed;
    }

    /**
     * @return shooter RPMs for left and right flywheels. Format - {leftRPM,
     *         rightRPM}
     */
    public double[] getShooterRPM() {
        double[] shooterRPMs = { getLeftShooterRPM(), getRightShooterRPM() };
        return shooterRPMs;
    }

    // ========================================================
    // ======================= OTHER ==========================

    /**
     * @return current shooter state.
     */
    public ShooterStates getShooterState() {
        return shooterState;
    }

    /**
     * @return current index state.
     */
    public IndexStates getIndexState() {
        return indexState;
    }
}
