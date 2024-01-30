// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.util.States.ShooterStates;
import frc.robot.Constants.ShooterConstants;
import com.revrobotics.AbsoluteEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * ShooterSubsystem.java
 * 
 * Refers to the Shooter, which serializes and shoots rings.
 * 
 * Motors:
 * - Belt Serialization - NEO 550
 * - Right Shooter - Falcon 500
 * - Left Shooter - Falcon 500
 * 
 * Sensors:
 * - x2 REV Through Bore Absolute Encoder
 * (one direction is reversed)
 * - Beam Break Sensor to detect if ring is in possession.
 */
public class ShooterSubsystem extends SubsystemBase {
    private TalonFX m_leftShooter, m_rightShooter, m_index; // declaring motors and global scope
    private DigitalInput beamBreak;
    private ShooterStates shooterStates; //Shooter States

    // ========================================================
    // ============= CLASS & SINGLETON SETUP ==================

    // SINGLETON ----------------------------------------------

    private static ShooterSubsystem instance = null;

    private ShooterSubsystem() {
        // Initialize motors

        // Initialize Beam Break sensor
    }

    public static ShooterSubsystem getInstance() {
        if (instance == null)
            instance = new ShooterSubsystem();

        return instance;
    }

    // PERIODIC -----------------------------------------------

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    // ========================================================
    // ================== MOTOR ACTIONS =======================

    // INDEX --------------------------------------------------

    /**
     * Sets speed of index. Positive -> into shooter. Negative -> away from shooter.
     * 
     * @param percentOutput
     */
    public void setIndexSpeed(double percentOutput) {
        m_index.set(percentOutput);
    }

    // SHOOTER ------------------------------------------------

    /**
     * Sets speed of the shooter for both sets of wheels.
     * 
     * @param percentOutput % output for both motors.
     */
    public void setShooterSpeed(double percentOutput) {
        m_leftShooter.set(percentOutput);
        m_rightShooter.set(percentOutput);
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
        return !beamBreak.get();
    }

    // SHOOTER ------------------------------------------------

    /**
     * @return RPM of left set of wheels.
     */
    public double getLeftShooterRPM() {
        double leftSpeed = m_leftShooter.get();
        return leftSpeed;
    }

    /**
     * @return RPM of right set of wheels.
     */
    public double getRightShooterRPM() {
        double rightSpeed = m_rightShooter.get();
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
}
