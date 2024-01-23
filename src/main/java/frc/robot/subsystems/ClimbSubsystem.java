// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ClimbSubsystem.java
 * 
 * Refers to the Climb. Two climbers responsible for lifting robot off of the
 * ground during endgame.
 * 
 * Motors:
 * - Left Climb - Falcon 500
 * - Right Climb - Falcon 500
 * 
 * Sensors:
 * - Left Absolute Encoder (check WCP GreyT Elevator)
 * - Right Absolute Encoder (check WCP GreyT Elevator)
 */
public class ClimbSubsystem extends SubsystemBase {
    private TalonFX m_leftClimb, m_rightClimb; // declaring motors and global scope

    // ========================================================
    // ============= CLASS & SINGLETON SETUP ==================

    // SINGLETON ----------------------------------------------

    private static ClimbSubsystem instance = null;

    private ClimbSubsystem() {
        // Initialize motors

        // Initialize encoders
    }

    public static ClimbSubsystem getInstance() {
        if (instance == null)
            instance = new ClimbSubsystem();

        return instance;
    }

    // PERIODIC -----------------------------------------------

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    // ========================================================
    // ================== MOTOR ACTIONS =======================

    /**
     * Sets relative velocity of the climb. Positive is upwards, negative is
     * downwards.
     * 
     * @param percentOutput % output of climb motor.
     */
    public void setClimbSpeed(double percentOutput) {
    }

    // ========================================================
    // ===================== SENSORS ==========================

    /**
     * @return left climb hook height.
     */
    public double getLeftClimbHeight() {
        return 0; // TODO: find height of left climb
    }

    /**
     * @return right climb hook height.
     */
    public double getRightClimbHeight() {
        return 0; // TODO: find height of right climb
    }

    // ========================================================
    // ======================= OTHER ==========================
}
