// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ArmSubsystem.java
 * 
 * Refers to the double-link arm that the shooter is mounted to.
 * 
 * MOTORS ===========================
 * --- First Pivot ---
 * ~ Lead - Falcon 500
 * ~ Follower - Falcon 500 (reverse)
 * 
 * --- Second Pivot ---
 * ~ Lead - Falcon 500
 * ~ Follower - Falcon 500 (reverse)
 * 
 * 
 * SENSORS ===========================
 * --- First Pivot ---
 * ~ x2 REV Through Bore Absolute Encoder
 * (one direction is reversed)
 * 
 * --- Second Pivot ---
 * ~ x2 REV Through Bore Absolute Encoder
 * (one direction is reversed)
 */
public class ArmSubsystem extends SubsystemBase {
    private TalonFX m_firstStageLead, m_firstStageFollower,
            m_secondStageLead, m_secondStageFollower; // declaring motors and global scope
    private DutyCycleEncoder encoder1, encoder2; // declare encoders for arm actuation

    private double firstPivotAngle, secondPivotAngle; // declare variable to store first link angle in degrees

    // ========================================================
    // ============= CLASS & SINGLETON SETUP ==================

    // SINGLETON ----------------------------------------------

    private static ArmSubsystem instance = null;

    private ArmSubsystem() {
        // Initialize motors

        // Initialize encoders

        // Initialize arm positions
    }

    public static ArmSubsystem getInstance() {
        if (instance == null)
            instance = new ArmSubsystem();

        return instance;
    }

    // PERIODIC -----------------------------------------------

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    // ========================================================
    // ================== MOTOR ACTIONS =======================

    // FIRST PIVOT --------------------------------------------

    /**
     * Set the relative velocity of the first pivot.
     * 
     * @param percentOutput % output for both motors controlling first pivot.
     */
    public void setFirstPivotSpeed(double percentOutput) {
    }

    // SECOND PIVOT -------------------------------------------

    /**
     * Set the relative velocity of the second pivot.
     * 
     * @param percentOutput % output for both motors controlling second pivot.
     */
    public void setSecondPivotSpeed(double percentOutput) {
    }

    // ========================================================
    // ===================== SENSORS ==========================

    // FIRST PIVOT --------------------------------------------

    /**
     * @return fist pivot angle in degrees.
     */
    public double getFirstPivotDegrees() {
        return firstPivotAngle;
    }

    // SECOND PIVOT -------------------------------------------

    /**
     * @return second pivot angle in degrees.
     */
    public double getSecondPivotDegrees() {
        return secondPivotAngle;
    }

    // ========================================================
    // ======================= OTHER ==========================

    /**
     * Update the variable that stores angle of first pivot.
     * 
     * @param armAngleDegrees new angle of first pivot.
     */
    public void setFirstPivotDegrees(double armAngleDegrees) {
    }

    /**
     * Update the variable that stores angle of second pivot.
     * 
     * @param armAngleDegrees new angle of second pivot.
     */
    public void setSecondPivotDegrees(double armAngleDegrees) {
    }
}