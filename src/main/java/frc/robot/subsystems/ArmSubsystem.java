// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.ArmAngle;
import frc.robot.Constants.ArmConstants;

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
    private TalonFX m_firstStageLead, m_firstStageFollower, m_secondStageLead, m_secondStageFollower; // declaring motors and global scope
    private DutyCycleEncoder encoder1, encoder2; // declare encoders for arm actuation

    private ArmAngle armAngles;

    // ========================================================
    // ============= CLASS & SINGLETON SETUP ==================

    // SINGLETON ----------------------------------------------

    private static ArmSubsystem instance = null;

    private ArmSubsystem() {

        //Initialize Falcon Motors by setting IDs.
        m_firstStageLead = new TalonFX(ArmConstants.FIRST_PIVOT_LEAD_ID);
        m_firstStageFollower = new TalonFX(ArmConstants.FIRST_PIVOT_FOLLOWER_ID);

        m_secondStageLead = new TalonFX(ArmConstants.SECOND_PIVOT_LEAD_ID);
        m_secondStageFollower = new TalonFX(ArmConstants.SECOND_PIVOT_FOLLOWER_ID);

        // Initialize encoders (REV Through Bore Absolute Encoder) by setting IDs.
        encoder1 = new DutyCycleEncoder(ArmConstants.FIRST_PIVIOT_PORT);
        encoder2 = new DutyCycleEncoder(ArmConstants.SECOND_PIVIOT_PORT);

        // Initialize arm positions
    }

    /*
     * This method is invoked when the robot is first initialized.
     * Checks if there is an instance. If there is, return that instance.
     * If there's no instance, creates one.
     */
    public static ArmSubsystem getInstance() {
        if (instance == null)
            instance = new ArmSubsystem();

        return instance;
    }

    // PERIODIC -----------------------------------------------

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        // update arm position - run calculation to convert encoder reading to degrees
    }

    // ========================================================
    // ================== MOTOR ACTIONS =======================

    // GENERAL ------------------------------------------------

    /**
     * Stops all motors.
     */
    public void stopAllMotors() {
    }

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
    // ===================== POSITION =========================

    /**
     * Update the Arm Position variable.
     * 
     * @param newPosition replaces current arm position.
     */
    public void updateArmPosition(ArmAngle newPosition) {
        armAngles = newPosition;
    }

    // FIRST PIVOT --------------------------------------------

    /**
     * @return raw encoder values of first pivot. First value is left, second value
     *         is right.
     */
    public double[] getRawFirstEncoders() {
        double[] pivotEncoders = { -1, -1 };
        return pivotEncoders;
    }

    /**
     * @return offset encoder values of first pivot. First value is left, second
     *         value is right.
     */
    public double[] getOffsetFirstEncoders() {
        double[] pivotEncoders = getRawFirstEncoders();
        double[] offsetEncoders = { pivotEncoders[0] + ArmConstants.FIRST_PIVOT_LEFT_OFFSET,
                pivotEncoders[1] + ArmConstants.FIRST_PIVOT_RIGHT_OFFSET };
        return offsetEncoders;
    }

    /**
     * @return fist pivot angle in degrees.
     */
    public double getFirstPivot() {
        return armAngles.getFirstPivot();
    }

    // SECOND PIVOT -------------------------------------------

    /**
     * @return raw encoder values of first pivot. First value is left, second value
     *         is right.
     */
    public double[] getRawSecondEncoders() {
        double[] pivotEncoders = { -1, -1 };
        return pivotEncoders;
    }

    /**
     * @return offset encoder values of first pivot. First value is left, second
     *         value is right.
     */
    public double[] getOffsetSecondEncoders() {
        double[] pivotEncoders = getRawFirstEncoders();
        double[] offsetEncoders = { pivotEncoders[0] + ArmConstants.FIRST_PIVOT_LEFT_OFFSET,
                pivotEncoders[1] + ArmConstants.FIRST_PIVOT_RIGHT_OFFSET };
        return offsetEncoders;
    }

    /**
     * @return second pivot angle in degrees.
     */
    public double getSecondPivot() {
        return armAngles.getSecondPivot();
    }
}