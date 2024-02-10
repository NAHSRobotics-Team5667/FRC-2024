// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.ArmAngle;
import frc.robot.util.States.ArmState;

//44/60 - Gear for ratio for arm.
//1 climb rotation is 20.25 motor rotations.

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
    private TalonFXConfiguration firstConfig, secondConfig;
    private TalonFX m_firstStageLead, m_firstStageFollower, m_secondStageLead, m_secondStageFollower; // declaring
                                                                                                      // motors and
                                                                                                      // global scope
    private DutyCycleEncoder firstEncoderL, firstEncoderR,
            secondEncoderL, secondEncoderR; // declare encoders for arm actuation

    private ArmAngle armPos;

    private ArmState state;

    // ========================================================
    // ============= CLASS & SINGLETON SETUP ==================

    // SINGLETON ----------------------------------------------

    private static ArmSubsystem instance = null;

    private ArmSubsystem() {
        // ====== FIRST PIVOT ======

        // Initialize Falcon Motors by setting IDs.
        m_firstStageLead = new TalonFX(ArmConstants.FIRST_PIVOT_LEAD_ID);

        m_firstStageFollower = new TalonFX(ArmConstants.FIRST_PIVOT_FOLLOWER_ID);

        // ---- MOTOR CONFIG ----

        firstConfig = new TalonFXConfiguration();

        // --- Motion Magic ---

        // set slot 0 gains
        var slot0Configs = firstConfig.Slot0;
        slot0Configs.kS = ArmConstants.FIRST_kS; // Add __ V output to overcome static friction
        slot0Configs.kV = ArmConstants.FIRST_kV; // A velocity target of 1 rps results in __ V output
        slot0Configs.kA = ArmConstants.FIRST_kA; // An acceleration of 1 rps/s requires __ V output
        slot0Configs.kP = ArmConstants.FIRST_kP; // A position error of __ rotations results in 12 V output
        slot0Configs.kI = ArmConstants.FIRST_kI; // no output for integrated error
        slot0Configs.kD = ArmConstants.FIRST_kD; // A velocity error of 1 rps results in __ V output

        // set Motion Magic settings
        var motionMagicConfigs = firstConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.FIRST_TARGET_CRUISE_VEL; // Target cruise
                                                                                             // velocity of __ rps
        motionMagicConfigs.MotionMagicAcceleration = ArmConstants.FIRST_MAX_ACCEL; // Target acceleration of __ rps/s
                                                                                   // (0.5 seconds)
        motionMagicConfigs.MotionMagicJerk = ArmConstants.FIRST_TARGET_JERK; // Target jerk of __ rps/s/s (0.1 seconds)

        // Set conversion rate from motor shaft rotations to arm rotations
        firstConfig.Feedback.SensorToMechanismRatio = ArmConstants.FIRST_GEAR_RATIO;

        m_firstStageLead.getConfigurator().apply(firstConfig);
        m_firstStageFollower.setControl(new Follower(ArmConstants.FIRST_PIVOT_LEAD_ID, true)); // follow leader

        // ====== SECOND PIVOT ======

        m_secondStageLead = new TalonFX(ArmConstants.SECOND_PIVOT_LEAD_ID);

        m_secondStageFollower = new TalonFX(ArmConstants.SECOND_PIVOT_FOLLOWER_ID);

        // ---- Motor Config ----

        secondConfig = new TalonFXConfiguration();

        // set slot 0 gains
        var secondSlot0Configs = secondConfig.Slot0;
        secondSlot0Configs.kS = ArmConstants.SECOND_kS; // Add __ V output to overcome static friction
        secondSlot0Configs.kV = ArmConstants.SECOND_kV; // A velocity target of 1 rps results in __ V output
        secondSlot0Configs.kA = ArmConstants.SECOND_kA; // An acceleration of 1 rps/s requires __ V output
        secondSlot0Configs.kP = ArmConstants.SECOND_kP; // A position error of __ rotations results in 12 V output
        secondSlot0Configs.kI = ArmConstants.SECOND_kI; // no output for integrated error
        secondSlot0Configs.kD = ArmConstants.SECOND_kD; // A velocity error of 1 rps results in __ V output

        // set Motion Magic settings
        var secondMotionMagicConfigs = secondConfig.MotionMagic;
        secondMotionMagicConfigs.MotionMagicCruiseVelocity = ArmConstants.SECOND_TARGET_CRUISE_VEL; // Target cruise
        // velocity of __ rps
        secondMotionMagicConfigs.MotionMagicAcceleration = ArmConstants.SECOND_MAX_ACCEL; // Target acceleration of __
                                                                                          // rps/s
        // (0.5 seconds)
        secondMotionMagicConfigs.MotionMagicJerk = ArmConstants.SECOND_TARGET_JERK; // Target jerk of __ rps/s/s (0.1
                                                                                    // seconds)

        // Set conversion rate from motor shaft rotations to arm rotations
        secondConfig.Feedback.SensorToMechanismRatio = ArmConstants.SECOND_GEAR_RATIO;

        m_secondStageLead.getConfigurator().apply(secondConfig);
        m_secondStageFollower.setControl(new Follower(ArmConstants.SECOND_PIVOT_LEAD_ID, true)); // follow leader

        // ----------------------------------

        // Initialize encoders (REV Through Bore Absolute Encoder) by setting IDs.
        firstEncoderL = new DutyCycleEncoder(ArmConstants.FIRST_ENC_PORT_1);
        firstEncoderL.setDistancePerRotation(ArmConstants.FIRST_ENC_DIST_PER_ROT);

        firstEncoderR = new DutyCycleEncoder(ArmConstants.FIRST_ENC_PORT_2);
        firstEncoderR.setDistancePerRotation(ArmConstants.FIRST_ENC_DIST_PER_ROT);

        secondEncoderL = new DutyCycleEncoder(ArmConstants.SECOND_ENC_PORT_1);
        secondEncoderL.setDistancePerRotation(ArmConstants.SECOND_ENC_DIST_PER_ROT);

        secondEncoderR = new DutyCycleEncoder(ArmConstants.SECOND_ENC_PORT_2);
        secondEncoderR.setDistancePerRotation(ArmConstants.SECOND_ENC_DIST_PER_ROT);

        // initialize arm position
        armPos = ArmConstants.RESTING_POSITION;
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

    // ========================================================
    // ================== MOTOR ACTIONS =======================

    // GENERAL ------------------------------------------------

    /**
     * Stops all motors.
     */
    public void stopAllMotors() {

        m_firstStageLead.stopMotor();
        m_firstStageFollower.stopMotor();

        m_secondStageLead.stopMotor();
        m_secondStageFollower.stopMotor();
    }

    // FIRST PIVOT --------------------------------------------

    /**
     * Set the relative velocity of the first pivot.
     * 
     * @param percentOutput % output for both motors controlling first pivot.
     */
    public void setFirstPivotSpeed(double percentOutput) {
        double motorSpeed = percentOutput / 100;

        m_firstStageLead.set(motorSpeed);
    }

    /**
     * Moves the motors toward target position using Motion Magic. Needs to be
     * called periodically.
     * 
     * @param targetPos target position of first pivot in degrees.
     */
    public void firstPivotToTarget(double targetPos) {
        double targetPosRot = Units.degreesToRotations(targetPos);
        m_firstStageLead.setControl(new MotionMagicVoltage(targetPosRot));
    }

    // SECOND PIVOT -------------------------------------------

    /**
     * Set the relative velocity of the second pivot.
     * 
     * @param percentOutput % output for both motors controlling second pivot.
     */
    public void setSecondPivotSpeed(double percentOutput) {
        double motorSpeed = percentOutput / 100;

        m_secondStageLead.set(motorSpeed);
        m_secondStageFollower.set(motorSpeed);
    }

    /**
     * Moves the motors toward target position using Motion Magic. Needs to be
     * called periodically.
     * 
     * @param targetPos target position of first pivot in degrees.
     */
    public void secondPivotToTarget(double targetPos) {
        double targetPosRot = Units.degreesToRotations(targetPos);
        m_secondStageLead.setControl(new MotionMagicVoltage(targetPosRot));
    }

    // ========================================================
    // ===================== POSITION =========================

    /**
     * Update the Arm Position variable.
     * 
     * @param newPosition replaces current arm position.
     */
    public void updateArmPosition(ArmAngle newPosition) {
        armPos = newPosition;
    }

    /**
     * Update the Arm Position variable.
     * 
     * @param firstPivot  real first pivot angle.
     * @param secondPivot real second pivot angle.
     */
    public void updateArmPosition(double firstPivot, double secondPivot) {
        armPos.setFirstPivot(firstPivot);
        armPos.setSecondPivot(secondPivot);
    }

    /**
     * @return arm positions for first and second pivots.
     */
    public ArmAngle getArmPosition() {
        return armPos;
    }

    // FIRST PIVOT --------------------------------------------

    /**
     * @return raw encoder values of first pivot. First value is left, second value
     *         is right.
     */
    public double[] getRawFirstEncoders() {
        double[] pivotEncoders = { firstEncoderL.getAbsolutePosition(), firstEncoderR.getAbsolutePosition() };
        return pivotEncoders;
    }

    /**
     * @return offset encoder values of first pivot. First value is left, second
     *         value is right.
     */
    public double[] getOffsetFirstEncoders() {
        double[] pivotEncoders = getRawFirstEncoders();
        double[] offsetEncoders = { pivotEncoders[0] + ArmConstants.FIRST_LEFT_OFFSET,
                pivotEncoders[1] + ArmConstants.FIRST_RIGHT_OFFSET };
        return offsetEncoders;
    }

    /**
     * @return fist pivot angle in degrees.
     */
    public double getFirstPivot() {
        return armPos.getFirstPivot();
    }

    // SECOND PIVOT -------------------------------------------

    /**
     * @return raw encoder values of first pivot. First value is left, second value
     *         is right.
     */
    public double[] getRawSecondEncoders() {
        double[] pivotEncoders = { secondEncoderL.getAbsolutePosition(), secondEncoderR.getAbsolutePosition() };
        return pivotEncoders;
    }

    /**
     * @return offset encoder values of first pivot. First value is left, second
     *         value is right.
     */
    public double[] getOffsetSecondEncoders() {
        double[] pivotEncoders = getRawSecondEncoders();
        double[] offsetEncoders = { pivotEncoders[0] + ArmConstants.SECOND_LEFT_OFFSET,
                pivotEncoders[1] + ArmConstants.SECOND_RIGHT_OFFSET };
        return offsetEncoders;
    }

    /**
     * @return second pivot angle in degrees.
     */
    public double getSecondPivot() {
        return armPos.getSecondPivot();
    }

    // ========================================================
    // ======================= STATE ==========================

    /**
     * @param newState new state of the arm
     */
    public void setState(ArmState newState) {
        state = newState;
    }

    /**
     * @return current state of the arm.
     */
    public ArmState getState() {
        return state;
    }

    // ========================================================
    // ===================== PERIODIC =========================

    @Override
    public void periodic() { // This method will be called once per scheduler run - every 20ms.
        // update arm position with encoders
        updateArmPosition(
                Units.rotationsToDegrees(
                        (getOffsetFirstEncoders()[0] + getOffsetFirstEncoders()[1]) / 2), // average first pivot
                                                                                          // encoders
                Units.rotationsToDegrees(
                        (getOffsetSecondEncoders()[0] + getOffsetSecondEncoders()[1]) / 2)); // average second pivot
                                                                                             // encoders

        // TODO: update state
    }
}