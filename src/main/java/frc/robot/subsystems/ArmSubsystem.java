// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Time;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.ArmAngle;
import frc.robot.util.States.ArmMotionState;
import frc.robot.util.States.ArmPosState;

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
    private DigitalInput limitSwitch;

    private ProfiledPIDController firstPivotPID, secondPivotPID;

    private ArmAngle armPos = new ArmAngle();
    private ArmAngle targetArmPos = ArmConstants.getGoalPosition(ArmPosState.TRANSFER);

    private ArmPosState positionState = null;
    private ArmMotionState motionState = null;

    // ========================================================
    // ============= CLASS & SINGLETON SETUP ==================

    // SINGLETON ----------------------------------------------

    private static ArmSubsystem instance = null;

    private ArmSubsystem() {
        // ==== PID ====

        firstPivotPID = new ProfiledPIDController(
                ArmConstants.FIRST_kP,
                ArmConstants.FIRST_kI,
                ArmConstants.FIRST_kD,
                new TrapezoidProfile.Constraints(ArmConstants.FIRST_MAX_VELOCITY,
                        ArmConstants.FIRST_MAX_ACCEL));

        secondPivotPID = new ProfiledPIDController(
                ArmConstants.SECOND_kP,
                ArmConstants.SECOND_kI,
                ArmConstants.SECOND_kD,
                new TrapezoidProfile.Constraints(ArmConstants.SECOND_MAX_VELOCITY, ArmConstants.SECOND_MAX_ACCEL));

        setFirstPivotSetpoint(getTargetPosition().getFirstPivot());

        // ====== FIRST PIVOT ======

        // Initialize Falcon Motors by setting IDs.
        m_firstStageLead = new TalonFX(ArmConstants.FIRST_PIVOT_LEAD_ID);
        m_firstStageLead.setInverted(true);
        m_firstStageLead.setNeutralMode(NeutralModeValue.Brake);

        m_firstStageFollower = new TalonFX(ArmConstants.FIRST_PIVOT_FOLLOWER_ID);
        m_firstStageFollower.setNeutralMode(NeutralModeValue.Brake);

        // ---- MOTOR CONFIG ----

        firstConfig = new TalonFXConfiguration();

        // Set conversion rate from motor shaft rotations to arm rotations
        firstConfig.Feedback.SensorToMechanismRatio = ArmConstants.FIRST_GEAR_RATIO;

        m_firstStageLead.getConfigurator().apply(firstConfig);
        m_firstStageFollower.setControl(new Follower(ArmConstants.FIRST_PIVOT_LEAD_ID, true)); // follow leader

        // ====== SECOND PIVOT ======

        m_secondStageLead = new TalonFX(ArmConstants.SECOND_PIVOT_LEAD_ID);
        m_secondStageLead.setInverted(false);
        m_secondStageLead.setNeutralMode(NeutralModeValue.Brake);

        m_secondStageFollower = new TalonFX(ArmConstants.SECOND_PIVOT_FOLLOWER_ID);
        m_secondStageFollower.setInverted(false);
        m_secondStageFollower.setNeutralMode(NeutralModeValue.Brake);

        // ---- Motor Config ----

        secondConfig = new TalonFXConfiguration();

        // Set conversion rate from motor shaft rotations to arm rotations
        secondConfig.Feedback.SensorToMechanismRatio = ArmConstants.SECOND_GEAR_RATIO;

        m_secondStageLead.getConfigurator().apply(secondConfig);
        m_secondStageFollower.setControl(new Follower(ArmConstants.SECOND_PIVOT_LEAD_ID, true)); // follow leader

        // ----------------------------------

        // Initialize encoders (REV Through Bore Absolute Encoder) by setting IDs.
        firstEncoderL = new DutyCycleEncoder(ArmConstants.FIRST_ENC_PORT_1);
        firstEncoderL.setDistancePerRotation(ArmConstants.FIRST_ENC_DIST_PER_ROT);

        firstEncoderR = new DutyCycleEncoder(ArmConstants.FIRST_ENC_PORT_2); // one of them will have to be reversed
        firstEncoderR.setDistancePerRotation(ArmConstants.FIRST_ENC_DIST_PER_ROT);

        secondEncoderL = new DutyCycleEncoder(ArmConstants.SECOND_ENC_PORT_1);
        secondEncoderL.setDistancePerRotation(ArmConstants.SECOND_ENC_DIST_PER_ROT);

        secondEncoderR = new DutyCycleEncoder(ArmConstants.SECOND_ENC_PORT_2); // one of them will have to be reversed
        secondEncoderR.setDistancePerRotation(ArmConstants.SECOND_ENC_DIST_PER_ROT);

        // initialize limit switch
        limitSwitch = new DigitalInput((int) ArmConstants.LIMIT_SWITCH);
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
        m_firstStageFollower.setControl(new Follower(ArmConstants.FIRST_PIVOT_LEAD_ID, true));
    }

    /**
     * Moves the motors toward target position using Motion Magic. Needs to be
     * called periodically.
     * 
     * @param targetPos target position of first pivot in degrees.
     */
    public void firstPivotToTargetMotionMagic(double targetPos) {
        double targetPosRot = Units.degreesToRotations(targetPos);
        m_firstStageLead.setControl(new MotionMagicVoltage(targetPosRot));
    }

    /**
     * @param setpoint goal position of first pivot in rotations.
     */
    public void setFirstPivotSetpoint(double setpoint) {
        firstPivotPID.setGoal(setpoint);
    }

    /**
     * @param targetPos PID setpoint in degrees.
     * @return PID calculation in response.
     */
    public double calculateFirstPivotPID(double currentPos, double targetPos) {
        return MathUtil.clamp(
                firstPivotPID.calculate(getFirstPivotMotorDeg(), targetPos),
                -1, 1);
    }

    /**
     * Moves first pivot to the target using trapezoid profiled PID.
     * 
     * @param targetPos target for arm in degrees.
     */
    public void firstPivotToTargetPID(double currentPos, double targetPos) {
        double output = calculateFirstPivotPID(currentPos, targetPos);
        setFirstPivotSpeed(output * 100);
    }

    /**
     * @return velocity of first stage.
     */
    public double getFirstStageVelocity() {
        return m_firstStageLead.getVelocity().getValueAsDouble();
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
        m_secondStageFollower.setControl(new Follower(ArmConstants.SECOND_PIVOT_LEAD_ID, false));
    }

    /**
     * Moves the motors toward target position using Motion Magic. Needs to be
     * called periodically.
     * 
     * @param targetPos target position of first pivot in degrees.
     */
    public void secondPivotToTargetMotionMagic(double targetPos) {
        double targetPosRot = Units.degreesToRotations(targetPos);
        m_secondStageLead.setControl(new MotionMagicVoltage(targetPosRot));
    }

    /**
     * @param setpoint goal position of first pivot in rotations.
     */
    public void setSecondPivotSetpoint(double setpoint) {
        secondPivotPID.setGoal(setpoint);
    }

    /**
     * @param targetPos PID setpoint in degrees.
     * @return PID calculation in response.
     */
    public double calculateSecondPivotPID(double currentPos, double targetPos) {
        return MathUtil.clamp(
                secondPivotPID.calculate(getSecondPivotMotorDeg(), targetPos),
                -1, 1);
    }

    /**
     * Moves first pivot to the target using trapezoid profiled PID.
     * 
     * @param targetPos target for arm in degrees.
     */
    public void secondPivotToTargetPID(double currentPos, double targetPos) {
        double output = calculateSecondPivotPID(currentPos, targetPos);
        setSecondPivotSpeed(output * 100);
    }

    /**
     * @return velocity of second stage.
     */
    public double getSecondStageVelocity() {
        return m_secondStageLead.getVelocity().getValueAsDouble();
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

    /**
     * @return target arm position.
     */
    public ArmAngle getTargetPosition() {
        return targetArmPos;
    }

    /**
     * Sets target arm position for use by the SetArm command.
     * 
     * @param target target arm position.
     */
    public void setTargetPosition(ArmAngle target) {
        targetArmPos = target;
    }

    /**
     * Sets target arm position for use by the SetArm command.
     * 
     * @param targetState target arm state - gets associated arm position.
     */
    public void setTargetPosition(ArmPosState targetState) {
        setTargetPosition(ArmConstants.getGoalPosition(targetState));
    }

    /**
     * @return whether first pivot is at target.
     */
    public boolean firstPivotAtTarget() {
        return getFirstPivotAbsDeg() >= getTargetPosition().getFirstPivot() - ArmConstants.FIRST_ERR_MARGIN_DEG
                && getFirstPivotAbsDeg() <= getTargetPosition().getFirstPivot() + ArmConstants.FIRST_ERR_MARGIN_DEG;
    }

    /**
     * @return whether second pivot is at target.
     */
    public boolean secondPivotAtTarget() {
        return getSecondPivotAbsDeg() >= getTargetPosition().getSecondPivot() - ArmConstants.SECOND_ERR_MARGIN_DEG
                && getSecondPivotAbsDeg() <= getTargetPosition().getSecondPivot() + ArmConstants.SECOND_ERR_MARGIN_DEG;
    }

    // FIRST PIVOT --------------------------------------------

    /**
     * @return raw encoder values of first pivot. First value is left, second value
     *         is right.
     */
    public double[] getRawFirstEncoders() {
        double[] pivotEncoders = {
                (firstEncoderL.isConnected())
                        ? -firstEncoderL.getAbsolutePosition()
                        : firstEncoderR.getAbsolutePosition(),
                (firstEncoderR.isConnected())
                        ? firstEncoderR.getAbsolutePosition()
                        : -firstEncoderL.getAbsolutePosition() };
        return pivotEncoders;
    }

    /**
     * @return offset encoder values of first pivot. First value is left, second
     *         value is right.
     */
    public double[] getOffsetFirstEncoders() {
        double[] pivotEncoders = getRawFirstEncoders();
        double[] offsetEncoders = { pivotEncoders[0] - ArmConstants.FIRST_LEFT_OFFSET,
                pivotEncoders[1] - ArmConstants.FIRST_RIGHT_OFFSET };
        return offsetEncoders;
    }

    /**
     * @return first pivot angle in degrees. Reading updated using the encoder.
     */
    public double getFirstPivotAbsDeg() {
        return armPos.getFirstPivot();
    }

    /**
     * @return first pivot angle in degrees. Using motor values.
     */
    public double getFirstPivotMotorDeg() {
        return Units.rotationsToDegrees(m_firstStageLead.getPosition().getValueAsDouble());
    }

    /**
     * @return whether first arm is touching limit switch.
     */
    public boolean getLimitSwitch() {
        return !limitSwitch.get();
    }

    // SECOND PIVOT -------------------------------------------

    /**
     * @return raw encoder values of first pivot. First value is left, second value
     *         is right.
     */
    public double[] getRawSecondEncoders() {
        double[] pivotEncoders = {
                (secondEncoderL.isConnected())
                        ? -secondEncoderL.getAbsolutePosition()
                        : secondEncoderR.getAbsolutePosition(),
                (secondEncoderR.isConnected())
                        ? secondEncoderR.getAbsolutePosition()
                        : -secondEncoderL.getAbsolutePosition() };
        return pivotEncoders;
    }

    /**
     * @return offset encoder values of first pivot. First value is left, second
     *         value is right.
     */
    public double[] getOffsetSecondEncoders() {
        double[] pivotEncoders = getRawSecondEncoders();
        double[] offsetEncoders = { pivotEncoders[0] - ArmConstants.SECOND_LEFT_OFFSET,
                pivotEncoders[1] - ArmConstants.SECOND_RIGHT_OFFSET };
        return offsetEncoders;
    }

    /**
     * @return second pivot angle in degrees.
     */
    public double getSecondPivotAbsDeg() {
        return armPos.getSecondPivot();
    }

    /**
     * @return first pivot angle in degrees. Using motor values.
     */
    public double getSecondPivotMotorDeg() {
        return Units.rotationsToDegrees(m_secondStageLead.getPosition().getValueAsDouble());
    }

    // ========================================================
    // ======================= STATE ==========================

    /**
     * @param newState new positon state of the arm
     */
    public void updatePositionState(ArmPosState newState) {
        positionState = newState;
    }

    /**
     * @return current position state of the arm.
     */
    public ArmPosState getPositionState() {
        return positionState;
    }

    /**
     * @param newState new motion state of the arm
     */
    public void updateMotionState(ArmMotionState newState) {
        motionState = newState;
    }

    /**
     * @return current motion state of the arm.
     */
    public ArmMotionState getMotionState() {
        return motionState;
    }

    // ========================================================
    // ===================== PERIODIC =========================

    @Override
    public void periodic() { // This method will be called once per scheduler run - every 20ms.
        // update arm position with encoders
        updateArmPosition(
                Units.rotationsToDegrees(
                        ((getOffsetFirstEncoders()[0] + getOffsetFirstEncoders()[1]) / 2) * (44.0 / 60.0)), // average
                                                                                                            // first
                                                                                                            // pivot
                // encoders
                Units.rotationsToDegrees(
                        (getOffsetSecondEncoders()[0] + getOffsetSecondEncoders()[1]) / 2)); // average second pivot
                                                                                             // encoders

        // set motor encoders ---------------------------------

        if ((Math.abs(getFirstPivotAbsDeg() - getFirstPivotMotorDeg()) > 1)
                || (Math.abs(getSecondPivotAbsDeg() - getSecondPivotMotorDeg())) > 1) {
            // initial motor positions - first stage
            m_firstStageLead.getConfigurator()
                    .setPosition(Units.degreesToRotations(getFirstPivotAbsDeg()));
            m_firstStageFollower.getConfigurator()
                    .setPosition(Units.degreesToRotations(getFirstPivotAbsDeg()));

            // initial motor positions - second stage
            m_secondStageLead.getConfigurator()
                    .setPosition(Units.degreesToRotations(getSecondPivotAbsDeg()));
            m_secondStageFollower.getConfigurator()
                    .setPosition(Units.degreesToRotations(getSecondPivotAbsDeg()));
        }

        // ---------------------------------------------------
        // -------------- UPDATE MOTION STATE ----------------

        if (Math.abs(m_firstStageLead.getVelocity().getValueAsDouble()) >= ArmConstants.FIRST_VEL_THRESHOLD
                || Math.abs(m_secondStageLead.getVelocity().getValueAsDouble()) >= ArmConstants.SECOND_VEL_THRESHOLD) {
            // set arm motion state to moving if any velocities are over thresholds
            updateMotionState(ArmMotionState.MOVING);
        } else {
            // set arm motion state to idle if both velocities are under thresholds
            updateMotionState(ArmMotionState.IDLE);
        }

        // ---------------------------------------------------
        // ---------- UPDATE CURR POSITION STATE -------------

        if (getFirstPivotAbsDeg() <= ArmConstants.getGoalPosition(ArmPosState.TRANSFER).getFirstPivot()
                + ArmConstants.FIRST_ERR_MARGIN_DEG
                && getFirstPivotAbsDeg() >= ArmConstants.getGoalPosition(ArmPosState.TRANSFER).getFirstPivot()
                        - ArmConstants.FIRST_ERR_MARGIN_DEG) {
            // check second pivot
            if (getSecondPivotAbsDeg() <= ArmConstants.getGoalPosition(ArmPosState.TRANSFER).getSecondPivot()
                    + ArmConstants.SECOND_ERR_MARGIN_DEG
                    && getSecondPivotAbsDeg() >= ArmConstants.getGoalPosition(ArmPosState.TRANSFER).getSecondPivot()
                            - ArmConstants.SECOND_ERR_MARGIN_DEG) {

                updatePositionState(ArmPosState.TRANSFER); // TRANSFER
            }

        } else if (getFirstPivotAbsDeg() <= ArmConstants.getGoalPosition(ArmPosState.SPEAKER).getFirstPivot()
                + ArmConstants.FIRST_ERR_MARGIN_DEG
                && getFirstPivotAbsDeg() >= ArmConstants.getGoalPosition(ArmPosState.SPEAKER).getFirstPivot()
                        - ArmConstants.FIRST_ERR_MARGIN_DEG) {
            // check second pivot
            if (getSecondPivotAbsDeg() <= ArmConstants.getGoalPosition(ArmPosState.SPEAKER).getSecondPivot()
                    + ArmConstants.SECOND_ERR_MARGIN_DEG
                    && getSecondPivotAbsDeg() >= ArmConstants.getGoalPosition(ArmPosState.SPEAKER).getSecondPivot()
                            - ArmConstants.SECOND_ERR_MARGIN_DEG) {

                updatePositionState(ArmPosState.SPEAKER); // SPEAKER
            }

        } else if (getFirstPivotAbsDeg() <= ArmConstants.getGoalPosition(ArmPosState.AMP).getFirstPivot()
                + ArmConstants.FIRST_ERR_MARGIN_DEG
                && getFirstPivotAbsDeg() >= ArmConstants.getGoalPosition(ArmPosState.AMP).getFirstPivot()
                        - ArmConstants.FIRST_ERR_MARGIN_DEG) {
            // check second pivot
            if (getSecondPivotAbsDeg() <= ArmConstants.getGoalPosition(ArmPosState.AMP).getSecondPivot()
                    + ArmConstants.SECOND_ERR_MARGIN_DEG
                    && getSecondPivotAbsDeg() >= ArmConstants.getGoalPosition(ArmPosState.AMP).getSecondPivot()
                            - ArmConstants.SECOND_ERR_MARGIN_DEG) {

                updatePositionState(ArmPosState.AMP); // AMP
            }

        } else if (getFirstPivotAbsDeg() <= ArmConstants.getGoalPosition(ArmPosState.TRAP).getFirstPivot()
                + ArmConstants.FIRST_ERR_MARGIN_DEG
                && getFirstPivotAbsDeg() >= ArmConstants.getGoalPosition(ArmPosState.TRAP).getFirstPivot()
                        - ArmConstants.FIRST_ERR_MARGIN_DEG) {
            // check second pivot
            if (getSecondPivotAbsDeg() <= ArmConstants.getGoalPosition(ArmPosState.TRAP).getSecondPivot()
                    + ArmConstants.SECOND_ERR_MARGIN_DEG
                    && getSecondPivotAbsDeg() >= ArmConstants.getGoalPosition(ArmPosState.TRAP).getSecondPivot()
                            - ArmConstants.SECOND_ERR_MARGIN_DEG) {

                updatePositionState(ArmPosState.TRAP); // TRAP
            }

        } else if (getFirstPivotAbsDeg() <= ArmConstants.getGoalPosition(ArmPosState.CLIMB).getFirstPivot()
                + ArmConstants.FIRST_ERR_MARGIN_DEG
                && getFirstPivotAbsDeg() >= ArmConstants.getGoalPosition(ArmPosState.CLIMB).getFirstPivot()
                        - ArmConstants.FIRST_ERR_MARGIN_DEG) {
            // check second pivot
            if (getSecondPivotAbsDeg() <= ArmConstants.getGoalPosition(ArmPosState.CLIMB).getSecondPivot()
                    + ArmConstants.SECOND_ERR_MARGIN_DEG
                    && getSecondPivotAbsDeg() >= ArmConstants.getGoalPosition(ArmPosState.CLIMB)
                            .getSecondPivot()
                            - ArmConstants.SECOND_ERR_MARGIN_DEG) {

                updatePositionState(ArmPosState.CLIMB); // HUMAN PLAYER
            }

        } else {
            updatePositionState(ArmPosState.INTERMEDIATE); // INTERMEDIATE if not at any of the other positions
        }

        // ---------------------------------------------------
        // ------------------- TELEMETRY ---------------------

        SmartDashboard.putNumber("[ARM] Abs First Pivot Deg", getFirstPivotAbsDeg());
        SmartDashboard.putNumber("[ARM] Abs Second Pivot Deg", getSecondPivotAbsDeg());

        // First Encoder -------------------------------------
        SmartDashboard.putNumber("[ARM] Raw First Abs Encoder 1", getRawFirstEncoders()[0]);
        SmartDashboard.putNumber("[ARM] Raw First Abs Encoder 2", getRawFirstEncoders()[1]);

        SmartDashboard.putNumber("[ARM] Offset First Abs Encoder 1", getOffsetFirstEncoders()[0]);
        SmartDashboard.putNumber("[ARM] Offset First Abs Encoder 2", getOffsetFirstEncoders()[1]);

        SmartDashboard.putNumber("[ARM] First Stage Motor Encoder Deg", getFirstPivotMotorDeg());

        // Second Encoder ------------------------------------
        SmartDashboard.putNumber("[ARM] Raw Second Abs Encoder 1", getRawSecondEncoders()[0]);
        SmartDashboard.putNumber("[ARM] Raw Second Abs Encoder 2", getRawSecondEncoders()[1]);

        SmartDashboard.putNumber("[ARM] Offset Second Abs Encoder 1", getOffsetSecondEncoders()[0]);
        SmartDashboard.putNumber("[ARM] Offset Second Abs Encoder 2", getOffsetSecondEncoders()[1]);

        SmartDashboard.putNumber("[ARM] Second Stage Motor Encoder Deg", getSecondPivotMotorDeg());

        // States --------------------------------------------
        SmartDashboard.putString("[ARM] Curr Pos State", positionState.toString());
        SmartDashboard.putString("[ARM] Curr Motion State", motionState.toString());

        // Limit Switch --------------------------------------
        SmartDashboard.putBoolean("[ARM] Limit Switch", getLimitSwitch());

        // PID -----------------------------------------------
        SmartDashboard.putNumber("[ARM] 1st PID Setpoint", firstPivotPID.getGoal().position);
        SmartDashboard.putNumber("[ARM] 1st PID Output",
                calculateFirstPivotPID(getFirstPivotMotorDeg(), getTargetPosition().getFirstPivot()));
        SmartDashboard.putNumber("[ARM] 1st PID Error", firstPivotPID.getPositionError());
        SmartDashboard.putNumber("[ARM] 1st Target", getTargetPosition().getFirstPivot());

        SmartDashboard.putNumber("[ARM] 2nd PID Setpoint", secondPivotPID.getGoal().position);
        SmartDashboard.putNumber("[ARM] 2nd PID Output",
                calculateSecondPivotPID(getFirstPivotMotorDeg(), getTargetPosition().getSecondPivot()));
        SmartDashboard.putNumber("[ARM] 2nd PID Error", secondPivotPID.getPositionError());
        SmartDashboard.putNumber("[ARM] 2nd Target", getTargetPosition().getSecondPivot());
    }
}