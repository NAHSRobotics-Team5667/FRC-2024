// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

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
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.util.ArmAngle;

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

    private ProfiledPIDController firstPivotPID;
    private ProfiledPIDController secondPivotPID;

    private ArmAngle armPos = new ArmAngle();

    private StateManager states;

    // ========================================================
    // ==================== PID TUNING ========================

    // ShuffleboardTab tunerTab = Shuffleboard.getTab("Arm Tuning");

    // GenericEntry pEntry = tunerTab.add("Tune P", ArmConstants.FIRST_kP)
    // .withWidget(BuiltInWidgets.kTextView)
    // .withPosition(1, 1)
    // .getEntry();

    // GenericEntry iEntry = tunerTab.add("Tune I", ArmConstants.FIRST_kI)
    // .withWidget(BuiltInWidgets.kTextView)
    // .withPosition(1, 1)
    // .getEntry();

    // GenericEntry dEntry = tunerTab.add("Tune D", ArmConstants.FIRST_kD)
    // .withWidget(BuiltInWidgets.kTextView)
    // .withPosition(1, 1)
    // .getEntry();

    // ========================================================
    // ============= CLASS & SINGLETON SETUP ==================

    // SINGLETON ----------------------------------------------

    private static ArmSubsystem instance = null;

    private ArmSubsystem() {
        // ==== STATES ====

        states = StateManager.getInstance();

        // ==== PID ====

        firstPivotPID = new ProfiledPIDController(
                ArmConstants.FIRST_kP /* pEntry.getDouble(ArmConstants.FIRST_kP) */,
                ArmConstants.FIRST_kI /* iEntry.getDouble(ArmConstants.FIRST_kI) */,
                ArmConstants.FIRST_kD /* dEntry.getDouble(ArmConstants.FIRST_kD) */,
                new TrapezoidProfile.Constraints( // sets trapezoid profile for first pivot velocity
                        ArmConstants.FIRST_MAX_VELOCITY,
                        ArmConstants.FIRST_MAX_ACCEL));

        secondPivotPID = new ProfiledPIDController(
                ArmConstants.SECOND_kP,
                ArmConstants.SECOND_kI,
                ArmConstants.SECOND_kD,
                new TrapezoidProfile.Constraints( // trapezoid second pivot velocity
                        ArmConstants.SECOND_MAX_VELOCITY,
                        ArmConstants.SECOND_MAX_ACCEL));

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
        // TODO: fix when second stage motor is swapped
        // m_secondStageFollower.setControl(new
        // Follower(ArmConstants.SECOND_PIVOT_LEAD_ID, true)); // follow leader
        m_secondStageFollower.setNeutralMode(NeutralModeValue.Coast);

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
     * Set the relative velocity of the first pivot. Positive is away from resting
     * position.
     * 
     * @param speed % output for both motors controlling first pivot.
     */
    public void setFirstPivot(double speed) {
        double motorSpeed = speed / 100;

        m_firstStageLead.set(motorSpeed);
        m_firstStageFollower.setControl(new Follower(ArmConstants.FIRST_PIVOT_LEAD_ID, true));
    }

    /**
     * @param targetPos PID setpoint in degrees.
     * @return PID calculation in response.
     */
    public double calculateFirstPivotPID(double currentPos, double targetPos) {
        return MathUtil.clamp(
                firstPivotPID.calculate(currentPos, targetPos),
                -0.4, 1);
    }

    /**
     * Moves first pivot to the target using trapezoid profiled PID.
     * 
     * @param targetPos target for arm in degrees.
     */
    public void firstPivotToTargetPID(double targetPos) {
        double output = calculateFirstPivotPID(getFirstPivotMotorDeg(), targetPos);
        setFirstPivot(output * 100);
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
     * @param speed % output for both motors controlling second pivot.
     */
    public void setSecondPivot(double speed) {
        double motorSpeed = speed / 100;

        m_secondStageLead.set(motorSpeed);
        // m_secondStageFollower.setControl(new
        // Follower(ArmConstants.SECOND_PIVOT_LEAD_ID, false));
    }

    /**
     * @param targetPos PID setpoint in degrees.
     * @return PID calculation in response.
     */
    public double calculateSecondPivotPID(double currentPos, double targetPos) {
        return MathUtil.clamp(
                secondPivotPID.calculate(currentPos, targetPos),
                -1, // max going up
                1); // max going down
    }

    /**
     * Moves first pivot to the target using trapezoid profiled PID.
     * 
     * @param targetPos target for arm in degrees.
     */
    public void secondPivotToTargetPID(double targetPos) {
        double output = calculateSecondPivotPID(getSecondPivotMotorDeg(), targetPos);
        setSecondPivot(output * 100);
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
     * @return whether first pivot is at target.
     */
    public boolean firstPivotAtTarget() {
        return getFirstPivotAbsDeg() >= states.getTargetArmAngle().getFirstPivot()
                - ArmConstants.FIRST_ERR_MARGIN_DEG
                && getFirstPivotAbsDeg() <= states.getTargetArmAngle().getFirstPivot()
                        + ArmConstants.FIRST_ERR_MARGIN_DEG;
    }

    /**
     * @return whether second pivot is at target.
     */
    public boolean secondPivotAtTarget() {
        return getSecondPivotAbsDeg() >= states.getTargetArmAngle().getSecondPivot()
                - ArmConstants.SECOND_ERR_MARGIN_DEG
                && getSecondPivotAbsDeg() <= states.getTargetArmAngle().getSecondPivot()
                        + ArmConstants.SECOND_ERR_MARGIN_DEG;
    }

    /**
     * @return whether both pivots are at targets.
     */
    public boolean armAtTarget() {
        return firstPivotAtTarget() && secondPivotAtTarget();
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
    // ===================== PERIODIC =========================

    @Override
    public void periodic() { // This method will be called once per scheduler run - every 20ms.
        // PID Tuning ----------------------------------------

        // firstPivotPID.setPID(
        // pEntry.getDouble(ArmConstants.FIRST_kP),
        // iEntry.getDouble(ArmConstants.FIRST_kI),
        // dEntry.getDouble(ArmConstants.FIRST_kD));

        // update arm position with encoders -----------------
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

        if (Math.abs(getFirstPivotAbsDeg() - getFirstPivotMotorDeg()) > ArmConstants.ACC_FIRST_PIVOT_DIFF) {
            // initial motor positions - first stage
            m_firstStageLead.getConfigurator()
                    .setPosition(Units.degreesToRotations(getFirstPivotAbsDeg()));
            m_firstStageFollower.getConfigurator()
                    .setPosition(Units.degreesToRotations(getFirstPivotAbsDeg()));
        }

        if (Math.abs(getSecondPivotAbsDeg() - getSecondPivotMotorDeg()) > ArmConstants.ACC_SECOND_PIVOT_DIFF) {
            // initial motor positions - second stage
            m_secondStageLead.getConfigurator()
                    .setPosition(Units.degreesToRotations(getSecondPivotAbsDeg()));
            m_secondStageFollower.getConfigurator()
                    .setPosition(Units.degreesToRotations(getSecondPivotAbsDeg()));
        }

        // ---------------------------------------------------
        // ------------------- TELEMETRY ---------------------

        // First Encoder -------------------------------------

        /* USE FOR TROUBLESHOOTING */

        // SmartDashboard.putNumber("[ARM] Raw First Abs Encoder 1",
        // getRawFirstEncoders()[0]);
        // SmartDashboard.putNumber("[ARM] Raw First Abs Encoder 2",
        // getRawFirstEncoders()[1]);

        // SmartDashboard.putNumber("[ARM] Offset First Abs Encoder 1",
        // getOffsetFirstEncoders()[0]);
        // SmartDashboard.putNumber("[ARM] Offset First Abs Encoder 2",
        // getOffsetFirstEncoders()[1]);

        /*------------------------ */

        SmartDashboard.putNumber("[ARM] Abs First Pivot Deg", getFirstPivotAbsDeg());
        SmartDashboard.putNumber("[ARM] Motor First Pivot Deg", getFirstPivotMotorDeg());

        // Second Encoder ------------------------------------

        /* USE FOR TROUBLESHOOTING */

        // SmartDashboard.putNumber("[ARM] Raw Second Abs Encoder 1",
        // getRawSecondEncoders()[0]);
        // SmartDashboard.putNumber("[ARM] Raw Second Abs Encoder 2",
        // getRawSecondEncoders()[1]);

        // SmartDashboard.putNumber("[ARM] Offset Second Abs Encoder 1",
        // getOffsetSecondEncoders()[0]);
        // SmartDashboard.putNumber("[ARM] Offset Second Abs Encoder 2",
        // getOffsetSecondEncoders()[1]);

        /*------------------------ */

        SmartDashboard.putNumber("[ARM] Abs Second Pivot Deg", getSecondPivotAbsDeg());
        SmartDashboard.putNumber("[ARM] Motor Second Pivot Deg", getSecondPivotMotorDeg());

        // States --------------------------------------------
        SmartDashboard.putString("[ARM] State", states.getArmState().toString());

        // Limit Switch --------------------------------------
        SmartDashboard.putBoolean("[ARM] Limit Switch", getLimitSwitch());

        // PID -----------------------------------------------
        SmartDashboard.putNumber("[ARM] 1st PID Setpoint", firstPivotPID.getGoal().position);
        SmartDashboard.putNumber("[ARM] 1st PID Output",
                calculateFirstPivotPID(getFirstPivotMotorDeg(), states.getTargetArmAngle().getFirstPivot()));
        SmartDashboard.putNumber("[ARM] 1st PID Error", firstPivotPID.getPositionError());
        // SmartDashboard.putNumber("[ARM] 1st Target",
        // getTargetPosition().getFirstPivot());

        SmartDashboard.putNumber("[ARM] 2nd PID Setpoint", secondPivotPID.getGoal().position);
        SmartDashboard.putNumber("[ARM] 2nd PID Output",
                calculateSecondPivotPID(getSecondPivotMotorDeg(), states.getTargetArmAngle().getSecondPivot()));
        SmartDashboard.putNumber("[ARM] 2nd PID Error", secondPivotPID.getPositionError());
        // SmartDashboard.putNumber("[ARM] 2nd Target",
        // getTargetPosition().getSecondPivot());
    }
}