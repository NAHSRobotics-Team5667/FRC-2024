// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.util.States.ShooterStates;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    // private IndexStates indexState = null; // current state of index

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

        m_rightShooter.setInverted(true);

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

        targetLeftRPM = output * ShooterConstants.SHOOTER_MAX_RPM;
        targetRightRPM = output * ShooterConstants.SHOOTER_MAX_RPM;

        m_leftShooter.set(output);
        m_rightShooter.set(output);
    }

    /**
     * Sets speed of shooter wheels with separate speeds for each. Use for spin and
     * lateral adjustments. 0-100;
     * 
     * @param left  % output for left motor.
     * @param right % output for right motor.
     */
    public void setShooterSpeed(double left, double right) {
        double leftOutput = left / 100;
        double rightOutput = right / 100;

        targetLeftRPM = leftOutput * ShooterConstants.SHOOTER_MAX_RPM;
        targetRightRPM = rightOutput * ShooterConstants.SHOOTER_MAX_RPM;

        m_leftShooter.set(leftOutput);
        m_rightShooter.set(rightOutput);
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
     * @return target left RPM.
     */
    public double getTargetLeftRPM() {
        return targetLeftRPM;
    }

    /**
     * @return target right RPM.
     */
    public double getTargetRightRPM() {
        return targetRightRPM;
    }

    /**
     * @return RPM of right set of wheels.
     */
    public double getRightShooterRPM() {
        double rightSpeed = m_rightShooter.getVelocity().getValueAsDouble();
        return rightSpeed;
    }

    /**
     * @return whether left wheels are at target RPM.
     */
    public boolean leftWheelsReady() {
        return getLeftShooterRPM() >= targetLeftRPM - ShooterConstants.RPM_ERROR_MARGIN
                && getLeftShooterRPM() <= targetLeftRPM + ShooterConstants.RPM_ERROR_MARGIN;
    }

    /**
     * @return whether right wheels are at target RPM.
     */
    public boolean rightWheelsReady() {
        return getRightShooterRPM() >= targetRightRPM - ShooterConstants.RPM_ERROR_MARGIN
                && getRightShooterRPM() <= targetRightRPM + ShooterConstants.RPM_ERROR_MARGIN;
    }

    // ========================================================
    // ======================= STATE ==========================

    /**
     * @return current shooter state.
     */
    public ShooterStates getShooterState() {
        return shooterState;
    }

    // ========================================================
    // ====================== PERIODIC ========================

    @Override
    public void periodic() { // This method will be called once per scheduler run

        // adjust shooter state passively
        if (leftWheelsReady() && rightWheelsReady()) {
            shooterState = ShooterStates.READY;
        } else if (getLeftShooterRPM() == 0 && getRightShooterRPM() == 0) {
            shooterState = ShooterStates.STOPPED; // shooter stopped
        } else {
            shooterState = ShooterStates.ADJUST_VEL; // shooter not at correct speed
        }

        // Telemetry ------------------------------------------

        SmartDashboard.putNumber("[SHOOTER] Right RPM", getRightShooterRPM());
        SmartDashboard.putNumber("[SHOOTER] Left RPM", getLeftShooterRPM());

        SmartDashboard.putNumber("[SHOOTER] Target Right RPM", targetRightRPM);
        SmartDashboard.putNumber("[SHOOTER] Target Left RPM", targetLeftRPM);

        SmartDashboard.putString("[SHOOTER] State", shooterState.toString());
        SmartDashboard.putBoolean("[SHOOTER] Beambreak", this.hasGamePiece());
    }
}
