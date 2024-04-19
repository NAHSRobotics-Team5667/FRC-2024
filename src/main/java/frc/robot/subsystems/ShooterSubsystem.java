// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private TalonFX m_topShooter;
    private TalonFX m_bottomShooter; // declaring motors and global scope

    private CANSparkMax m_fan;

    private double targetLeftRPM = ShooterConstants.SHOOTER_MAX_RPM;
    private double targetRightRPM = ShooterConstants.SHOOTER_MAX_RPM;

    // ========================================================
    // ============= CLASS & SINGLETON SETUP ==================

    // SINGLETON ----------------------------------------------

    private static ShooterSubsystem instance = null;

    private ShooterSubsystem() {
        // Initialize Shooter Motors (Falcon 500s).
        m_topShooter = new TalonFX(ShooterConstants.SHOOTER_TOP_ID);
        m_topShooter.setNeutralMode(NeutralModeValue.Coast);
        m_topShooter.setInverted(true);

        m_bottomShooter = new TalonFX(ShooterConstants.SHOOTER_BOTTOM_ID);
        m_bottomShooter.setNeutralMode(NeutralModeValue.Coast);
        m_bottomShooter.setInverted(true);

        // Initialize Fan Motor
        m_fan = new CANSparkMax(ShooterConstants.FAN_ID, MotorType.kBrushless);
        m_fan.setSmartCurrentLimit(20); // set current limit of neo550 to 20A to avoid motor failure
        m_fan.setInverted(true);
    }

    public static ShooterSubsystem getInstance() {
        if (instance == null)
            instance = new ShooterSubsystem();

        return instance;
    }

    // ========================================================
    // ================== MOTOR ACTIONS =======================

    // SHOOTER ------------------------------------------------

    /**
     * Sets speed of the shooter for both sets of wheels. 0-100.
     * 
     * @param percentOutput % output for both motors.
     */
    public void set(double percentOutput) {
        double output = percentOutput / 100;

        targetLeftRPM = output * ShooterConstants.SHOOTER_MAX_RPM;
        targetRightRPM = output * ShooterConstants.SHOOTER_MAX_RPM;

        m_topShooter.set(output);
        m_bottomShooter.set(output);
    }

    /**
     * Sets speed of shooter wheels with separate speeds for each. Use for spin and
     * lateral adjustments. 0-100;
     * 
     * @param top    % output for left motor.
     * @param bottom % output for right motor.
     */
    public void set(double top, double bottom) {
        double topOutput = top / 100;
        double bottomOutput = bottom / 100;

        targetLeftRPM = topOutput * ShooterConstants.SHOOTER_MAX_RPM;
        targetRightRPM = bottomOutput * ShooterConstants.SHOOTER_MAX_RPM;

        m_topShooter.set(topOutput);
        m_bottomShooter.set(bottomOutput);
    }

    /**
     * Turns fan on or off.
     *
     * @param on whether fan should be turned on or off.
     */
    public void setFan(boolean on) {
        double fanSpeed = ShooterConstants.TRAP_FAN_SPEED / 100;

        m_fan.set((on) ? fanSpeed : 0);
    }

    // ========================================================
    // ===================== SENSORS ==========================

    // SHOOTER ------------------------------------------------

    /**
     * @return RPM of left set of wheels.
     */
    public double getLeftShooterRPM() {
        double leftSpeed = m_topShooter.getVelocity().getValueAsDouble();
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
        double rightSpeed = m_bottomShooter.getVelocity().getValueAsDouble();
        return rightSpeed;
    }

    // ========================================================
    // ====================== PERIODIC ========================

    @Override
    public void periodic() { // This method will be called once per scheduler run

        // Telemetry ------------------------------------------

        SmartDashboard.putNumber("[SHOOTER] Right RPM", getRightShooterRPM());
        SmartDashboard.putNumber("[SHOOTER] Left RPM", getLeftShooterRPM());

        SmartDashboard.putNumber("[SHOOTER] Target Right RPM", targetRightRPM);
        SmartDashboard.putNumber("[SHOOTER] Target Left RPM", targetLeftRPM);

        SmartDashboard.putString("[SHOOTER] State", StateManager.getInstance().getShooterState().toString());
    }
}
