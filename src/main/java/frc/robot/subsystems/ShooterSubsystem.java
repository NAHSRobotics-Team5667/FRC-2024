// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.util.ArmAngle;
import frc.robot.util.States.ArmState;
import frc.robot.util.States.ShooterState;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
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
    private TalonFX m_topShooter;
    private TalonFX m_bottomShooter; // declaring motors and global scope

    private double targetLeftRPM = ShooterConstants.SHOOTER_MAX_RPM;
    private double targetRightRPM = ShooterConstants.SHOOTER_MAX_RPM;

    // ========================================================
    // ============= CLASS & SINGLETON SETUP ==================

    // SINGLETON ----------------------------------------------

    private static ShooterSubsystem instance = null;

    private ShooterSubsystem() {

        // Initialize Motors (Falcon 500s).
        m_topShooter = new TalonFX(ShooterConstants.SHOOTER_TOP_ID);
        m_topShooter.setNeutralMode(NeutralModeValue.Coast);
        m_topShooter.setInverted(true);

        m_bottomShooter = new TalonFX(ShooterConstants.SHOOTER_BOTTOM_ID);
        m_bottomShooter.setNeutralMode(NeutralModeValue.Coast);
        m_bottomShooter.setInverted(true);
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
     * @param left  % output for left motor.
     * @param right % output for right motor.
     */
    public void set(double left, double right) {
        double leftOutput = left / 100;
        double rightOutput = right / 100;

        targetLeftRPM = leftOutput * ShooterConstants.SHOOTER_MAX_RPM;
        targetRightRPM = rightOutput * ShooterConstants.SHOOTER_MAX_RPM;

        m_topShooter.set(leftOutput);
        m_bottomShooter.set(rightOutput);
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
