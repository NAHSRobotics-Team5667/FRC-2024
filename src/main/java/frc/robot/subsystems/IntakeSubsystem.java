// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * IntakeSubsystem.java
 * 
 * Refers to the Intake. Responsible for picking up rings off of the ground.
 * 
 * Motors:
 * - Intake - NEO
 * 
 * - 2 Pistons connected to 1 Solenoid
 */
public class IntakeSubsystem extends SubsystemBase {
    private TalonFX m_intake = new TalonFX(0); // declaring motors and global scope
    private Solenoid m_piston = new Solenoid(null, 0); // declare solenoid

    // ========================================================
    // ============= CLASS & SINGLETON SETUP ==================

    // SINGLETON ----------------------------------------------

    private static IntakeSubsystem instance = null;

    private IntakeSubsystem() {
        // Initialize motors

        // Initialize piston
    
    }

    public static IntakeSubsystem getInstance() {
        if (instance == null)
            instance = new IntakeSubsystem();

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
     * Sets relative velocity of the intake.
     * 
     * @param percentOutput % output of intake motor.
     */
    public void setIntakeSpeed(double percentOutput) {

        double motorSpeed = percentOutput / 100;
        m_intake.set(motorSpeed);

    }

    /**
     * Sets pistons to either extend or retract.
     * 
     * @param extend boolean value for detemining if piston is extending or
     *               retracting. true = extend, false = retract.
     */
    public void setPiston(boolean extend) {

        if (extend = true){
// Extend the piston
        } else {
// Retract the pistons
        }
    }

    // ========================================================
    // ===================== SENSORS ==========================

    /**
     * @return whether intake is deployed.
     */
    public boolean isIntakeDeployed() {
        return false; // TODO: check if intake is deployed
    }

    /**
     * @return whether intake is running.
     */
    public boolean isIntakeRunning() {
        return false; // TODO: check if motor is spinning via encoder
    }

    // ========================================================
    // ======================= OTHER ==========================
}