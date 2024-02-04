// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * IntakeSubsystem.java
 * 
 * Refers to the Intake. Responsible for picking up rings off of the ground.
 * 
 * Motors:
 * - Intake - NEO 1.1 Brushless Motors
 * 
 * - 2 Pistons connected to 1 Solenoid
 */
public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax m_intake; // Declaring motor.
    private Solenoid m_piston; // Declare solenoid.

    // ========================================================
    // ============= CLASS & SINGLETON SETUP ==================

    // SINGLETON ----------------------------------------------

    private static IntakeSubsystem instance = null;

    private IntakeSubsystem() {
        // Initialize NEO 1.1 Motors.
        m_intake = new CANSparkMax(IntakeConstants.SPARKMAX_ID, MotorType.kBrushless);

        // Initialize Pistons by setting IDs.
        m_piston = new Solenoid(PneumaticsModuleType.REVPH, IntakeConstants.SOLENOID_PORT);
    }

    /*
     * This method is invoked when the robot is first initialized.
     * Checks if there is an instance. If there is, return that instance.
     * If there's no instance, creates one.
     */
    public static IntakeSubsystem getInstance() {
        if (instance == null)
            instance = new IntakeSubsystem();

        return instance;
    }

    // PERIODIC -----------------------------------------------

    @Override
    public void periodic() {
        // This method will be called once per scheduler run - every 20ms.
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
        m_piston.set(extend);
    }

    // ========================================================
    // ===================== SENSORS ==========================

    /**
     * @return whether intake is deployed.
     */
    public boolean isIntakeDeployed() {
        return m_piston.get();
    }

    /**
     * @return whether intake is running.
     */
    public boolean isIntakeRunning() {

        boolean isRunning = false;

        //Checks the speed. If it's not 0, it's running the motors.
        if (m_intake.get() != 0){
            isRunning = true;
        }

        return isRunning;
    }

    // ========================================================
    // ======================= OTHER ==========================
}
