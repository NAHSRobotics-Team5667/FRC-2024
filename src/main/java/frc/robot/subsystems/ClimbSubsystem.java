// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * ClimbSubsystem.java
 * 
 * Refers to the Climb. Two climbers responsible for lifting robot off of the
 * ground during endgame.
 * 
 * Motors:
 * - Left Climb - Falcon 500
 * - Right Climb - Falcon 500
 * 
 * Sensors:
 * - Left Absolute Encoder (check WCP GreyT Elevator)
 * - Right Absolute Encoder (check WCP GreyT Elevator)
 */
public class ClimbSubsystem extends SubsystemBase {
    private TalonFX m_leftClimb, m_rightClimb; // declaring motors and global scope
    private DutyCycleEncoder leftEncoder, rightEncoder;
    // ========================================================
    // ============= CLASS & SINGLETON SETUP ==================

    // SINGLETON ----------------------------------------------

    private static ClimbSubsystem instance = null;

    private ClimbSubsystem() {
        // Initialize motors

        m_leftClimb = new TalonFX(0);
        m_rightClimb = new TalonFX(0);
        //set ID's

        // Initialize encoders

        DutyCycleEncoder leftEncoder = new DutyCycleEncoder(0);
        DutyCycleEncoder rightEncoder = new DutyCycleEncoder(0);
        //set channel
    
    }

    public static ClimbSubsystem getInstance() {
        if (instance == null)
            instance = new ClimbSubsystem();

        return instance;
    }

    // PERIODIC -----------------------------------------------

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        rightEncoder.getAbsolutePosition();
        leftEncoder.getAbsolutePosition();
    }

    // ========================================================
    // ================== MOTOR ACTIONS =======================

    public void Distance(){

        // Control the distance for every motor rotation 

        leftEncoder.setDistancePerRotation(0);
        rightEncoder.setDistancePerRotation(0);
        //set rotation distance

        double lDistance = leftEncoder.getDistance();
        double rDistance = rightEncoder.getDistance();

        if (lDistance == 0) {
              leftEncoder.setDistancePerRotation(0);
        }


         if (rDistance == 0) {
              rightEncoder.setDistancePerRotation(0);
            
        }

    }

 
    
    /**
     * Sets relative velocity of the climb. Positive is upwards, negative is
     * downwards.
     * 
     * @param percentOutput % output of climb motor.
     */
    public void setClimbSpeed(double percentOutput) {
        double motorSpeed = percentOutput / 100;

        m_leftClimb.set(motorSpeed);
        m_rightClimb.set(motorSpeed);

    }

    // ========================================================
    // ===================== SENSORS ==========================

    /**
     * @return left climb hook height.
     */
    public double getLeftClimbHeight() {
        leftEncoder.getDistance();
        return 0; 

    }

    /**
     * @return right climb hook height.
     */
    public double getRightClimbHeight() {
        rightEncoder.getDistance();
        return 0; 


    }

    // ========================================================
    // ======================= OTHER ==========================
}
