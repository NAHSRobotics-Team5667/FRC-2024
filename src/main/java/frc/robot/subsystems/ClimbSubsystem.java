// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbConstants;


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
 * - Left Absolute Encoder (check WCP GreyT Elevator) - CANcoder.
 * - Right Absolute Encoder (check WCP GreyT Elevator) - CANcoder.
 */
public class ClimbSubsystem extends SubsystemBase {
    private TalonFX m_leftClimb, m_rightClimb; // declaring motors and global scope
    private CANcoder leftEncoder, rightEncoder; // absolute encoders
    // ========================================================
    // ============= CLASS & SINGLETON SETUP ==================

    // SINGLETON ----------------------------------------------

    private static ClimbSubsystem instance = null;

    private ClimbSubsystem() {

        // Initialize Falcon Motors by setting IDs.
        m_leftClimb = new TalonFX(ClimbConstants.LEFT_CLIMB_ID); 
        m_rightClimb = new TalonFX(ClimbConstants.RIGHT_CLIMB_ID);

        // Initialize encoders (CANcoders) by setting IDs.
        leftEncoder = new CANcoder(ClimbConstants.LEFT_CLIMB_ENCODER_ID); 
        rightEncoder = new CANcoder(ClimbConstants.RIGHT_CLIMB_ENCODER_ID);
    }

    /*
     * This method is invoked when the robot is first initialized.
     * Checks if there is an instance. If there is, return that instance.
     * If there's no instance, creates one.
     */
    public static ClimbSubsystem getInstance() {
        if (instance == null)
            instance = new ClimbSubsystem();

        return instance;
    }

    // ========================================================
    // ================== MOTOR ACTIONS =======================

    public void Distance() {
        
        rightEncoder.setControl(null);
        leftEncoder.setControl(null);

        m_leftClimb.setInverted(false);
        m_rightClimb.setInverted(true);

        // Control the distance for every motor rotation

        // the distance per rotation method just sets up the conversion rate from
        // encoder rotations to the distance that it travels linearly. e.g. a wheel's
        // distance per rotation would be its circumference

        //get climb up onto chains

        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
        // set rotation distance ^

        //set distances when ready
        //this is to lift the rbt off the ground
        
        if (getLeftClimbHeight() == 0) {
            leftEncoder.setPosition(-0);
             
             //when rbt reaches desired height off ground
             if (getLeftClimbHeight() == 0) {
                m_leftClimb.set(0);
             }  
        }

        if (getRightClimbHeight() == 0) {
            rightEncoder.setPosition(-0); 

             //when rbt reaches desired height off ground
            if (getRightClimbHeight() == 0) {
                m_rightClimb.set(0);
            } 
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
        leftEncoder.getPosition();
        return getLeftClimbHeight();
    }

    /**
     * @return right climb hook height.
     */
    public double getRightClimbHeight() {
        rightEncoder.getPosition();
        return getRightClimbHeight();
    }

    // ========================================================
    // ======================= OTHER ==========================


    // PERIODIC -----------------------------------------------
    @Override
    public void periodic() {
        // This method will be called once per scheduler run. This is normally every 20ms.
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run. This is normally every 20ms. This only runs during simulation.
    }
}