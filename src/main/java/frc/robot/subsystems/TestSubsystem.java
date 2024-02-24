// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/**
 * Used to test individual motors.
 */
public class TestSubsystem extends SubsystemBase {
    private TalonFX fLDrive, fLTurn,
            fRDrive, fRTurn,
            bLDrive, bLTurn,
            bRDrive, bRTurn;

    private static TestSubsystem instance = null;

    /** Creates a new TestSubsystem. */
    private TestSubsystem() {
        fLDrive = new TalonFX(DriveConstants.FRONT_LEFT_DRIVE_ID);
        fRDrive = new TalonFX(DriveConstants.FRONT_RIGHT_DRIVE_ID);
        bLDrive = new TalonFX(DriveConstants.BACK_LEFT_DRIVE_ID);
        bRDrive = new TalonFX(DriveConstants.BACK_RIGHT_DRIVE_ID);

        fLTurn = new TalonFX(DriveConstants.FRONT_LEFT_TURN_ID);
        fRTurn = new TalonFX(DriveConstants.FRONT_RIGHT_TURN_ID);
        bLTurn = new TalonFX(DriveConstants.BACK_LEFT_TURN_ID);
        bRTurn = new TalonFX(DriveConstants.BACK_RIGHT_TURN_ID);
    }

    public static TestSubsystem getInstance() {
        if (instance == null) {
            instance = new TestSubsystem();
        }

        return instance;
    }

    /**
     * Used to run any motors that need to be tested.
     * 
     * @param speed percent output from 0-100.
     */
    public void runMotors(double speed) {
        speed /= 100;
        // fLDrive.set(speed);
        // fRDrive.set(speed);
        // bLDrive.set(speed);
        // bRDrive.set(speed);

        // fLTurn.set(speed);
        // fRTurn.set(speed);
        bLTurn.set(speed);
        // bRTurn.set(speed);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}
