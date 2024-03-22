// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexConstants;

public class IndexSubsystem extends SubsystemBase {
    private CANSparkMax index;
    private DigitalInput beamBreak;

    private static IndexSubsystem instance = null;

    // ========================================================
    // =================== CONSTRUCTOR ========================

    /** Creates a new IndexSubsystem. */
    private IndexSubsystem() {
        index = new CANSparkMax(IndexConstants.BELT_INDEX_ID, MotorType.kBrushless);

        beamBreak = new DigitalInput(IndexConstants.BEAM_BREAK_CHANNEL_ID);
    }

    // ========================================================
    // ===================== SINGLEeTON ========================

    public static IndexSubsystem getInstance() {
        if (instance == null) {
            instance = new IndexSubsystem();
        }

        return instance;
    }

    // ========================================================
    // =================== MOTOR ACTIONS ======================

    /**
     * Sets index velocity.
     * 
     * @param speed percent output of index. Positive value is toward shooter. 0-100
     *              scale.
     */
    public void set(double speed) {
        speed /= 100;
        index.set(speed);
    }

    // ========================================================
    // ====================== SENSORS =========================

    /**
     * @return whether beam is broken, indicating presence of ring in holding.
     */
    public boolean hasGamePiece() {
        return !beamBreak.get();
    }

    // ========================================================
    // ===================== PERIODIC =========================

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("[INDEX] Beam Break", hasGamePiece());
    }
}
