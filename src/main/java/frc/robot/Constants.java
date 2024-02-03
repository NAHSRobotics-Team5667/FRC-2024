// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.util.Units;
import frc.robot.util.ArmAngle;
import frc.robot.util.States.ArmState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        // =======================================================
        // ==================== DRIVER ===========================

        public static final double LEFT_X_DEADBAND = 0.1;
        public static final double LEFT_Y_DEADBAND = 0.1;
        public static final double RIGHT_X_DEADBAND = 0.1;
        public static final double RIGHT_Y_DEADBAND = 0.1;
    }

    // Contains all of our Drive Constants for Swerve Drive.
    public static class DriveConstants {

        // =======================================================
        // ================= MOTOR IDS ===========================

        // ---- FRONT LEFT ----
        public static final int FRONT_LEFT_DRIVE_ID = -1;
        public static final int FRONT_LEFT_TURN_ID = -1;

        // ---- FRONT RIGHT ----
        public static final int FRONT_RIGHT_DRIVE_ID = -1;
        public static final int FRONT_RIGHT_TURN_ID = -1;

        // ---- BACK LEFT ----
        public static final int BACK_LEFT_DRIVE_ID = -1;
        public static final int BACK_LEFT_TURN_ID = -1;

        // ---- BACK RIGHT ----
        public static final int BACK_RIGHT_DRIVE_ID = -1;
        public static final int BACK_RIGHT_TURN_ID = -1;

        // =======================================================
        // ====================== PID ============================

        public static final double AUTO_DRIVE_P = 5.0;
        public static final double AUTO_DRIVE_I = 0;
        public static final double AUTO_DRIVE_D = 0;
        public static final double AUTO_DRIVE_F = 0;

        // =======================================================
        // ======================= GENERAL =======================

        public static final double MAX_VELOCITY_FEET = 16.5;
        public static final double MAX_VELOCITY_METERS = Units.feetToMeters(MAX_VELOCITY_FEET);

        public static final double MAX_ACCELERATION_METERS = 5.0;
    }

    public static class ArmConstants {

        // =======================================================
        // ================= MOTOR IDS ===========================

        // ---- FIRST PIVOT ----
        public static final int FIRST_PIVOT_LEAD_ID = -1;
        public static final int FIRST_PIVOT_FOLLOWER_ID = -1;

        // ---- SECOND PIVOT ----
        public static final int SECOND_PIVOT_LEAD_ID = -1;
        public static final int SECOND_PIVOT_FOLLOWER_ID = -1;

        // =======================================================
        // ===================== ENCODERS ========================

        // ---- FIRST PIVOT ----
        public static final double FIRST_PIVOT_LEFT_OFFSET = -1;
        public static final double FIRST_PIVOT_RIGHT_OFFSET = -1;

        // ---- SECOND PIVOT ----
        public static final double SECOND_PIVOT_LEFT_OFFSET = -1;
        public static final double SECOND_PIVOT_RIGHT_OFFSET = -1;

        // =======================================================
        // ================= ARM POSITION ========================

        public static final ArmAngle INITIAL_POSITION = new ArmAngle(
                -1,
                -1); // TODO: determine initial angles of arm

        // create a map of goal states and their target arm positions
        public static final Map<ArmState, ArmAngle> GOAL_POSITIONS = Map.of(
                ArmState.RESTING, INITIAL_POSITION, // arm position when at resting position
                ArmState.DEFAULT_SPEAKER, new ArmAngle(), // default position for speaker
                ArmState.AMP, new ArmAngle(), // position for amp
                ArmState.TRAP, new ArmAngle(), // position for trap
                ArmState.HUMAN_PLAYER, new ArmAngle()); // position for human player intake

        /**
         * Returns the goal position for a given target.
         * 
         * @param key target position.
         * @return goal position associated with target.
         */
        public static ArmAngle getGoalPosition(ArmState key) {
            return GOAL_POSITIONS.get(key);
        }

        // =======================================================
        // ====================== MOTION =========================

        // ==== PID ====
        // ---- FIRST PIVOT ----
        public static final double FIRST_P = -1;
        public static final double FIRST_I = -1;
        public static final double FIRST_D = -1;

        public static final double FIRST_F = -1; // if using feedforward

        // ---- SECOND PIVOT ----
        public static final double SECOND_P = -1;
        public static final double SECOND_I = -1;
        public static final double SECOND_D = -1;

        public static final double SECOND_F = -1; // if using feedforward
    }

    public static class ShooterConstants {
        // ==== MOTORS ====
        public static final int SHOOTER_LEFT_ID = 0;
        public static final int SHOOTER_RIGHT_ID = 0;

        // ---- BEAM BREAK ----
        public static final int BEAM_BREAK_CHANNEL_ID = 0; // Goes into 2 Seperate Digital Inputs. We need to know which one is used here.

        // ---- MAXIMUM RPM ----
        public static final double SHOOTER_MAX_RPM = 6000;
        public static final double INDEX_MAX_RPM = 11000;
    }

    public static class IntakeConstants {
    }

    public static class ClimbConstants {
    }
}
