// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

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
        // ====================== MOTION =========================

        // ==== PID ====
        // ---- FRONT LEFT ---
        public static final double FL_DRIVE_P = -1;
        public static final double FL_DRIVE_I = -1;
        public static final double FL_DRIVE_D = -1;
        public static final double FL_DRIVE_F = -1;

        public static final double FL_TURN_P = -1;
        public static final double FL_TURN_I = -1;
        public static final double FL_TURN_D = -1;
        public static final double FL_TURN_F = -1;

        // ---- FRONT RIGHT ---
        public static final double FR_DRIVE_P = -1;
        public static final double FR_DRIVE_I = -1;
        public static final double FR_DRIVE_D = -1;
        public static final double FR_DRIVE_F = -1;

        public static final double FR_TURN_P = -1;
        public static final double FR_TURN_I = -1;
        public static final double FR_TURN_D = -1;
        public static final double FR_TURN_F = -1;

        // ---- BACK LEFT ---
        public static final double BL_DRIVE_P = -1;
        public static final double BL_DRIVE_I = -1;
        public static final double BL_DRIVE_D = -1;
        public static final double BL_DRIVE_F = -1;

        public static final double BL_TURN_P = -1;
        public static final double BL_TURN_I = -1;
        public static final double BL_TURN_D = -1;
        public static final double BL_TURN_F = -1;

        // ---- BACK RIGHT ---
        public static final double BR_DRIVE_P = -1;
        public static final double BR_DRIVE_I = -1;
        public static final double BR_DRIVE_D = -1;
        public static final double BR_DRIVE_F = -1;

        public static final double BR_TURN_P = -1;
        public static final double BR_TURN_I = -1;
        public static final double BR_TURN_D = -1;
        public static final double BR_TURN_F = -1;

        // =======================================================
        // ======================= AUTO ==========================
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
    }

    public static class IntakeConstants {
    }

    public static class ClimbConstants {
    }
}
