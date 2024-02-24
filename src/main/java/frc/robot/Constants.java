// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import frc.robot.util.ArmAngle;
import frc.robot.util.States.ArmPosState;

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
        // The following IDs constants have been commented out.
        // The IDs for motors are located in src/main/deploy/swerve/* files.

        // ---- FRONT LEFT ----
        public static final int FRONT_LEFT_DRIVE_ID = 3;
        public static final int FRONT_LEFT_TURN_ID = 7;

        // ---- FRONT RIGHT ----
        public static final int FRONT_RIGHT_DRIVE_ID = 2;
        public static final int FRONT_RIGHT_TURN_ID = 6;

        // ---- BACK LEFT ----
        public static final int BACK_LEFT_DRIVE_ID = 5;
        public static final int BACK_LEFT_TURN_ID = 9;

        // ---- BACK RIGHT ----
        public static final int BACK_RIGHT_DRIVE_ID = 4;
        public static final int BACK_RIGHT_TURN_ID = 8;

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
        // ================== MECHANICAL =========================

        /** Gear ratio between first motors and first pivot. */
        public static final double FIRST_GEAR_RATIO = 100.0 * (60.0 / 44.0); // 100*(60/44):1
        /** Gear ratio between second motors and second pivot. */
        public static final double SECOND_GEAR_RATIO = 80.0; // 80:1 - torqued up by factor of 80

        // =======================================================
        // ================= MOTOR IDS ===========================

        // ---- FIRST PIVOT ----
        public static final int FIRST_PIVOT_LEAD_ID = 13;
        public static final int FIRST_PIVOT_FOLLOWER_ID = 12;

        // ---- SECOND PIVOT ----
        public static final int SECOND_PIVOT_LEAD_ID = 15;
        public static final int SECOND_PIVOT_FOLLOWER_ID = 14;

        // =======================================================
        // ===================== ENCODERS ========================

        // ---- FIRST PIVOT ----
        public static final int FIRST_ENC_PORT_1 = 7; // left side
        public static final int FIRST_ENC_PORT_2 = 8; // right side
        public static final double FIRST_LEFT_OFFSET = -0.703;
        public static final double FIRST_RIGHT_OFFSET = 0.472;
        /** Rotations of first pivot for each rotation of encoder. */
        public static final double FIRST_ENC_DIST_PER_ROT = (44.0 / 60.0) * 360.0;

        public static final double LIMIT_SWITCH = 0;

        // ---- SECOND PIVOT ----
        public static final int SECOND_ENC_PORT_1 = 6; // left side
        public static final int SECOND_ENC_PORT_2 = 9; // right side
        public static final double SECOND_LEFT_OFFSET = -0.162;
        public static final double SECOND_RIGHT_OFFSET = 0.936;
        /** Rotations of second pivot for each rotation of encoder. */
        public static final double SECOND_ENC_DIST_PER_ROT = 1.0 * 360.0;

        // =======================================================
        // ================= ARM POSITION ========================

        // create a map of goal states and their target arm positions
        public static Map<ArmPosState, ArmAngle> GOAL_POSITIONS = Map.of(
                ArmPosState.TRANSFER, new ArmAngle(12.2, -11.25), // arm position when at resting position
                ArmPosState.SPEAKER, new ArmAngle(50, 0), // default position for speaker
                ArmPosState.AMP, new ArmAngle(108.1, -122.7), // position for amp
                ArmPosState.TRAP, new ArmAngle(100, 100), // position for trap
                ArmPosState.CLIMB, new ArmAngle()); // position for human player intake

        // create a map of arm positions and their target goal states - maps aren't
        // bi-directional :(
        public static Map<ArmAngle, ArmPosState> ARM_STATES = Map.of(
                getGoalPosition(ArmPosState.TRANSFER), ArmPosState.TRANSFER, // arm position when at resting position
                getGoalPosition(ArmPosState.SPEAKER), ArmPosState.SPEAKER, // default position for speaker
                getGoalPosition(ArmPosState.AMP), ArmPosState.AMP, // position for amp
                getGoalPosition(ArmPosState.TRAP), ArmPosState.TRAP, // position for trap
                getGoalPosition(ArmPosState.CLIMB), ArmPosState.CLIMB); // position for human player
                                                                        // intake

        /**
         * Returns the goal position for a given target.
         * 
         * @param key target position.
         * @return goal position associated with target.
         */
        public static ArmAngle getGoalPosition(ArmPosState key) {
            return GOAL_POSITIONS.get(key);
        }

        /**
         * Updates ArmAngle linked to a given arm position. Only use when aiming shooter
         * at speaker.
         * 
         * @param key    arm position state to update.
         * @param newPos new position to link to arm position state.
         */
        public static void setGoalPosition(ArmPosState key, ArmAngle newPos) {
            GOAL_POSITIONS.put(key, newPos);
        }

        /**
         * Returns the goal position for a given target.
         * 
         * @param key arm angles that you want the linked state of.
         * @return linked state to arm angles.
         */
        public static ArmPosState getState(ArmAngle key) {
            return ARM_STATES.get(key);
        }

        // ==== POSITION ERROR MARGINS ====

        public static final double FIRST_ERR_MARGIN_DEG = 5.0;
        public static final double SECOND_ERR_MARGIN_DEG = 5.0;

        // =======================================================
        // ====================== MOTION =========================

        // ==== POSITION-MOTION MAPPED PROCEDURES ====

        public static final Map<ArmPosState, Boolean> SECOND_PIVOT_PRIORITY_MAP = Map.of(
                ArmPosState.TRANSFER, true, // move second pivot first when going to transfer
                ArmPosState.SPEAKER, true, // move second pivot first when going to speaker
                ArmPosState.AMP, false, // move first pivot first when going to amp
                ArmPosState.TRAP, false); // move first pivot first when going to trap

        /**
         * @param state state to check second pivot's priority for.
         * @return whether second pivot has priority in the given state.
         */
        public static boolean getSecondPivotPriority(ArmPosState state) {
            return SECOND_PIVOT_PRIORITY_MAP.get(state);
        }

        // ==== MOTION MAGIC ====

        // ---- FIRST PIVOT ----
        public static final double FIRST_kP = 0.015;
        public static final double FIRST_kI = 0;
        public static final double FIRST_kD = 0;
        public static final double FIRST_kF = 0.5;
        public static final double FIRST_kS = 0;
        public static final double FIRST_kV = 0;
        public static final double FIRST_kA = 0;

        public static final double FIRST_MAX_VELOCITY = 20; // maximum achievable velocity (deg per sec)
        public static final double FIRST_TARGET_CRUISE_VEL = FIRST_MAX_VELOCITY * 0.5; // target cruise velocity
        public static final double FIRST_MAX_ACCEL = 20; // target acceleration (deg / sec / sec)
        public static final double FIRST_TARGET_JERK = 0.1; // target jerk (deg / sec / sec / sec)

        // ---- SECOND PIVOT ----
        public static final double SECOND_kP = 0.01;
        public static final double SECOND_kI = 0;
        public static final double SECOND_kD = 0;
        public static final double SECOND_kF = 0;
        public static final double SECOND_kS = 0;
        public static final double SECOND_kV = 0;
        public static final double SECOND_kA = 0;

        public static final double SECOND_MAX_VELOCITY = 30; // maximum achievable velocity (deg per sec)
        public static final double SECOND_TARGET_CRUISE_VEL = FIRST_MAX_VELOCITY * 0.5; // target cruise velocity
        public static final double SECOND_MAX_ACCEL = 10; // target acceleration (deg / sec / sec)
        public static final double SECOND_TARGET_JERK = -1; // target jerk (deg / sec / sec / sec)

        // ==== VELOCITY THRESHOLDS ====

        /**
         * Minimum motor velocity to be considered as arm motion on first pivot. Used
         * for updating Arm motion and position states.
         */
        public static final double FIRST_VEL_THRESHOLD = 1E-6;
        /**
         * Minimum motor velocity to be considered as arm motion on second pivot. Used
         * for updating Arm motion and position states.
         */
        public static final double SECOND_VEL_THRESHOLD = 1E-6;
    }

    public static class ShooterConstants {
        // ==== MOTORS ====
        public static final int SHOOTER_LEFT_ID = 11;
        public static final int SHOOTER_RIGHT_ID = 10;

        public static final int BELT_INDEX_ID = 19;

        // ---- BEAM BREAK ----
        public static final int BEAM_BREAK_CHANNEL_ID = 5;

        // ---- MAXIMUM RPM ----
        public static final double SHOOTER_MAX_RPM = 87;
        public static final double INDEX_MAX_RPM = 11000;

        public static final double RPM_ERROR_MARGIN = 10;
    }

    public static class IntakeConstants {
        // ==== MOTORS - NEO 1.1s ====
        public static final int SPARKMAX_ID = 18; // TODO: This needs to be double checked.

        // ==== SOLENOID ====
        public static final int SOLENOID_PORT = 1;
    }

    public static class ClimbConstants {
        // ==== MOTORS - FALCON 500s ====
        public static final int RIGHT_CLIMB_ID = 16; // TODO: This needs to be double checked.
        public static final int LEFT_CLIMB_ID = 17; // TODO: This needs to be double checked.

        // ==== ENCODERS - CANCoders ====
        public static final int RIGHT_CLIMB_ENCODER_ID = 24; // TODO: This needs to be double checked.
        public static final int LEFT_CLIMB_ENCODER_ID = 25; // TODO: This needs to be double checked.

        // ==== RATIOS ====
        public static final double RATIO_WINCH = 20.25;
    }
}
