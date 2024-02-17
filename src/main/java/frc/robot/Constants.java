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
        // ================== MECHANICAL =========================

        /** Gear ratio between first motors and first pivot. */
        public static final double FIRST_GEAR_RATIO = 100.0 * (60.0 / 44.0);
        /** Gear ratio between second motors and second pivot. */
        public static final double SECOND_GEAR_RATIO = 80.0; // 80:1 - torqued up by factor of 80

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
        public static final int FIRST_ENC_PORT_1 = -1;
        public static final int FIRST_ENC_PORT_2 = -1;
        public static final double FIRST_LEFT_OFFSET = -1;
        public static final double FIRST_RIGHT_OFFSET = -1;
        /** Rotations of first pivot for each rotation of encoder. */
        public static final double FIRST_ENC_DIST_PER_ROT = 44.0 / 60.0;

        // ---- SECOND PIVOT ----
        public static final int SECOND_ENC_PORT_1 = -1;
        public static final int SECOND_ENC_PORT_2 = -1;
        public static final double SECOND_LEFT_OFFSET = -1;
        public static final double SECOND_RIGHT_OFFSET = -1;
        /** Rotations of second pivot for each rotation of encoder. */
        public static final double SECOND_ENC_DIST_PER_ROT = 1.0;

        // =======================================================
        // ================= ARM POSITION ========================

        public static final ArmAngle RESTING_POSITION = new ArmAngle(
                -1,
                -1); // TODO: determine initial angles of arm

        // create a map of goal states and their target arm positions
        public static Map<ArmPosState, ArmAngle> GOAL_POSITIONS = Map.of(
                ArmPosState.TRANSFER, RESTING_POSITION, // arm position when at resting position
                ArmPosState.SPEAKER, new ArmAngle(), // default position for speaker
                ArmPosState.AMP, new ArmAngle(90, 75), // position for amp
                ArmPosState.TRAP, new ArmAngle(100, 100), // position for trap
                ArmPosState.HUMAN_PLAYER, new ArmAngle()); // position for human player intake

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

        // ==== POSITION ERROR MARGINS ====

        public static final double FIRST_ERR_MARGIN_DEG = 1.0;
        public static final double SECOND_ERR_MARGIN_DEG = 1.0;

        // =======================================================
        // ====================== MOTION =========================

        // ==== MOTION MAGIC ====

        // ---- FIRST PIVOT ----
        public static final double FIRST_kP = -1;
        public static final double FIRST_kI = -1;
        public static final double FIRST_kD = -1;
        public static final double FIRST_kF = -1;
        public static final double FIRST_kS = -1;
        public static final double FIRST_kV = -1;
        public static final double FIRST_kA = -1;

        public static final double FIRST_MAX_VELOCITY = -1; // maximum achievable velocity (rots per sec)
        public static final double FIRST_TARGET_CRUISE_VEL = FIRST_MAX_VELOCITY * 0.5; // target cruise velocity
        public static final double FIRST_MAX_ACCEL = -1; // target acceleration (rots / sec / sec)
        public static final double FIRST_TARGET_JERK = -1; // target jerk (rots / sec / sec / sec)

        // ---- SECOND PIVOT ----
        public static final double SECOND_kP = -1;
        public static final double SECOND_kI = -1;
        public static final double SECOND_kD = -1;
        public static final double SECOND_kF = -1;
        public static final double SECOND_kS = -1;
        public static final double SECOND_kV = -1;
        public static final double SECOND_kA = -1;

        public static final double SECOND_MAX_VELOCITY = -1; // maximum achievable velocity (rots per sec)
        public static final double SECOND_TARGET_CRUISE_VEL = FIRST_MAX_VELOCITY * 0.5; // target cruise velocity
        public static final double SECOND_MAX_ACCEL = -1; // target acceleration (rots / sec / sec)
        public static final double SECOND_TARGET_JERK = -1; // target jerk (rots / sec / sec / sec)

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
        public static final int SHOOTER_LEFT_ID = 39;
        public static final int SHOOTER_RIGHT_ID = 12; //The actual is 9.

        public static final int BELT_INDEX_ID = 10;

        // ---- BEAM BREAK ----
        public static final int BEAM_BREAK_CHANNEL_ID = 1; // Goes into 2 Seperate Digital Inputs. We need to know
                                                            // which one is used here.

        // ---- MAXIMUM RPM ----
        public static final double SHOOTER_MAX_RPM = 87;
        public static final double INDEX_MAX_RPM = 11000;

        public static final double RPM_ERROR_MARGIN = 10;
    }

    public static class IntakeConstants {
        // ==== MOTORS - NEO 1.1s ====
        public static final int SPARKMAX_ID = -1;

        // ==== SOLENOID ====
        public static final int SOLENOID_PORT = -1;
    }

    public static class ClimbConstants {
        // ==== MOTORS - FALCON 500s ====
        public static final int RIGHT_CLIMB_ID = -1;
        public static final int LEFT_CLIMB_ID = -1;

        // ==== ENCODERS - CANCoders ====
        public static final int RIGHT_CLIMB_ENCODER_ID = -1;
        public static final int LEFT_CLIMB_ENCODER_ID = -1;

        // ==== RATIOS ====
        public static final double RATIO_WINCH = 20.25;
    }
}
