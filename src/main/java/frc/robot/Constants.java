// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
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

        public static final double AUTO_DRIVE_P = 10;
        public static final double AUTO_DRIVE_I = 0;
        public static final double AUTO_DRIVE_D = 0;
        public static final double AUTO_DRIVE_F = 0;

        public static final double ALIGN_P = 0.013;
        public static final double ALIGN_I = 0;
        public static final double ALIGN_D = 0.0;

        public static final double INTAKE_P = 0.01;
        public static final double INTAKE_I = 0;
        public static final double INTAKE_D = 0.0;

        // =======================================================
        // ======================= GENERAL =======================

        public static final double MAX_VELOCITY_FEET = 16.5;
        public static final double MAX_VELOCITY_METERS = 6;

        public static final double MAX_ACCELERATION_METERS = 5.0;
    }

    public static class ArmConstants {
        // =======================================================
        // ================== MECHANICAL =========================

        /** Gear ratio between first motors and first pivot. */
        public static final double FIRST_GEAR_RATIO = 125.0 * (60.0 / 44.0); // 125*(60/44):1
        /** Gear ratio between second motors and second pivot. */
        public static final double SECOND_GEAR_RATIO = 80.0; // 80:1 - torqued up by factor of 80

        // =======================================================
        // ================= MOTOR IDS ===========================

        // ---- FIRST PIVOT ----
        public static final int FIRST_PIVOT_LEAD_ID = 13;
        public static final int FIRST_PIVOT_FOLLOWER_ID = 12;

        // ---- SECOND PIVOT ----
        public static final int SECOND_PIVOT_LEAD_ID = 15;
        public static final int SECOND_PIVOT_FOLLOWER_ID = 34;

        // =======================================================
        // ===================== ENCODERS ========================

        // ---- FIRST PIVOT ----
        public static final int FIRST_ENC_PORT_1 = 7; // left side
        public static final int FIRST_ENC_PORT_2 = 8; // right side
        public static final double FIRST_LEFT_OFFSET = -0.198 - (0.25 * (60.0 / 44.0));
        public static final double FIRST_RIGHT_OFFSET = 0.676 - (0.25 * (60.0 / 44.0));
        /** Rotations of first pivot for each rotation of encoder. */
        public static final double FIRST_ENC_DIST_PER_ROT = (44.0 / 60.0) * 360.0;

        public static final double LIMIT_SWITCH = 0;

        // ---- SECOND PIVOT ----
        public static final int SECOND_ENC_PORT_1 = 6; // left side
        public static final int SECOND_ENC_PORT_2 = 9; // right side
        public static final double SECOND_LEFT_OFFSET = -0.325;
        public static final double SECOND_RIGHT_OFFSET = 0.780;
        /** Rotations of second pivot for each rotation of encoder. */
        public static final double SECOND_ENC_DIST_PER_ROT = 1.0 * 360.0;

        // =======================================================
        // ================= ARM POSITION ========================

        // create a map of goal states and their target arm positions
        public static Map<ArmState, ArmAngle> GOAL_POSITIONS = Map.of(
                ArmState.TRANSFER, new ArmAngle(/* 108.1 */ 15.2, -6.25), // arm position when at resting position
                ArmState.SPEAKER, new ArmAngle(/* 108.1 */ 55, 0), // default position for speaker
                ArmState.AMP, new ArmAngle(108.1, -129.7), // position for amp
                ArmState.FEED, new ArmAngle(45, 0), // position for feed
                ArmState.TRAP, new ArmAngle(44, -11.15), // position for trap
                ArmState.CLIMB, new ArmAngle(69.303, -109.58),
                ArmState.HANGING, new ArmAngle(15.2, 17.8)); // position for human player intake

        // create a map of arm positions and their target goal states - maps aren't
        // bi-directional :(
        public static Map<ArmAngle, ArmState> ARM_STATES = Map.of(
                getGoalArmAngle(ArmState.TRANSFER), ArmState.TRANSFER, // arm position when at resting position
                getGoalArmAngle(ArmState.SPEAKER), ArmState.SPEAKER, // default position for speaker
                getGoalArmAngle(ArmState.AMP), ArmState.AMP, // position for amp
                getGoalArmAngle(ArmState.FEED), ArmState.FEED, // position for feed
                getGoalArmAngle(ArmState.TRAP), ArmState.TRAP, // position for trap
                getGoalArmAngle(ArmState.CLIMB), ArmState.CLIMB,
                getGoalArmAngle(ArmState.HANGING), ArmState.HANGING); // position for human player
        // intake

        /**
         * Returns the goal position for a given target.
         * 
         * @param key target position.
         * @return goal position associated with target.
         */
        public static ArmAngle getGoalArmAngle(ArmState key) {
            return GOAL_POSITIONS.get(key);
        }

        /**
         * @param ty limelight crosshair vertical angle from target.
         * @return ideal first pivot angle to aim into speaker.
         */
        public static double calculateSpeakerFirstPivot(double ty) {
            // 36.8
            double output = 37.2 + (0.945 * ty) + -(8.84E-03 * Math.pow(ty, 2));

            if (ty == 0.0 && DriverStation.isAutonomousEnabled()) {
                output = 50;
            }

            // double output = -31.4 - (12.2 * ty) - (0.548 * Math.pow(ty, 2)); 4.12//
            // equation
            // for farther shots

            // continuous passthrough below

            // if (ty > -7.8 /* && DriverStation.isAutonomousEnabled() */) {
            // output = 28.4 + (0.546 * ty) + (0.0121 * Math.pow(ty, 2)); // equation for
            // normal shots with transfer
            // }
            return output;
        }

        /**
         * @param firstPivotDegrees first pivot setpoint in degrees.
         * @param ty                limelight ty from april tag.
         * @return second pivot setpoint. Currently set to 0 bc passthrough not working
         *         well enough.
         */
        public static double calculateSpeakerSecondPivot(double firstPivotDegrees, double ty) {
            double transferX = 4.67;
            double transferY = -2.51;

            double c = Math.sqrt(Math.pow(transferX, 2) + Math.pow(transferY, 2));

            double a = 20.6; // first pivot is 20.6in
            double b = Math.sqrt(
                    Math.pow((a * Math.cos(Units.degreesToRadians(firstPivotDegrees))) - transferX, 2)
                            + Math.pow((a * Math.sin(Units.degreesToRadians(firstPivotDegrees))) - transferY,
                                    2));

            double output = 0; // second pivot goes up when making farther shots

            // continuous passthrough below

            // if (ty > -7.5 /* && DriverStation.isAutonomousEnabled() */) {
            // // second pivot meets intake to shoot
            // output = -Units
            // .radiansToDegrees(Math.acos((Math.pow(c, 2) - Math.pow(a, 2) - Math.pow(b,
            // 2)) / (-2 * a * b)));
            // }

            return output;
        }

        /**
         * Updates first pivot value for speaker. Call to adjust the arm angle to auto
         * aim.
         * 
         * @param target new first pivot goal for speaker.
         */
        public static void setFirstPivotSpeakerSetpoint(double target) {
            getGoalArmAngle(ArmState.SPEAKER).setFirstPivot(target);
        }

        /**
         * Updates second pivot value for speaker. Call to adjust the arm angle to auto
         * aim.
         * 
         * @param target new second pivot goal for speaker.
         */
        public static void setSecondPivotSpeakerSetpoint(double target) {
            getGoalArmAngle(ArmState.SPEAKER).setSecondPivot(target);
        }

        /**
         * Updates first pivot goal for climb.
         * 
         * @param target new first pivot goal for climb.
         */
        public static void setFirstPivotHanging(double target) {
            getGoalArmAngle(ArmState.HANGING).setFirstPivot(target);
        }

        /**
         * Updates second pivot goal for climb.
         * 
         * @param target new second pivot goal for climb.
         */
        public static void setSecondPivotHanging(double target) {
            getGoalArmAngle(ArmState.HANGING).setSecondPivot(target);
        }

        /**
         * Returns the goal position for a given target.
         * 
         * @param key arm angles that you want the linked state of.
         * @return linked state to arm angles.
         */
        public static ArmState getState(ArmAngle key) {
            return ARM_STATES.get(key);
        }

        // ==== POSITION ERROR MARGINS ====

        /**
         * Acceptable difference between motor encoder measurement and absolute encoder
         * for first pivot (deg).
         */
        public static final double ACC_FIRST_PIVOT_DIFF = 1.0;
        /**
         * Acceptable difference between motor encoder measurement and absolute encoder
         * for first pivot (deg).
         */
        public static final double ACC_SECOND_PIVOT_DIFF = 3.0;

        /** Acceptable margin of error in PID control for first pivot (deg). */
        public static final double FIRST_ERR_MARGIN_DEG = 2.0;
        /** Acceptable margin of error in PID control for first pivot (deg). */
        public static final double SECOND_ERR_MARGIN_DEG = 1.5;

        // =======================================================
        // ====================== MOTION =========================

        // ==== POSITION-MOTION MAPPED PROCEDURES ====

        public static final Map<ArmState, Boolean> SECOND_PIVOT_PRIORITY_MAP = Map.of(
                ArmState.TRANSFER, false, // move second pivot first when going to transfer
                ArmState.SPEAKER, false, // move second pivot first when going to speaker
                ArmState.AMP, false, // move first pivot first when going to amp
                ArmState.TRAP, false,
                ArmState.CLIMB, false,
                ArmState.HANGING, true,
                ArmState.FEED, false);

        /**
         * @param state state to check second pivot's priority for.
         * @return whether second pivot has priority in the given state.
         */
        public static boolean getSecondPivotPriority(ArmState state) {
            return SECOND_PIVOT_PRIORITY_MAP.get(state);
        }

        // ==== MOTION MAGIC ====

        // ---- FIRST PIVOT ----
        public static final double FIRST_kP = 0.024;
        public static final double FIRST_kI = 0;
        public static final double FIRST_kD = 0.0;
        public static final double FIRST_kF = 0.5;
        public static final double FIRST_kS = 0;
        public static final double FIRST_kV = 0;
        public static final double FIRST_kA = 0;

        public static final double AIM_kP = 0.020;
        public static final double AIM_kI = 0.0;
        public static final double AIM_kD = 0.00;

        public static final double FIRST_MAX_VELOCITY = 600; // maximum achievable velocity (deg per sec)
        public static final double FIRST_TARGET_CRUISE_VEL = FIRST_MAX_VELOCITY * 0.5; // target cruise velocity
        public static final double FIRST_MAX_ACCEL = 1000; // target acceleration (deg / sec / sec)

        // ---- SECOND PIVOT ----
        public static final double SECOND_kP = 0.021;
        public static final double SECOND_kI = 0;
        public static final double SECOND_kD = 0;
        public static final double SECOND_kF = 0;
        public static final double SECOND_kS = 0;
        public static final double SECOND_kV = 0;
        public static final double SECOND_kA = 0;

        public static final double SECOND_MAX_VELOCITY = 300; // maximum achievable velocity (deg per sec)
        public static final double SECOND_TARGET_CRUISE_VEL = FIRST_MAX_VELOCITY * 0.5; // target cruise velocity
        public static final double SECOND_MAX_ACCEL = 500; // target acceleration (deg / sec / sec)

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
        // ==== ELECTRONICS ====
        public static final int SHOOTER_TOP_ID = 11;
        public static final int SHOOTER_BOTTOM_ID = 10;
        public static final int FAN_ID = 20;

        // ---- MAXIMUM RPM ----
        public static final double SHOOTER_MAX_RPM = 87;
        public static final double RPM_ERROR_MARGIN = 10;
        public static final double SHOOTER_RAMP_TIME = 0.25;

        // --- SPEEDS ----
        public static final double IDLE_SPEED = 20;

        public static final double AMP_SPEED = 70;

        public static final double SPEAKER_BOTTOM_SPEED = 100;
        public static final double SPEAKER_TOP_SPEED = 100;

        public static final double SPEAKER_DEFAULT_SPEED = 69;

        public static final double OUTTAKE_SPEED = 20;

        public static final double FEED_SPEED = 65;

        public static final double TRAP_SPEED = 37;
        public static final double TRAP_FAN_SPEED = 100;

        public static double shooterOffset = 0;

        public static double getSpeakerShooterSpeed(double ty) {
            // double output = Math.max(SPEAKER_DEFAULT_SPEED, 68.9 + (-0.87 * ty) + (0.0388
            // * Math.pow(ty, 2)));

            // double output = -(0.327 * ty) + 95.3;
            double output = -0.654 * ty + 90.4;
            return output;

            // TODO: for shooter tuning
            // return 100;
        }
    }

    public static class IndexConstants {
        // ==== ELECTRONICS ====
        public static final int BELT_INDEX_ID = 19;
        public static final int BEAM_BREAK_CHANNEL_ID = 5;

        // ==== SPEEDS ====
        public static final double INTAKE_SPEED = 37;
        public static final double SHOOT_SPEED = 100;
    }

    public static class IntakeConstants {
        // ==== MOTORS - NEO 1.1s ====
        public static final int SPARKMAX_ID = 18;

        // ==== SOLENOID ====
        public static final int SOLENOID_PORT = 1;

        // ==== SPEED ====
        public static final double INTAKE_SPEED = 100;
        public static final double OUTTAKE_SPEED = -100;
    }

    public static class ClimbConstants {
        // ==== MOTORS - FALCON 500s ====
        public static final int RIGHT_CLIMB_ID = 16;
        public static final int LEFT_CLIMB_ID = 17;

        // ==== ENCODERS - CANCoders ====
        public static final int RIGHT_CLIMB_ENCODER_ID = 24;
        public static final int LEFT_CLIMB_ENCODER_ID = 25;

        // ==== RATIOS ====
        public static final double RATIO_WINCH = 20.25;
    }

    public static class LimelightConstants {
        public static final double kCamHeight = 0; // Height of the limelight from the ground
        public static final double kCamAngle = 0; // Pitch angle of direction the limelight is pointed in
    }

    public static class LEDConstants {
        public static final int LED_PORT = 0;
        public static final int LED_LENGTH = 38; // number of LEDs in the sequence
    }
}
