// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

        // ================= MOTOR IDS ===========================

        // ---- FRONT LEFT ----
        public static final int kFrontLeftDriveID = 0;
        public static final int kFrontLeftTurningID = 0;

        // ---- FRONT RIGHT ----
        public static final int kFrontRightDriveID = 0;
        public static final int kFrontRightTurningID = 0;

        // ---- BACK LEFT ----
        public static final int kBackLeftDriveID = 0;
        public static final int kBackLeftTurningID = 0;

        // ---- BACK RIGHT ----
        public static final int kBackRightDriveID = 0;
        public static final int kBackRightTurningID = 0;

        // ============================================================

        // The configurations for drivetrain speeds.
        public static final int kSpeedMultiplier = 5;
    }

    public static class ArmConstants {
    }

    public static class ShooterConstants {
    }

    public static class IntakeConstants {
    }

    public static class ClimbConstants {
    }
}
