package frc.robot.util;

public class States {
    public static enum ArmState {
        /** Transfer state between intake and shooter. */
        TRANSFER,
        /** State for scoring in speaker. */
        SPEAKER,
        /** State for scoring in amp. */
        AMP,
        /** State for scoring in trap. */
        TRAP,
        /** State for climb. */
        CLIMB,
        /** State for hanging. */
        HANGING,
        /** State that represents an undefined position. */
        INTERMEDIATE
    }

    public static enum ShooterState {
        /** Both shooter wheels are stopped. */
        STOPPED,
        /**
         * At least one shooter wheel is moving, but not ramped to the right velocity.
         */
        MOVING,
        /** Shooter is ready to fire. */
        READY
    }

    public static enum RobotState {
        IDLE,
        INTAKE,
        AMP,
        SPEAKER,
        TRAP,
        CLIMB,
        HANGING
    }
}
