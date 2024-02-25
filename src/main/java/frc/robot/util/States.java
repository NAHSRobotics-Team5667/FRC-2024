package frc.robot.util;

public class States {
    // public static enum ArmMotionState {
    // IDLE,
    // MOVING
    // }

    public static enum ArmPosState {
        /** Transfer state between intake and shooter. */
        TRANSFER,
        /** State for scoring in speaker. */
        SPEAKER,
        /** State for scoring in amp. */
        AMP,
        /** State for scoring in trap. */
        TRAP,
        /** State for intaking from human player elevated station. */
        CLIMB,
        /** State that represents an undefined position. */
        INTERMEDIATE
    }

    public static enum ShooterStates {
        STOPPED,
        ADJUST_VEL,
        READY
    }

    // public static enum IndexStates {
    // EMPTY,
    // FULL
    // }
}
