package frc.robot.util;

public class States {
    public static enum ArmMotionState {
        IDLE,
        MOVING
    }

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
        HUMAN_PLAYER,
        /** State that represents an undefined position. */
        INTERMEDIATE
    }

    public static enum ShooterStates {
        STOPPED,
        SPEEDING_UP,
        FULL_SPEED,
        SLOWING_DOWN
    }

    public static enum IndexStates {
        EMPTY,
        FULL
    }
}
