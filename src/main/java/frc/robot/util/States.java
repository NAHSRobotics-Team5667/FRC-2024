package frc.robot.util;

public class States {
    public static enum ArmMotion {
        IDLE,
        MOVING_TO_POSITION
    }

    public static enum ArmState {
        RESTING,
        DEFAULT_SPEAKER,
        AMP,
        TRAP,
        HUMAN_PLAYER
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
