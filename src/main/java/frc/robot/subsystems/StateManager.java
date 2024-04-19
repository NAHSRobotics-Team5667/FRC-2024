// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.util.ArmAngle;
import frc.robot.util.States.*;

/**
 * Handles everything relating to states of the robot across subsystems.
 */
public class StateManager extends SubsystemBase {
    // arm
    private ArmState armState, targetArmState;

    // shooter
    private ShooterState shooterState;
    private double shooterStartTime;

    // robot state
    private RobotState desiredRobotState;

    // leds
    private LEDSubsystem led;

    // maps of robot state to other states --------------------
    private Map<RobotState, ArmState> robotArmMap = Map.of(
            RobotState.IDLE, ArmState.TRANSFER,
            RobotState.IDLE_AIM, ArmState.SPEAKER,
            RobotState.INTAKE, ArmState.TRANSFER,
            RobotState.OUTTAKE, ArmState.TRANSFER,
            RobotState.AMP, ArmState.AMP,
            RobotState.SPEAKER, ArmState.SPEAKER,
            RobotState.FEED, ArmState.FEED,
            RobotState.CLIMB, ArmState.CLIMB,
            RobotState.HANGING, ArmState.HANGING,
            RobotState.TRAP, ArmState.TRAP);

    // singleton
    private static StateManager instance = null;

    // ========================================================
    // =================== CONSTRUCTOR ========================

    /** Creates a new StateSubsystem. */
    private StateManager() {
        // Robot ----------------------------------------------
        desiredRobotState = RobotState.IDLE;

        // Arm ------------------------------------------------
        armState = ArmState.TRANSFER;
        targetArmState = ArmState.TRANSFER;

        // Shooter --------------------------------------------
        shooterState = ShooterState.STOPPED;
        shooterStartTime = 0;

        // LED ------------------------------------------------
        led = LEDSubsystem.getInstance();
    }

    // ========================================================
    // ==================== SINGLETON =========================

    /**
     * @return singleton instance of the State subsystem.
     */
    public static StateManager getInstance() {
        if (instance == null) {
            instance = new StateManager();
        }

        return instance;
    }

    // ========================================================
    // ======================= ROBOT ==========================

    /**
     * Sets the robot state.
     * 
     * @param newState state to replace old state with.
     */
    public void setDesiredRobotState(RobotState newState) {
        desiredRobotState = newState;
    }

    public void updateIdleRobotState() {
        if (desiredRobotState == RobotState.IDLE && IndexSubsystem.getInstance().hasGamePiece()
                && (LimelightSubsystem.getInstance().getAprilTagID() == 4
                        || LimelightSubsystem.getInstance().getAprilTagID() == 7)) {

            desiredRobotState = RobotState.IDLE_AIM;
        } else if (desiredRobotState == RobotState.IDLE_AIM && (LimelightSubsystem.getInstance().getAprilTagID() != 4
                && LimelightSubsystem.getInstance().getAprilTagID() != 7)) {
            desiredRobotState = RobotState.IDLE;
        }
    }

    /**
     * @return current robot state.
     */
    public RobotState getDesiredRobotState() {
        return desiredRobotState;
    }

    // ========================================================
    // ======================== ARM ===========================

    /**
     * Periodically called to passively update the arm state.
     */
    private void updateArmState() {
        if (ArmSubsystem.getInstance().armAtTarget()) {
            armState = targetArmState;
        } else {
            armState = ArmState.INTERMEDIATE;
        }
    }

    /**
     * Updates the target arm state passively on a periodic loop.
     */
    private void updateTargetArmState() {
        targetArmState = robotArmMap.get(desiredRobotState);
    }

    /**
     * @return target arm state.
     */
    public ArmState getTargetArmState() {
        return targetArmState;
    }

    /**
     * @return target arm angles.
     */
    public ArmAngle getTargetArmAngle() {
        return ArmConstants.getGoalArmAngle(targetArmState);
    }

    /**
     * @return current arm state.
     */
    public ArmState getArmState() {
        return armState;
    }

    /**
     * @return whether arm is at target.
     */
    public boolean armAtTarget() {
        return armState.equals(targetArmState);
    }

    // ========================================================
    // ===================== SHOOTER ==========================

    /**
     * Periodically called to passively update the shooter state.
     */
    private void updateShooterState() {
        if (Timer.getFPGATimestamp() - shooterStartTime >= ShooterConstants.SHOOTER_RAMP_TIME && armAtTarget()
                && LimelightSubsystem.getInstance().getAprilTagID() != 1) {
            shooterState = ShooterState.READY;
        } else if (ShooterSubsystem.getInstance().getLeftShooterRPM() > 0
                || ShooterSubsystem.getInstance().getRightShooterRPM() > 0) {
            shooterState = ShooterState.MOVING;
        } else {
            shooterState = ShooterState.STOPPED;
        }
    }

    /**
     * @return shooter state.
     */
    public ShooterState getShooterState() {
        return shooterState;
    }

    /**
     * Set shooter start time so that ramp time can be measured.
     * 
     * @param time start shooter time.
     */
    public void setShooterStartTime(double time) {
        shooterStartTime = time;
    }

    /**
     * @return shooter start time.
     */
    public double getShooterStartTime() {
        return shooterStartTime;
    }

    /**
     * @return whether shooter is ready to shoot.
     */
    public boolean isShooterReady() {
        return shooterState.equals(ShooterState.READY);
    }

    // ========================================================
    // ======================= LEDS ===========================

    private void updateLights() {
        if (DriverStation.isEnabled()) {
            if (desiredRobotState.equals(RobotState.INTAKE)) {
                // robot is picking up a game piece
                led.flashingRGB(255, 100, 0, 5); // flash orange while intaking
            } else if (desiredRobotState.equals(RobotState.SPEAKER) || desiredRobotState.equals(RobotState.TRAP)) {
                if (armAtTarget() && isShooterReady()) {
                    // signify to human player to press button - shooter is ready to fire
                    led.setSolidRGB(255, 255, 255); // solid white LED when ready to shoot
                } else {
                    // ramping up to speed or arm moving to target
                    led.flashingRGB(255, 255, 255, 5); // white flashing when ramping up
                }
            } else if (desiredRobotState.equals(RobotState.OUTTAKE)) {
                led.flashingRGB(255, 0, 0, 10); // flash red when outtaking
            } else {
                if (IndexSubsystem.getInstance().hasGamePiece()) {
                    // robot is in transit with game piece
                    led.setSolidRGB(0, 255, 0); // bright green when robot has game piece
                } else {
                    // default behavior
                    led.setSolidRGB(255, 100, 0); // solid orange
                }
            }
        } else {
            // run cyclon if robot is disabled
            led.cylon(160, 255, 1);
        }
    }

    // ========================================================
    // ====================== RUMBLE ==========================

    private void updateRumble() {
        if (IndexSubsystem.getInstance().hasGamePiece() && DriverStation.isTeleopEnabled()) {
            // rumbleTimer = Timer.getFPGATimestamp();
            // start rumbling
            RobotContainer.getRumbleController().setRumble(RumbleType.kLeftRumble, 0.25);
        } else {
            // stop rumbling
            RobotContainer.getRumbleController().setRumble(RumbleType.kBothRumble, 0);
            // if (!IndexSubsystem.getInstance().hasGamePiece()) { // reset rumble timer
            // rumbleTimer = 0;
            // }
        }
    }

    // ========================================================
    // ===================== PERIODIC =========================

    @Override
    public void periodic() {
        updateTargetArmState(); // update target arm state in accordance with robot state
        updateArmState(); // update arm state periodically
        updateShooterState(); // update shooter state periodically
        updateRumble(); // update rumble state of controller
        updateLights(); // update LEDs
        updateIdleRobotState(); // updates desired robot state to only idly aim if it sees the right april tags

        SmartDashboard.putBoolean("[STATES] Arm At Target", armAtTarget());
    }
}
