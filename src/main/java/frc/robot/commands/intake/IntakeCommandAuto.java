package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.States.ArmState;
import frc.robot.util.States.RobotState;

/*
 * This command is going to trigger the intake to drop down and start spinning.
 * 
 * What the command should do is check if the button is pressed.
 * When the command is triggered, it checks if the intake has been deployed.
 * If it has, we retract it backup.
 * If it has not been deploy, we deploy it.
 * 
 * Retract: Motor speed 0, solenoid extend is false.
 * Deploy: Motor speed 1, solenoid extend is true.
 * 
 * Use the built in methods created in your subsystem to determine this.
 */
public class IntakeCommandAuto extends Command {

    private IntakeSubsystem intake;
    private StateManager states;

    private boolean goingIn;

    private double startTime;

    /**
     * Creates a new IntakeCommand.
     * 
     * @param goingIn whether a note is going in.
     */
    public IntakeCommandAuto(boolean goingIn) {
        intake = IntakeSubsystem.getInstance();
        states = StateManager.getInstance(); // DO NOT add to addRequirements()

        this.goingIn = goingIn;

        // addRequirement() - prevent two commands from being run at the same time
        addRequirements(intake);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {
        intake.set(0);
        states.setDesiredRobotState(RobotState.INTAKE);

        startTime = Timer.getFPGATimestamp();
    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        // intake.setPiston(true);
        if (states.getArmState().equals(ArmState.TRANSFER)) {
            intake.set((goingIn) ? IntakeConstants.INTAKE_SPEED : IntakeConstants.OUTTAKE_SPEED);
        } else {
            intake.set(0);
        }
    }

    // Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {
        states.setDesiredRobotState(RobotState.IDLE);
        intake.set(0);

        // reset field centric to true
        SwerveSubsystem.getInstance().setFieldCentric(true);
    }

    // Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return ((goingIn) ? IndexSubsystem.getInstance().hasGamePiece() : false)
                || Timer.getFPGATimestamp() - startTime >= 2.25; // finishes intake command once game
        // piece is collected or intake timer is complete
    }
}