package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.StateManager;
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
public class IntakeCommandAuto extends Command { // TODO: make outtake functional

    private IntakeSubsystem intake;
    private StateManager states;

    private boolean goingIn;

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
        states.setDesiredRobotState(RobotState.INTAKE);
        intake.set(0);
    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        // intake.setPiston(true);
        intake.set((goingIn) ? IntakeConstants.INTAKE_SPEED : IntakeConstants.OUTTAKE_SPEED);
    }

    // Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {
        states.setDesiredRobotState(RobotState.IDLE);
        intake.set(0);
        // intake.setPiston(false);
    }

    // Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return false; // finishes intake command once game
                      // piece is collected or interrupted
                      // if going the other way
    }
}