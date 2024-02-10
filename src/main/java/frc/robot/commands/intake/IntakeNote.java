package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.IntakeSubsystem;

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
public class IntakeNote extends Command {

    public IntakeSubsystem intake;

    // * Creates a new intake */
    public IntakeNote() {
        intake = IntakeSubsystem.getInstance();
        // addRequirement() - prevent two commands from being run at the same time
        addRequirements(intake);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {
        if (intake.isIntakeDeployed() == true) { // the piston is already extended - want to retract and reset the motor
            intake.setPiston(false);
            intake.setIntakeSpeed(0);
        }
    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        if (intake.isIntakeDeployed() == false) {

            intake.setIntakeSpeed(1);

            intake.setPiston(true);
        }

    }

    // Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0);
        intake.setPiston(false);
    }

    // Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return false;
    }
}