package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {

    private IntakeSubsystem intakeSubsystem;

    // * Creates a new intake */
    public IntakeCommand() {
        intakeSubsystem = IntakeSubsystem.getInstance();
        // addRequirement() - prevent two commands from being run at the same time
        addRequirements(intakeSubsystem);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {
        if (intakeSubsystem.isIntakeDeployed() == true) { // the piston is already extended - want to retract and reset
                                                          // the motor
            intakeSubsystem.setPiston(false);
            intakeSubsystem.setIntakeSpeed(0);
        }
    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        if (intakeSubsystem.isIntakeDeployed() == false) {
            intakeSubsystem.setIntakeSpeed(1);

            Timer.delay(2);

            intakeSubsystem.setPiston(true);
        }

    }

    // Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.setIntakeSpeed(0);
        intakeSubsystem.setPiston(false);
    }

    // Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return false;
    }
}