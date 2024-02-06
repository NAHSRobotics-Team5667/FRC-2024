import frc.robot.commands;
import edu.wpifirst.wpilibj.Timer;

public class IntakeCommand extends CommandBase {

    public IntakeSubsystem intake;

    //* Creates a new intake */
    public setIntake() {
        intake = IntakeSubsystem.getInstance();
        //addRequirement() - prevent two commands from being run at the same time
        addRequirements(intake);
    }

    //Called when command is initiated/first scheduled
    @Override
    public void initialize() {
        if (isIntakeDeployed() == true) { //the piston is already extended - want to retract and reset the motor
            intake.setPiston(false);
            intake.setIntakeSpeed(0);
        }
    }

    //Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        if (isIntakeDeployed == false) {
            intake.setIntakeSpeed();
            delay(2);
            intake.setPiston(true);
        }
        
    }

    //Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0);
        intake.setPiston(false);
    }

    //Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return false;
    }


}