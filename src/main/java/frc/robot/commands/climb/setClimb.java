package frc.robot.commands.climb;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * This command should raise the climb as well as drop the climb with a single
 * button press.
 * 
 */
public class SetClimb extends Command {

    public ClimbSubsystem climb;

    // * Creates a new Climb. */
    public SetClimb() {
        climb = ClimbSubsystem.getInstance();

        // addRequirement() - prevent two commands from being run at the same time.
        addRequirements(climb);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {

    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        double positionLeft = climb.getLeftClimbHeight();
        double positionRight = climb.getRightClimbHeight();
        int maxClimbHeight = -1;

        if (positionLeft == maxClimbHeight) {
            climb.setClimbSpeed(0.5);
        }

        if (positionRight == maxClimbHeight) {
            climb.setClimbSpeed(0.5);
        }

        if (positionLeft == 0) {
            climb.setClimbSpeed(-0.5);
        }

        if (positionRight == 0) {
            climb.setClimbSpeed(-0.5);
        }

    }

    // Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {

    }

    // Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return false;
    }
}
