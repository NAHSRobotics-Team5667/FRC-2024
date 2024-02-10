package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This command set the speed of the motor on the intake to 100% and shoot out
 * the note.
 * 
 */
public class ShootNote extends Command {

    public ShooterSubsystem shooter;

    // * Creates a new Shooter. */
    public ShootNote() {
        shooter = ShooterSubsystem.getInstance();

        // addRequirement() - prevent two commands from being run at the same time.
        addRequirements(shooter);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {

    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {

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
