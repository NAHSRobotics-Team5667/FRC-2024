package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.States.ShooterStates;

/**
 * This command set the speed of the motor on the intake to 100% and shoot out
 * the note.
 * 
 */
public class shootNote extends Command {

    public ShooterSubsystem shooter;
    public ShooterStates shooterState;
     public double[] shooterRPMs = shooter.getShooterRPM();


    // * Creates a new Shooter. */
    public shootNote() {
        shooter = ShooterSubsystem.getInstance();

        // addRequirement() - prevent two commands from being run at the same time.
        addRequirements(shooter);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {
        shooterState = ShooterStates.STOPPED;
    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        shooter.setShooterSpeed(1.00);
        
        //Checking Shooter RPMs until it reaches Max RPM to set Shooter State to FULL_SPEED
        while (shooterRPMs[0] <= 5900 && shooterRPMs[1] <= 5900) {
            shooterState = ShooterStates.SPEEDING_UP;
        }
        shooterState = ShooterStates.FULL_SPEED;
        
    }

    // Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {
        boolean hasGamePiece = shooter.hasGamePiece();        
        if (hasGamePiece == false) {
            shooter.setShooterSpeed(0.00);

            //Checking Shooter RPMs until its at 0 to set Shooter State to STOPPED
            while (shooterRPMs[0] > 0 && shooterRPMs[1] > 0) {
                shooterState = ShooterStates.SLOWING_DOWN;
        }
            shooterState = ShooterStates.STOPPED;

        }

    }

    // Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return false;
    }
}
