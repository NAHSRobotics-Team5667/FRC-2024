package frc.robot.commands.climb;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj.event.BooleanEvent;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * This command should raise the climb as well as drop the climb with a single button press.
 * 
 */
public class setClimb extends Command {

    public ClimbSubsystem climb; 
    
   //* Creates a new Climb. */
    public setClimb() {
        climb = ClimbSubsystem.getInstance();

        //addRequirement() - prevent two commands from being run at the same time.
        addRequirements(climb);
    }

    //Called when command is initiated/first scheduled
    @Override
    public void initialize() {
        
    }

    //Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {

    }

    //Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {

    }

    //Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return false;
    }
}
