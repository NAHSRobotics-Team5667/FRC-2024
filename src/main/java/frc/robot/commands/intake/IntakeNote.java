package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.States.ArmPosState;

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
    public ShooterSubsystem shooter;

    // * Creates a new intake */
    public IntakeNote() {
        intake = IntakeSubsystem.getInstance();
        shooter = ShooterSubsystem.getInstance();

        // addRequirement() - prevent two commands from being run at the same time
        addRequirements(intake, shooter);
    }

    // Called when command is initiated/first scheduled
    @Override
    public void initialize() {
        intake.setIntakeSpeed(0);

        //ArmSubsystem.getInstance().setTargetPosition(ArmPosState.TRANSFER);
    }

    // Called when scheduler runs while the command is scheduled
    @Override
    public void execute() {
        //intake.setPiston(true);
        intake.setIntakeSpeed(50);

        shooter.setIndexSpeed(30);
    }

    // Called when the command is interruped or ended
    @Override
    public void end(boolean interrupted) {
        intake.setIntakeSpeed(0);
        shooter.setIndexSpeed(0);
        //intake.setPiston(false);
    }

    // Called so it should return true when the command will end
    @Override
    public boolean isFinished() {
        return shooter.hasGamePiece(); // finishes intake command once game piece is collected
    }
}