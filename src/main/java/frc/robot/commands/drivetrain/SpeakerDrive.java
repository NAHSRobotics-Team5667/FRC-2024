// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.States.ArmState;

public class SpeakerDrive extends Command {
    private LimelightSubsystem limelight;
    private SwerveSubsystem swerve;
    private StateManager states;

    private PIDController alignPID;

    private DoubleSupplier vX, vY, vRot;

    /** Creates a new SpeakerDrive. */
    public SpeakerDrive(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier vRot) {
        this.vX = vX;
        this.vY = vY;
        this.vRot = vRot;

        swerve = SwerveSubsystem.getInstance();
        states = StateManager.getInstance();
        limelight = LimelightSubsystem.getInstance(); // DO NOT add to addReqs()

        alignPID = new PIDController(
                DriveConstants.ALIGN_P,
                DriveConstants.ALIGN_I,
                DriveConstants.ALIGN_D);

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(swerve);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // set default value of angular velocity based on controller
        double angularRotationVel = Math.pow(vRot.getAsDouble(), 3)
                * swerve.getMaximumAngularVelocity();
        double xPercent = vX.getAsDouble();
        double yPercent = vY.getAsDouble();

        // adjust rotational and translational velocity based on april tag
        if (states.getTargetArmState().equals(ArmState.SPEAKER)
                && (limelight.getAprilTagID() == 4 || limelight.getAprilTagID() == 7)) {

            angularRotationVel = MathUtil.clamp(
                    -alignPID.calculate(limelight.getTx(), 0), -1, 1) *
                    swerve.getMaximumAngularVelocity();

            xPercent = MathUtil.clamp(xPercent, -0.75, 0.75);
            yPercent = MathUtil.clamp(yPercent, -0.75, 0.75);
        }

        // putting it all together
        swerve.drive(
                new Translation2d(
                        Math.pow(xPercent, 3) * swerve.getMaximumVelocity(),
                        Math.pow(yPercent, 3) * swerve.getMaximumVelocity()),
                angularRotationVel,
                true);

        SmartDashboard.putNumber("[ALIGN] Error", alignPID.getPositionError());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
