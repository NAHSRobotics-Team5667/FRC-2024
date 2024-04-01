// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Automatically drives to a note in view. ALWAYS call in parallel with intake.
 * Ends when interrupted.
 */
public class IntakeDriveAuto extends Command {
    private SwerveSubsystem swerve;
    private LimelightSubsystem limelight;

    private ProfiledPIDController pid_x, pid_y;

    /**
     * Creates a new IntakeDrive. Automatically drives to a note in view.ALWAYS call
     * in parallel while intake is running.
     */
    public IntakeDriveAuto() {
        swerve = SwerveSubsystem.getInstance();
        limelight = LimelightSubsystem.getInstance(); // DO NOT add to addRequirements()

        pid_x = new ProfiledPIDController(
                DriveConstants.INTAKE_P,
                DriveConstants.INTAKE_I,
                DriveConstants.INTAKE_D, new TrapezoidProfile.Constraints(180, 260));

        pid_y = new ProfiledPIDController(
                DriveConstants.INTAKE_P,
                DriveConstants.INTAKE_I,
                DriveConstants.INTAKE_D, new TrapezoidProfile.Constraints(180, 260));

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
        double x = 0, y = 0;
        if (limelight.getNoteTy() != 0.0) {
            x = MathUtil.clamp(-pid_x.calculate(limelight.getNoteTy(), -30), -1, 1)
                    * swerve.getMaximumVelocity();
            y = MathUtil.clamp(pid_y.calculate(limelight.getNoteTx(), 0), -1, 1) * swerve.getMaximumVelocity();
        }

        swerve.driveRobotOriented(new Translation2d(x, y), 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerve.driveRobotOriented(new Translation2d(), 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
