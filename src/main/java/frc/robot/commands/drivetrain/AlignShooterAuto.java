// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * Aligns the shooter to shoot into the speaker during auto.
 */
public class AlignShooterAuto extends Command {
    private SwerveSubsystem swerve;
    private LimelightSubsystem limelight;

    private PIDController alignPID;

    /** Creates a new AlignShooterAuto. */
    public AlignShooterAuto() {
        swerve = SwerveSubsystem.getInstance();
        limelight = LimelightSubsystem.getInstance(); // DO NOT add to addRequirements()

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
        double rot = MathUtil.clamp(alignPID.calculate(limelight.getTagTx(), 7.8), -1, 1)
                * swerve.getMaximumAngularVelocity();

        swerve.driveRobotOriented(new Translation2d(), rot);

        SmartDashboard.putNumber("[ALIGN] Position Error", alignPID.getPositionError());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerve.driveRobotOriented(new Translation2d(), 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return Math.abs(alignPID.getPositionError()) < 3 || limelight.getTagTx() == 0.0;
    }
}
