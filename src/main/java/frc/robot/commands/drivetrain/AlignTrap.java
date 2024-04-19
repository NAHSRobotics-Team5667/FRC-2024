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

public class AlignTrap extends Command {
    private SwerveSubsystem swerve;
    private LimelightSubsystem limelight;

    private PIDController alignPID;
    private ProfiledPIDController pid_x, pid_y;

    private double tx_target, ty_target;

    /** Creates a new AlignTrap. */
    public AlignTrap() {
        swerve = SwerveSubsystem.getInstance();
        limelight = LimelightSubsystem.getInstance();

        tx_target = -19.45;
        ty_target = 19.11;

        alignPID = new PIDController(
                DriveConstants.ALIGN_P,
                DriveConstants.ALIGN_I,
                DriveConstants.ALIGN_D);

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
        swerve.driveRobotOriented(new Translation2d(), 0);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // line up to tag
        double xPercent = MathUtil.clamp(-pid_x.calculate(limelight.getTagTy(), ty_target), -1,
                1);
        double yPercent = MathUtil.clamp(-pid_y.calculate(limelight.getTagTx(), tx_target), -1,
                1);
        // angle to tag
        double angleTarget = 0;

        if (limelight.getAprilTagID() == 11 || limelight.getAprilTagID() == 15) {
            angleTarget = 122;
        } else if (limelight.getAprilTagID() == 12 || limelight.getAprilTagID() == 16) {
            angleTarget = -122;
        } else if (limelight.getAprilTagID() == 13 || limelight.getAprilTagID() == 14) {
            angleTarget = 2;
        }

        double newAngularRotationPercent = MathUtil.clamp(
                alignPID.calculate(swerve.getYaw(), angleTarget), -1, 1);

        swerve.driveRobotOriented(
                new Translation2d(xPercent * swerve.getMaximumVelocity(), yPercent * swerve.getMaximumVelocity()),
                newAngularRotationPercent * swerve.getMaximumAngularVelocity());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        swerve.driveRobotOriented(new Translation2d(), 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return limelight.getAprilTagID() == -1
                || ((limelight.getTagTx() >= tx_target - 3 && limelight.getTagTx() <= tx_target + 3)
                        && (limelight.getTagTy() >= ty_target - 3 && limelight.getTagTy() <= ty_target + 3));
    }
}
