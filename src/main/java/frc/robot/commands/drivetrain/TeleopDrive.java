// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.States.ArmState;
import frc.robot.util.States.RobotState;

public class TeleopDrive extends Command {
    private LimelightSubsystem limelight;
    private SwerveSubsystem swerve;
    private StateManager states;

    private PIDController alignPID;
    private ProfiledPIDController pid_x, pid_y;

    private DoubleSupplier vX, vY, vRot;

    /** Creates a new SpeakerDrive. */
    public TeleopDrive(DoubleSupplier vX, DoubleSupplier vY, DoubleSupplier vRot) {
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
        boolean autoDrive = false;

        // set default value of angular velocity based on controller
        double angularRotationPercent = Math.pow(vRot.getAsDouble(), 3);
        double xPercent = vX.getAsDouble();
        double yPercent = vY.getAsDouble();

        double newAngularRotationPercent = 0;

        // adjust rotational and translational velocity based on april tag
        if (states.getTargetArmState().equals(ArmState.SPEAKER)
                && (limelight.getAprilTagID() == 4 || limelight.getAprilTagID() == 7)) {
            // align to april tag - setpoint is 8.8 <units> away from center
            newAngularRotationPercent = MathUtil.clamp(
                    alignPID.calculate(limelight.getTagTx(), 7.8), -1, 1);

        } else if (states.getDesiredRobotState().equals(RobotState.AMP)
                && states.getTargetArmState().equals(ArmState.AMP)) {
            // make drivetrain slower to avoid jiggling arm
            xPercent = MathUtil.clamp(xPercent, -0.7, 0.7);
            yPercent = MathUtil.clamp(yPercent, -0.7, 0.7);

        } else if (states.getDesiredRobotState().equals(RobotState.INTAKE)) {
            if (limelight.getNoteTy() != 0.0) {
                // find note
                xPercent = MathUtil.clamp(-pid_x.calculate(limelight.getNoteTy(), -30), -1, 1);
                yPercent = MathUtil.clamp(pid_y.calculate(limelight.getNoteTx(), 0), -1, 1);
                // align to note
                newAngularRotationPercent = MathUtil.clamp(
                        pid_y.calculate(limelight.getNoteTx(), 0), -1, 1);

                autoDrive = true;
            } else {
                autoDrive = false;
            }

        } else if (states.getDesiredRobotState().equals(RobotState.FEED)) {
            // align to ideal angle
            newAngularRotationPercent = MathUtil.clamp(
                    alignPID.calculate(swerve.getYaw(), 15), -1, 1);

        } else if (states.getDesiredRobotState().equals(RobotState.TRAP)) {
            if (limelight.getAprilTagID() != -1) {
                // line up to tag
                // TODO: get values to line up for trap - tx and ty
                xPercent = MathUtil.clamp(-pid_x.calculate(limelight.getTagTy(), 19.11), -1, 1);
                yPercent = MathUtil.clamp(-pid_y.calculate(limelight.getTagTx(), 19.45), -1, 1);
                // angle to tag
                double angleTarget = 0;

                if (limelight.getAprilTagID() == 11 || limelight.getAprilTagID() == 15) {
                    angleTarget = 122;
                } else if (limelight.getAprilTagID() == 12 || limelight.getAprilTagID() == 16) {
                    angleTarget = 302;
                } else if (limelight.getAprilTagID() == 13 || limelight.getAprilTagID() == 14) {
                    angleTarget = 2;
                }

                newAngularRotationPercent = MathUtil.clamp(
                        alignPID.calculate(swerve.getYaw(), angleTarget), -1, 1);

                autoDrive = true;
            } else {
                autoDrive = false;
            }
        }

        // if PID does not give a heading output, give heading control back to driver
        if (newAngularRotationPercent != 0) {
            angularRotationPercent = newAngularRotationPercent;
        }

        if (!autoDrive) {
            // putting it all together
            swerve.driveTeleop(
                    new Translation2d(
                            xPercent * swerve.getMaximumVelocity(),
                            yPercent * swerve.getMaximumVelocity()),
                    angularRotationPercent * swerve.getMaximumAngularVelocity());
        } else {
            swerve.driveRobotOriented(
                    new Translation2d(
                            xPercent * swerve.getMaximumVelocity(),
                            yPercent * swerve.getMaximumVelocity()),
                    angularRotationPercent * swerve.getMaximumAngularVelocity());
        }

        SmartDashboard.putNumber("[DRIVE_CMD] Error", alignPID.getPositionError());
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
