// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.actions.IntakeAndShootAuto;
import frc.robot.commands.actions.IntakeNote;
import frc.robot.commands.actions.RemoveNote;
import frc.robot.commands.actions.ShootNoteAmp;
import frc.robot.commands.actions.ShootNoteSpeaker;
import frc.robot.commands.arm.SetArm;
import frc.robot.commands.drivetrain.SpeakerDrive;
import frc.robot.commands.index.IndexCommand;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.commands.shooter.ShooterCommandAuto;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TestSubsystem;
import frc.robot.util.States.RobotState;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private StateManager states = StateManager.getInstance();
    private LimelightSubsystem limelight = LimelightSubsystem.getInstance();

    private SwerveSubsystem drive = SwerveSubsystem.getInstance();
    private IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private ArmSubsystem arm = ArmSubsystem.getInstance();
    private ShooterSubsystem shooter = ShooterSubsystem.getInstance();

    private ClimbSubsystem climb;
    private TestSubsystem testSubsystem;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private static final CommandXboxController driverXbox = new CommandXboxController(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // ========================================================
        // ==================== DRIVETRAIN ========================

        // ---- SETUP ----

        // configure dt based on JSON config files
        SwerveSubsystem.initialize(new File(Filesystem.getDeployDirectory(),
                "swerve"));
        drive = SwerveSubsystem.getInstance();

        // ---- DRIVE COMMANDS ----

        // Real drive command
        Command driveFieldOrientedAnglularVelocity = drive.driveCommand(
                () -> MathUtil.applyDeadband(-driverXbox.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND), // X direction
                // is front
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND), // Y direction
                // is left
                () -> -driverXbox.getRightX()); // right stick horizontal value

        Command speakerDrive = new SpeakerDrive(
                () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
                () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
                () -> MathUtil.applyDeadband(-driverXbox.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));

        // Simulation drive command
        Command driveFieldOrientedAnglularVelocitySim = drive.simDriveCommand(
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND), // X direction
                // is front
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND), // Y direction
                // is left
                () -> driverXbox.getRightX()); // right stick horizontal value

        // Sets default DT command to the real command when robot is IRL
        // Sets default DT command to the sim command when robot is simulated

        drive.setDefaultCommand(
                !RobotBase.isSimulation() ? speakerDrive : driveFieldOrientedAnglularVelocitySim);

        // ========================================================
        // ===================== INTAKE ===========================

        // intake = IntakeSubsystem.getInstance();

        // ========================================================
        // ====================== ARM =============================

        // arm = ArmSubsystem.getInstance();
        arm.setDefaultCommand(new SetArm());

        // ========================================================
        // ==================== SHOOTER ===========================

        // shooter = ShooterSubsystem.getInstance();

        // ========================================================
        // ====================== CLIMB ===========================

        // climb = ClimbSubsystem.getInstance();

        // ========================================================
        // ======================= TEST ===========================

        // testSubsystem = TestSubsystem.getInstance();
        // testSubsystem.setDefaultCommand(new TestCommand(50));

        // ========================================================
        // ======================== AUTO ==========================

        NamedCommands.registerCommand("IntakeAndShootAuto", new IntakeAndShootAuto());
        NamedCommands.registerCommand("ShootNoteSpeaker", new ShootNoteSpeaker());
        NamedCommands.registerCommand("IntakeNote", new IntakeNote());
        NamedCommands.registerCommand("ResetMotorsToEncoders", new InstantCommand(() -> arm.resetMotorsToEncoders()));

        // ========================================================
        // ================== CONTROLLER ==========================

        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() {

        /*
         * Driver Controller:
         * - Joystick 1 Movement - Movement on Field
         * - Joystick 2 Movement - Direction on Field (Where robot front is facing).
         * - Press X - reset gyro --> found in drivetrain subsystem periodic
         * - Press A - toggle intake
         * - Press B - toggle outtake
         * - Press RB - toggle shooter (speaker)
         * - Press LB - toggle amp
         * - Right Trigger - run index into shooter
         * - Right Stick Button - reset motor to absolute encoder --> found in arm
         * subsystem periodic
         */

        driverXbox.x().onTrue(new InstantCommand(() -> drive.resetGyro()));

        driverXbox.a().toggleOnTrue(new IntakeNote());
        driverXbox.b().toggleOnTrue(new RemoveNote());

        driverXbox.rightBumper().toggleOnTrue(new ShooterCommand(false));
        driverXbox.leftBumper().toggleOnTrue(new ShooterCommand(true));

        driverXbox.rightTrigger().whileTrue(new IndexCommand(true));

        driverXbox.rightStick().onTrue(new InstantCommand(() -> arm.resetMotorsToEncoders()));
        driverXbox.leftStick().onTrue(new InstantCommand(() -> drive.toggleFieldCentric()));

        driverXbox.povUp().onTrue(new InstantCommand(() -> states.setDesiredRobotState(RobotState.CLIMB)));
        driverXbox.povDown().onTrue(new InstantCommand(() -> states.setDesiredRobotState(RobotState.HANGING)));
    }

    public static CommandXboxController getDriverController() {
        return driverXbox;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return drive.getAutonomousCommand("6_note", true);
    }
}
