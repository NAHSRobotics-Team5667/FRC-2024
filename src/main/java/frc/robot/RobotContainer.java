// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.actions.FindAndIntakeNote;
import frc.robot.commands.actions.IntakeAndShootAuto;
import frc.robot.commands.actions.IntakeNote;
import frc.robot.commands.actions.IntakeNoteAuto;
import frc.robot.commands.actions.JustShoot;
import frc.robot.commands.actions.RemoveNote;
import frc.robot.commands.actions.ShootNoteSpeaker;
import frc.robot.commands.arm.SetArm;
import frc.robot.commands.drivetrain.AlignShooterAuto;
import frc.robot.commands.drivetrain.TeleopDrive;
import frc.robot.commands.index.IndexCommand;
import frc.robot.commands.shooter.ShooterCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.StateManager;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.States.RobotState;

import java.io.File;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    // initialize each subsystem here
    private SwerveSubsystem drive = SwerveSubsystem.getInstance();
    private IntakeSubsystem intake = IntakeSubsystem.getInstance();
    private ArmSubsystem arm = ArmSubsystem.getInstance();
    private ShooterSubsystem shooter = ShooterSubsystem.getInstance();
    private LEDSubsystem led = LEDSubsystem.getInstance();

    private SendableChooser<String> autoChooser = new SendableChooser<>();

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private static final CommandXboxController driverXbox = new CommandXboxController(0);
    private static final XboxController driverRumble = new XboxController(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // ---- Auto Chooser ----

        autoChooser.addOption("4_vis", "4_vis");
        autoChooser.addOption("5_full_vis", "5_full_vis");
        autoChooser.addOption("5_steal", "5_steal");
        autoChooser.addOption("farside", "farside");
        autoChooser.setDefaultOption("5_full_vis", "5_full_vis");

        SmartDashboard.putData(autoChooser);

        // ========================================================
        // ==================== DRIVETRAIN ========================

        // ---- SETUP ----

        // configure dt based on JSON config files
        SwerveSubsystem.initialize(new File(Filesystem.getDeployDirectory(),
                "swerve"));
        drive = SwerveSubsystem.getInstance();

        // ---- DRIVE COMMANDS ----

        Command speakerDrive = new TeleopDrive(
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
        // ====================== ARM =============================

        arm.setDefaultCommand(new SetArm());

        // ========================================================
        // ======================= LED ============================

        led.setSolidRGB(0, 255, 0);

        // ========================================================
        // ================== CONTROLLER ==========================

        // Configure the trigger bindings
        configureBindings();

        // ========================================================
        // ======================== AUTO ==========================

        // ---- Named Commands ----

        NamedCommands.registerCommand("IntakeAndShootAuto", new IntakeAndShootAuto());
        NamedCommands.registerCommand("ShootNoteSpeaker", new ShootNoteSpeaker());
        NamedCommands.registerCommand("JustShoot", new JustShoot());
        NamedCommands.registerCommand("IntakeNote", new IntakeNoteAuto());
        NamedCommands.registerCommand("ResetMotorsToEncoders", new InstantCommand(() -> arm.resetMotorsToEncoders()));
        NamedCommands.registerCommand("ResetGyro", new InstantCommand(() -> drive.resetGyro()));
        NamedCommands.registerCommand("ShooterCommand", new ShooterCommand(RobotState.SPEAKER));
        NamedCommands.registerCommand("CheckAprilTags", new InstantCommand(() -> drive.addVisionReading()));
        NamedCommands.registerCommand("FindAndIntakeNote", new FindAndIntakeNote());
        NamedCommands.registerCommand("AlignShooterAuto", new AlignShooterAuto());

        drive.loadAutos();
    }

    /**
     * Use this method to define your trigger->command mappings.
     */
    private void configureBindings() {

        /*
         * Driver Controller:
         * - Left Stick - Movement on Field
         * - Right Stick - Direction on Field (Where robot front is facing).
         * - Press X - reset gyro --> found in drivetrain subsystem periodic
         * - Press A - toggle intake
         * - Press B - toggle outtake
         * - Press RB - toggle shooter (speaker)
         * - Press LB - toggle amp
         * - Right Trigger - run index into shooter
         * - Right Stick Button - reset arm motors to absolute encoder --> found in arm
         * subsystem periodic
         * - Press Y - feed over stage
         * - DPad Up - line up climb
         * - DPad Down - hang
         * - DPad Left - Raise speaker arm setpoint
         * - DPad right - Lower speaker arm setpoint
         */

        driverXbox.x().onTrue(new InstantCommand(() -> drive.resetGyro()));

        driverXbox.a().toggleOnTrue(new IntakeNote());
        // driverXbox.a().toggleOnTrue(new FindAndIntakeNote());
        driverXbox.b().toggleOnTrue(new RemoveNote());

        driverXbox.rightBumper().toggleOnTrue(new ShooterCommand(RobotState.SPEAKER));
        driverXbox.leftBumper().toggleOnTrue(new ShooterCommand(RobotState.AMP));

        driverXbox.rightTrigger().whileTrue(new IndexCommand(RobotState.SPEAKER));

        driverXbox.rightStick().onTrue(new InstantCommand(() -> arm.resetMotorsToEncoders()));

        driverXbox.povUp().onTrue(new InstantCommand(() -> states.setDesiredRobotState(RobotState.CLIMB)));
        driverXbox.povDown().onTrue(new InstantCommand(() -> states.setDesiredRobotState(RobotState.HANGING)));

        driverXbox.y().toggleOnTrue(new ShooterCommand(RobotState.FEED));

        driverXbox.leftTrigger().onTrue(new InstantCommand(() -> drive.setFieldCentric(true)));
        driverXbox.leftTrigger().onFalse(new InstantCommand(() -> drive.setFieldCentric(false)));

        driverXbox.leftStick().toggleOnTrue(new ShooterCommand(RobotState.TRAP));
    }

    public static CommandXboxController getDriverController() {
        return driverXbox;
    }

    public static XboxController getRumbleController() {
        return driverRumble;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return drive.getAutonomousCommand(autoChooser.getSelected(), true);
    }
}
