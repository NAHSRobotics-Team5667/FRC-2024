// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.shooter.SetArm;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.States.ArmPosState;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
    private SwerveSubsystem drive;
    private IntakeSubsystem intake;
    private ArmSubsystem arm;
    private ShooterSubsystem shooter;
    private ClimbSubsystem climb;

    // Replace with CommandPS4Controller or CommandJoystick if needed
    private final CommandXboxController driverXbox = new CommandXboxController(0);
    private final CommandXboxController secondaryXbox = new CommandXboxController(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // ========================================================
        // ==================== DRIVETRAIN ========================

        // ---- SETUP ----

        // configure dt based on JSON config files
        SwerveSubsystem.initialize(new File(Filesystem.getDeployDirectory(), "swerve"));
        drive = SwerveSubsystem.getInstance();

        drive.setupPathPlanner();

        // ---- DRIVE COMMANDS ----

        // Real drive command
        Command driveFieldOrientedAnglularVelocity = drive.driveCommand(
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), // X direction
                                                                                                        // is front
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), // Y direction
                                                                                                        // is left
                () -> driverXbox.getRightX()); // right stick horizontal value

        // Simulation drive command
        Command driveFieldOrientedAnglularVelocitySim = drive.simDriveCommand(
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND), // X direction
                                                                                                        // is front
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.LEFT_X_DEADBAND), // Y direction
                                                                                                        // is left
                () -> driverXbox.getRightX()); // right stick horizontal value

        // Sets default DT command to the real command when robot is IRL
        // Sets default DT command to the sim command when robot is simulated
        drive.setDefaultCommand(
                RobotBase.isReal() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocitySim);

        // ========================================================
        // ===================== INTAKE ===========================

        // intake = IntakeSubsystem.getInstance();

        // ========================================================
        // ====================== ARM =============================

        // arm = ArmSubsystem.getInstance();
        // arm.setDefaultCommand(new SetArm());

        // ========================================================
        // ==================== SHOOTER ===========================

        // shooter = ShooterSubsystem.getInstance();
        // shooter.setDefaultCommand(new ShootCommand());

        // ========================================================
        // ====================== CLIMB ===========================

        // climb = ClimbSubsystem.getInstance();

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
         * -
         */

        // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
        // new Trigger(m_exampleSubsystem::exampleCondition)
        // .onTrue(new ExampleCommand(m_exampleSubsystem));

        // Schedule `exampleMethodCommand` when the Xbox controller's B button is
        // pressed,
        // cancelling on release.
        // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        // return Autos.exampleAuto(m_exampleSubsystem);
        // return null;
        drive.postTrajectory("Test1");
        return drive.getAutonomousCommand("Test1", true);
    }
}
