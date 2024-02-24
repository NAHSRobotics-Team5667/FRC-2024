// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.TestCommand;
import frc.robot.commands.intake.IntakeNote;
import frc.robot.commands.shooter.SetArm;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
// import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.TestSubsystem;
import frc.robot.util.ArmAngle;
import frc.robot.util.States.ArmPosState;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
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
    private SwerveSubsystem drive;
    private IntakeSubsystem intake;
    private ArmSubsystem arm;
    private ShooterSubsystem shooter;
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

        // // ---- DRIVE COMMANDS ----

        // Real drive command
        Command driveFieldOrientedAnglularVelocity = drive.driveCommand(
                () -> MathUtil.applyDeadband(driverXbox.getLeftY(),
                        OperatorConstants.LEFT_Y_DEADBAND), // X direction
                // is front
                () -> MathUtil.applyDeadband(driverXbox.getLeftX(),
                        OperatorConstants.LEFT_X_DEADBAND), // Y direction
                // is left
                () -> driverXbox.getRightX()); // right stick horizontal value

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
                !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedAnglularVelocitySim);

        // ========================================================
        // ===================== INTAKE ===========================

        intake = IntakeSubsystem.getInstance();

        // ========================================================
        // ====================== ARM =============================

        arm = ArmSubsystem.getInstance();
        arm.setDefaultCommand(new SetArm());

        // ========================================================
        // ==================== SHOOTER ===========================

        shooter = ShooterSubsystem.getInstance();

        // ========================================================
        // ====================== CLIMB ===========================

        // climb = ClimbSubsystem.getInstance();

        // ========================================================
        // ======================= TEST ===========================

        // testSubsystem = TestSubsystem.getInstance();
        // testSubsystem.setDefaultCommand(new TestCommand(50));

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
         * - Press A - toggle intake
         * - Press B - toggle outtake
         * - Press RB - toggle shooter (speaker)
         * - Press LB - toggle amp
         */

        driverXbox.a().toggleOnTrue(new IntakeNote(true));
        driverXbox.b().toggleOnTrue(new IntakeNote(false)); // watch for errors bc of overlapping commands
        driverXbox.rightBumper().toggleOnTrue(new ShootCommand(100, 100, false));
        driverXbox.leftBumper().toggleOnTrue(new ShootCommand(30, 30, true));
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
        //
        // An example command will be run in autonomous
        // return Autos.exampleAuto(m_exampleSubsystem);
        // return null;
        // drive.postTrajectory("Test1");
        // return drive.getAutonomousCommand("Test1", true);
        return new SetArm();
    }
}
