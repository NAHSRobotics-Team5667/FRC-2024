// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
    /** Creates a new ExampleSubsystem. */
    private final ShuffleboardTab m_tab = Shuffleboard.getTab("Limelight");
    GenericEntry customTagHeight;
    GenericEntry customCamHeight;
    GenericEntry customCamAngle;

    private static LimelightSubsystem instance = null;

    private LinearFilter tagYFilter = LinearFilter.movingAverage(10);

    // ========================================================
    // =================== CONSTRUCTOR ========================

    private LimelightSubsystem() {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight-tag");
        table.getEntry("pipeline").setNumber(0);
        // NetworkTableEntry tx = table.getEntry("tx");
        // NetworkTableEntry ty = table.getEntry("ty");
        // NetworkTableEntry ta = table.getEntry("ta");
        System.out.println("LIMELIGHT SUBSYSTEM");
        m_tab.addNumber("AprilTag ID: ", this::getAprilTagID);
        m_tab.addNumber("Tag tx", this::getTagTx);
        m_tab.addNumber("Tag ty", this::getTagTy);
        m_tab.addNumber("Tag ta", this::getTagTa);
        m_tab.addNumber("Note tx", this::getNoteTx);
        m_tab.addNumber("Note ty", this::getNoteTy);
        m_tab.addNumber("Note ta", this::getNoteTa);

        // m_tab.addNumber("CameraPose X", this::getCameraPoseX);
        // m_tab.addNumber("CameraPose Y", this::getCameraPoseY);
        // m_tab.addNumber("CameraPose Z", this::getCameraPoseZ);

        // m_tab.addNumber("CameraPose Roll", this::getCameraPoseRoll);
        // m_tab.addNumber("CameraPose Pitch", this::getCameraPosePitch);
        // m_tab.addNumber("CameraPose Yaw", this::getCameraPoseYaw);

        // Gets input from shuffleboard for the height an april tag for testing purposes
        customTagHeight = m_tab.add("Custom April Tag Height", 0).getEntry();

        // Gets input from shuffleboard for the height of the camera for testing
        // purposes
        customCamHeight = m_tab.add("Custom Limelight Height", 0).getEntry();

        // Gets input from shuffleboard for the angle of the camera for testing purposes
        customCamAngle = m_tab.add("Custom Limelight Angle", 0).getEntry();

        m_tab.addNumber("April Tag Distance", this::getDistance);
    }

    // ========================================================
    // ===================== SINGLETON ========================

    /**
     * @return singleton instance of limelight.
     */
    public static LimelightSubsystem getInstance() {
        if (instance == null) {
            instance = new LimelightSubsystem();
        }

        return instance;
    }

    // ========================================================
    // =================== APRIL TAG DATA =====================

    public double getTagHeight() { // returns height of the viewed april tag in inches based off of their expected
                                   // game field height
        double customHeight = customTagHeight.getDouble(0);
        if (customHeight != 0)
            return customHeight;

        double id = getAprilTagID();

        if (id == 1.0 || id == 2.0 || id == 5.0 || id == 6.0 || id == 9.0 || id == 10.0) {
            return 53.38; // height of source and amp april tags
        } else if (id == 3.0 || id == 4.0 || id == 7.0 || id == 8.0) {
            return 57.13;
        } else {
            return 52.00; // remaining tags are for stage april tags
        }
    }

    public double getDistance() {
        double camHeight = (customCamHeight.getDouble(0) == 0) ? LimelightConstants.kCamHeight
                : customCamHeight.getDouble(0);
        double camAngle = (customCamAngle.getDouble(0) == 0) ? LimelightConstants.kCamAngle
                : customCamAngle.getDouble(0);
        double d = (getTagHeight() - camHeight) / Math.tan(Math.toRadians(getTagTy() + camAngle));
        return d;
    }

    public double getAprilTagID() {
        NetworkTableEntry tID = NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("tid");
        double AprilTagID = tID.getDouble(0.0);
        return AprilTagID;
    }

    public DoubleSupplier getAprilTagIDSupplier() {
        DoubleSupplier AprilTagID = this::getAprilTagID;
        return AprilTagID;
    }

    public double getTagTx() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("tx").getDouble(0.0);
        return tx;
    }

    public double getTagTy() {
        double ty = NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("ty").getDouble(-7.4);
        return ty;
    }

    public double getFilteredTagTy() {
        return tagYFilter.calculate(getTagTy());
    }

    public double getTagTa() {
        double ta = NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("ta").getDouble(0.0);
        return ta;
    }

    /**
     * Sets April Tag to read tx, ty data from. Does not affect localization.
     * 
     * @param priorityID april tag ID to focus on. {RED} speaker: 4 {BLUE} speaker:
     *                   7
     */
    public void setPriorityTag(int priorityID) {
        NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("priorityid").setValue(priorityID);
    }

    // ========================================================
    // =================== NOTE DETECTION =====================

    public double getNoteTx() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight-note").getEntry("tx").getDouble(0.0);
        return tx;
    }

    public double getNoteTy() {
        double ty = NetworkTableInstance.getDefault().getTable("limelight-note").getEntry("ty").getDouble(-40.0);
        return ty;
    }

    public double getNoteTa() {
        double ta = NetworkTableInstance.getDefault().getTable("limelight-note").getEntry("ta").getDouble(0.0);
        return ta;
    }

    private double predictXDist() {
        return -Units.inchesToMeters(0.844 * getNoteTx() + 0.59);
    }

    private double predictYDist() {
        return Units.inchesToMeters(32.2 - (1.95 * getNoteTy()) + (0.208 * Math.pow(getNoteTy(), 2)));
    }

    private double getDistanceToNote() {
        return Math.sqrt(Math.pow(predictXDist(), 2) + Math.pow(predictYDist(), 2));
    }

    private double getTotalTheta(double gyro) {
        return Units.degreesToRadians(gyro) + Math.atan2(predictYDist(), predictXDist());
    }

    public Pose2d getRobotPoseFromNote(double note_x, double note_y, double gyro) {
        double x_diff = getDistanceToNote() * Math.sin(getTotalTheta(gyro));
        double y_diff = getDistanceToNote() * Math.cos(getTotalTheta(gyro));

        double robot_x = note_x - x_diff;
        double robot_y = note_y - y_diff;

        return new Pose2d(robot_x, robot_y, Rotation2d.fromDegrees(gyro));
    }

    // ========================================================
    // ================ ROBOT LOCALIZATION ====================

    public Pose2d getBotPose() {
        double[] botpose = NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("botpose_wpiblue")
                .getDoubleArray(new double[11]);
        return new Pose2d(
                new Translation2d(
                        botpose[0], // botpose translation X
                        botpose[1]), // botpose translation Y
                Rotation2d.fromDegrees(botpose[5])); // botpose yaw
    }

    public double[] getBotPoseArray() {
        double[] botpose = NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("botpose_wpiblue")
                .getDoubleArray(new double[11]);
        return botpose;
    }

    public double getMegaTagLatency() {
        return getBotPoseArray()[6];
    }

    // ========================================================
    // ================ APRIL TAG POSITION ====================

    public double getAprilTagPoseX() {
        double[] tag3DPose = new double[6];
        return NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("targetpose_cameraspace")
                .getDoubleArray(tag3DPose)[0];
    }

    public double getAprilTagPoseY() {
        double[] tag3DPose = new double[6];
        return NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("targetpose_cameraspace")
                .getDoubleArray(tag3DPose)[1];
    }

    public double getAprilTagPoseZ() {
        double[] tag3DPose = new double[6];
        return NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("targetpose_cameraspace")
                .getDoubleArray(tag3DPose)[2];
    }

    public double getAprilTagPoseYaw() {
        double[] tag3DPose = new double[6];
        return NetworkTableInstance.getDefault().getTable("limelight-tag").getEntry("targetpose_cameraspace")
                .getDoubleArray(tag3DPose)[5];
    }

    // ========================================================
    // ===================== PERIODIC =========================

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("[LL] April Tag", getAprilTagID());

        SmartDashboard.putNumber("[LL] Tag Tx", getTagTx());
        SmartDashboard.putNumber("[LL] Tag Ty", getTagTy());

        tagYFilter.calculate(getTagTy());
    }
}