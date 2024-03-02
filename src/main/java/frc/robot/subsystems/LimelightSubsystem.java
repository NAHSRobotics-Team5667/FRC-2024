// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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

    // ========================================================
    // =================== CONSTRUCTOR ========================

    private LimelightSubsystem() {

        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("pipeline").setNumber(2);
        // NetworkTableEntry tx = table.getEntry("tx");
        // NetworkTableEntry ty = table.getEntry("ty");
        // NetworkTableEntry ta = table.getEntry("ta");
        System.out.println("LIMELIGHT SUBSYSTEM");
        m_tab.addNumber("AprilTag ID: ", this::getAprilTagID);
        m_tab.addNumber("Tag tx", this::getTx);
        m_tab.addNumber("Tag ty", this::getTy);
        m_tab.addNumber("Tag ta", this::getTa);

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
        double d = (getTagHeight() - camHeight) / Math.tan(Math.toRadians(getTy() + camAngle));
        return d;
    }

    public double getAprilTagID() {
        NetworkTableEntry tID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid");
        double AprilTagID = tID.getDouble(0.0);
        return AprilTagID;
    }

    public DoubleSupplier getAprilTagIDSupplier() {
        DoubleSupplier AprilTagID = this::getAprilTagID;
        return AprilTagID;
    }

    public double getTx() {
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        return tx;
    }

    public double getTy() {
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        return ty;
    }

    public double getTa() {
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
        return ta;
    }

    // ========================================================
    // ================== CAMERA POSITION =====================

    // public double[] getCameraPose() {
    // double[] camera3DPose = new double[6];
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace")
    // .getDoubleArray(camera3DPose);
    // return camera3DPose;
    // // I beleive Translation (X,Y,Z) Rotation(Roll,Pitch,Yaw)
    // }

    // public double getCameraPoseX() {// Axis side to side on limelight
    // double[] camera3DPose = getCameraPose();
    // // return camera3DPose[0];
    // return
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace")
    // .getDoubleArray(camera3DPose)[0];

    // }

    // public double getCameraPoseY() {// Axis up and down on limelight
    // double[] camera3DPose = getCameraPose();
    // return
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace")
    // .getDoubleArray(camera3DPose)[1];
    // }

    // public double getCameraPoseZ() {// Axis comming out perpendicular to front of
    // limelight
    // double[] camera3DPose = getCameraPose();
    // return
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace")
    // .getDoubleArray(camera3DPose)[2];
    // }

    // public double getCameraPoseRoll() {
    // double[] camera3DPose = getCameraPose();
    // return
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace")
    // .getDoubleArray(camera3DPose)[3];
    // }

    // public double getCameraPosePitch() {
    // double[] camera3DPose = getCameraPose();
    // return
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace")
    // .getDoubleArray(camera3DPose)[4];
    // }

    // public double getCameraPoseYaw() {
    // double[] camera3DPose = getCameraPose();
    // return
    // NetworkTableInstance.getDefault().getTable("limelight").getEntry("camerapose_targetspace")
    // .getDoubleArray(camera3DPose)[5];
    // }

    // ========================================================
    // ================ APRIL TAG POSITION ====================

    public double getAprilTagPoseX() {
        double[] tag3DPose = new double[6];
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace")
                .getDoubleArray(tag3DPose)[0];
    }

    public double getAprilTagPoseY() {
        double[] tag3DPose = new double[6];
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace")
                .getDoubleArray(tag3DPose)[1];
    }

    public double getAprilTagPoseZ() {
        double[] tag3DPose = new double[6];
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace")
                .getDoubleArray(tag3DPose)[2];
    }

    public double getAprilTagPoseYaw() {
        double[] tag3DPose = new double[6];
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("targetpose_cameraspace")
                .getDoubleArray(tag3DPose)[5];
    }

    // ========================================================
    // ================== AUTO ALIGNMENT ======================

    public double getAutoAimShooterAngle() {
        double posX = getAprilTagPoseX(); // offset from crosshair (horizontal)
        double posY = getAprilTagPoseY(); // offset from crosshair (vertical)
        double posZ = getAprilTagPoseZ(); // distance from robot
        double theta = Math.atan2(posY, posZ);
        return theta;
    }

    public double getAutoAimDriveDistanceX() {
        double distance = 0; // distance to the left in X to offset position
        double delta = (distance - getAprilTagPoseX());// - getCameraPoseX();
        return delta;
    }

    public double getAutoAimDriveDistanceZ() {
        double distance = 5; // distance Z from the april tag robot targets
        double delta = (distance - getAprilTagPoseZ());// - getCameraPoseZ();
        return delta;
    }

    public double getAutoAimYawTurnDistance() {
        double targetYaw = 0;
        double delta = targetYaw - getAprilTagPoseYaw();
        return delta;
    }

    // ========================================================
    // ===================== PERIODIC =========================

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SmartDashboard.putNumber("[LL] April Tag", getAprilTagID());
    }
}