package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;


public class Limelight {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry tx = table.getEntry("tx"); // x axis position
    NetworkTableEntry ty = table.getEntry("ty"); // y axis position
    NetworkTableEntry tv = table.getEntry("tv"); // is there valid target
    NetworkTableEntry ta = table.getEntry("ta"); // area in view
    NetworkTableEntry ts = table.getEntry("ts0"); // area in view
    NetworkTableEntry pos = table.getEntry("camera-pose_targetspace"); // 3D translation and rotations?
    NetworkTableEntry pos1 = table.getEntry("target-pose_cameraspace");
    NetworkTableEntry pos2 = table.getEntry("target-pose_robotspace");

    double ySpeed = 0;
    double rotationSpeed = 0;
    long lastTargetTime = 0;

    public Limelight() {
     
    }

    public void updateLimelight() {

    }

    /** Get rotation z value from botpose array. */
    public double getRotation() {
        return pos.getDoubleArray(new double[6])[5];
    }

    public double getTX() {
        return tx.getDouble(0);
    }

    public double getTY() {
        return ty.getDouble(0);
    }

    public double getTA() {
        return ta.getDouble(0);
    }
}
