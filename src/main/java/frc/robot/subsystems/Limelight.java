package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Limelight {
    NetworkTableEntry tx; 
    NetworkTableEntry ty; 
    NetworkTableEntry tv; 
    NetworkTableEntry ta; 
    NetworkTableEntry ts; 
    NetworkTableEntry pos; 
    NetworkTableEntry pos1; 
    NetworkTableEntry pos2; 
    
    public Limelight(String limelightName) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        tx = table.getEntry("tx"); // x axis position
        ty = table.getEntry("ty"); // y axis position
        tv = table.getEntry("tv"); // is there valid target
        ta = table.getEntry("ta"); // area in view
        ts = table.getEntry("ts0"); // area in view
        pos = table.getEntry("camera-pose_targetspace"); // 3D translation and rotations?
        pos1 = table.getEntry("target-pose_cameraspace");
        pos2 = table.getEntry("target-pose_robotspace");

    }

    public void updateLimelight() {
        SmartDashboard.putNumber("LimelightTX", getTX());
        SmartDashboard.putNumber("LimelightTY", getTY());
        SmartDashboard.putNumber("LimelightTA", getTA());
    }

    public double limelight_aim_proportional() { //gets it to flush angle with target
        // kP (constant of proportionality)
        // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
        // if it is too high, the robot will oscillate.
        // if it is too low, the robot will never reach its target
        // if the robot never turns in the correct direction, kP should be inverted.
        double kP = .035;

        

        // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
        // rightmost edge of your limelight 3 feed, tx should return roughly 31 degrees.
        double targetingAngularVelocity = getRotation() * kP; // TODO: Add the limelight string back when we have the exact Apriltag ID

        // convert to radians per second for our drive method
        targetingAngularVelocity *= DriveConstants.MAX_ANGULAR_SPEED*0.2; //TODO: make into constant

        // invert since tx is positive when the target is to the right of the crosshair

            return targetingAngularVelocity;
   
        }
    
    // Simple proportional ranging control with Limelight's "ty" value this works
    // best if your Limelight's mount height and target mount height are different.
    // If your limelight and target are mounted at the same or similar heights, use
    // "ta" (area) for target ranging rather than "ty"
    public double limelight_range_proportional() { //brings it forward to desired area of target
        double kP = .09;
        double targetingForwardSpeed = Math.max(Math.sqrt(Constants.DriveConstants.TARGET_TA_VALUE - getTA()), 0.1) * kP;// TODO: Add the limelight string back when we have the exact Apriltag ID
        targetingForwardSpeed *= DriveConstants.MAX_SPEED;
        targetingForwardSpeed *= 1.0;
        return targetingForwardSpeed;
    }

    public double limlight_strafe_proportional() { //gets it aligned in x axis
        double kP = .09;
        double targetingStrafeSpeed = Math.max(getTX(), 0.1) * kP;// TODO: Add the limelight string back when we have the exact Apriltag ID
        targetingStrafeSpeed *= DriveConstants.MAX_SPEED;
        targetingStrafeSpeed *= 1.0;
        return targetingStrafeSpeed;
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
