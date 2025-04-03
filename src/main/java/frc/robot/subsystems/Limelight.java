package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;

public class Limelight {
    private static final long NO_APRIL_TAG_ID = -1;
    private static final double NO_TX = 0;
    private static final double NO_TA = 0;

    NetworkTableEntry tx; 
    NetworkTableEntry ty; 
    NetworkTableEntry tv; 
    NetworkTableEntry ta; 
    NetworkTableEntry ts; 
    NetworkTableEntry pos; 
    NetworkTableEntry pos1; 
    NetworkTableEntry pos2; 
    NetworkTableEntry tid;
    NetworkTableEntry pipeline;

    long lastAprilTagID = NO_APRIL_TAG_ID;
    double lastTXValue = NO_TX;
    double lastTAValue = NO_TA;
    double targetingForwardSpeed = 0;
    double targetingStrafeSpeed = 0;
    double lastNoTagTime;

    LinearFilter filter = LinearFilter.singlePoleIIR(0.1, 0.02);
    
    public Limelight(String limelightName) {
        NetworkTable table = NetworkTableInstance.getDefault().getTable(limelightName);
        tx = table.getEntry("tx"); // x axis position
        ty = table.getEntry("ty"); // y axis position
        tv = table.getEntry("tv"); // is there valid target
        ta = table.getEntry("ta"); // area in view
        pos = table.getEntry("campose"); // 3D translation and rotations?
        pos1 = table.getEntry("targetpose_cameraspace");
        pos2 = table.getEntry("targetpose_robotspace");
        tid = table.getEntry("tid");
        pipeline = table.getEntry("pipeline");

    }

    public void updateLimelight() {
        SmartDashboard.putNumber("LimelightTX", getBestTX());
        SmartDashboard.putNumber("LimelightTY", getTY());
        SmartDashboard.putNumber("LimelightTA", getLinearFilterTA());
        //SmartDashboard.putNumber("LimelightROT", getRotation());
        SmartDashboard.putNumber("April Tag ID", getAprilTagID());
        SmartDashboard.putNumber("target strafe speed:", targetingStrafeSpeed);
        SmartDashboard.putNumber("target forward speed:", targetingForwardSpeed);
   
    }

    // public double limelight_aim_proportional() { //gets it to flush angle with target
    //     // kP (constant of proportionality)
    //     // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    //     // if it is too high, the robot will oscillate.
    //     // if it is too low, the robot will never reach its target
    //     // if the robot never turns in the correct direction, kP should be inverted.
    //     double kP = .035;

        

    //     // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the
    //     // rightmost edge of your limelight 3 feed, tx should return roughly 31 degrees.
    //     double targetingAngularVelocity = getRotation() * kP; // TODO: Add the limelight string back when we have the exact Apriltag ID

    //     // convert to radians per second for our drive method
    //     targetingAngularVelocity *= -DriveConstants.MAX_ANGULAR_SPEED*0.2; //TODO: make into constant

    //     // invert since tx is positive when the target is to the right of the crosshair

    //         return targetingAngularVelocity;
   
    //     }
    
    // Simple proportional ranging control with Limelight's "ty" value this works
    // best if your Limelight's mount height and target mount height are different.
    // If your limelight and target are mounted at the same or similar heights, use
    // "ta" (area) for target ranging rather than "ty"
    public double limelight_range_proportional() { //brings it forward to desired area of target
        double kP = .12;
        targetingForwardSpeed = Math.max(Math.sqrt(Math.abs((Constants.DriveConstants.TARGET_TA_VALUE - getLinearFilterTA()))), 0.1) * kP;// TODO: Add the limelight string back when we have the exact Apriltag ID
        targetingForwardSpeed *= DriveConstants.MAX_SPEED;
        targetingForwardSpeed *= 1.0;
        return Math.min(targetingForwardSpeed, 0.5);
    }

    public double limlight_strafe_proportional() { //gets it aligned in x axis
        double kP = .015;
        targetingStrafeSpeed = getBestTX() * kP;// TODO: Add the limelight string back when we have the exact Apriltag ID
        System.out.println(getBestTX());
        targetingStrafeSpeed *= DriveConstants.MAX_SPEED;
        targetingStrafeSpeed *= 1.0;
        return Math.min(targetingStrafeSpeed, 0.15);
    }

    

    /** Get rotation z value from botpose array. */
    
    public void setRightBranchPipeline() {
        pipeline.setNumber(0);
    }

    public void setLeftBranchPipeline() {
        pipeline.setNumber(1);
    }

    public long getAprilTagID() {
        long currentAprilTagID = tid.getInteger(NO_APRIL_TAG_ID);
        if (currentAprilTagID == NO_APRIL_TAG_ID) {
            if (lastNoTagTime == 0) {
                lastNoTagTime = Timer.getFPGATimestamp();
            } else {
                if (Timer.getFPGATimestamp() - lastNoTagTime > 1) {
                    return tid.getInteger(NO_APRIL_TAG_ID);
                }
            }
            return lastAprilTagID; 
        } else {
            lastNoTagTime = 0;
            lastAprilTagID = currentAprilTagID;
            return currentAprilTagID;
        }
    }
    
    // public double getRotation() {
    //     return pos.getDoubleArray(new double[6])[4];
    // }
    
    public double getBestTX() {
        double currentTX = getTX();
        if (currentTX == NO_TX) {
            return lastTXValue;
        } else {
            lastTXValue = currentTX;
            return currentTX;
        }
    }

    public double getTX() {
        return tx.getDouble(0);
    }

    public double getTY() {
        return ty.getDouble(0);
    }

    public double getBestTA() {
        double currentTA = getTA();
        if (currentTA == NO_TA) {
            return lastTAValue;
        } else {
            lastTAValue = currentTA;
            return currentTA;
        }
    }

    public double getLinearFilterTA() {
        return filter.calculate(getBestTA());
    }

    public double getTA() {
        return ta.getDouble(0);
    }
}
