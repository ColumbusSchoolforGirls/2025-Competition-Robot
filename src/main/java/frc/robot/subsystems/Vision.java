package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

import edu.wpi.first.apriltag.AprilTagFieldLayout;


public class Vision{
  public Pose2d botPose;
  public Pose2d tempPose;
  public double limeLatency;
  public boolean apriltagLimelightConnected = false;
  public boolean NNLimelightConnected = false;
  public Pose2d targetRobotRelativePose;
  public AprilTagFieldLayout aprilTagFieldLayout;

  //Shuffleboard telemetry - pose estimation
  private ShuffleboardTab tab = Shuffleboard.getTab("Vision");
  public GenericEntry visionXDataEntry = tab.add("VisionPose X", 0).getEntry();
  public GenericEntry visionYDataEntry = tab.add("VisionPose Y", 0).getEntry();
  public GenericEntry visionRotDataEntry = tab.add("VisionPose Rotation", 0).getEntry();

  // TODO- change values later!!!
  private Vision() { 
    botPose = new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0)));
    tempPose = new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(0)));
    targetRobotRelativePose = new Pose2d();
    limeLatency = 0.0;
    // botPose3d = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    // targetSeenCount = 0;
    // aimHorizontalOffset = 0;
    // aimVerticalOffset = 0;
  }     
}
