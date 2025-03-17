// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;



public class MockSwerveModule implements SwerveModuleInterface {
  // TODO: add later for precision
  // private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3);
  // private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /** Constructs a MockSwerveModule that does nothing for when we don't have hardware */
  public MockSwerveModule() {
 
  }

  public void resetRelativeTurnEncoder() {

  }

  /** Returns the current state of the module. */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
       0.0, new Rotation2d(0,0));
  }

  /** Returns the current position of the module. */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        0.0, new Rotation2d(0.0));
  }

  public double getDrivePositionMeters() {
    return 0;  }

  public double getVelocityMetersPerSecond() {
    return 0;  }

  public double getVelocityRPM() {
    return 0;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
  }

  public void setCurrentLimit() {
  }

  /** Zeroes all the SwerveModule drive encoders. */
  public void resetEncoder() {
   
  }

  public void setBrakeMode(){ 
  }

  public void setCoastMode(){
  }
}