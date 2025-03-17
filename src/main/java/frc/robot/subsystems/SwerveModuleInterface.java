// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;



public interface SwerveModuleInterface {

  public void resetRelativeTurnEncoder();

  /** Returns the current state of the module. */
  public SwerveModuleState getState();

  /** Returns the current position of the module. */
  public SwerveModulePosition getPosition() ;

  public double getDrivePositionMeters();

  public double getVelocityMetersPerSecond();

  public double getVelocityRPM();

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) ;

  public void setCurrentLimit() ;

  /** Zeroes all the SwerveModule drive encoders. */
  public void resetEncoder() ;

  public void setBrakeMode();

  public void setCoastMode();
}