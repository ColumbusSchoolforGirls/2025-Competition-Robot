// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.SwerveConstants;


public class SwerveModule {
  private final SparkMax driveMotor;
  private final SparkMax turnMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final DutyCycleEncoder turnAbsoluteEncoder; // CTRE SRX Mag Encoder using pulses, used only at RobotInit to reset relative encoder

  private final SparkClosedLoopController driveClosedLoopController;
  private final SparkClosedLoopController turnClosedLoopController;

  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

  // TODO: add later for precision
  // private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(1, 3);
  // private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   */
  public SwerveModule(int driveMotorID, int turnMotorID, int turnDIOPin, double chassisAngularOffset) {
    driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
    turnMotor = new SparkMax(turnMotorID, MotorType.kBrushless);

    driveEncoder = driveMotor.getEncoder(); // Drive motor only has a relative encoder
    turnRelativeEncoder = turnMotor.getEncoder();
    turnAbsoluteEncoder = new DutyCycleEncoder(turnDIOPin);

    driveClosedLoopController = driveMotor.getClosedLoopController();
    turnClosedLoopController = turnMotor.getClosedLoopController();

    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(turnRelativeEncoder.getPosition());
    driveEncoder.setPosition(0);
  }

  public void setRelativeTurnEncoder() {
    turnRelativeEncoder.setPosition(turnAbsoluteEncoder.get() + chassisAngularOffset);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(
        driveEncoder.getVelocity(), new Rotation2d(turnRelativeEncoder.getPosition()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getPosition(), new Rotation2d(turnRelativeEncoder.getPosition()));
  }

  public double getDrivePositionInches() {
    return driveEncoder.getPosition() / SwerveConstants.GEAR_RATIO * SwerveConstants.WHEEL_CIRCUMFERENCE;
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Apply chassis angular offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));
    
    // Optimize the reference state to avoid spinning further than 90 degrees.
    Rotation2d encoderRotation = new Rotation2d(turnRelativeEncoder.getPosition());
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired direction of travel that can occur when modules change directions.
    // This results in smoother driving.
    desiredState.cosineScale(encoderRotation);

    driveClosedLoopController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    turnClosedLoopController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    // TODO: add later with feedforward control
//     // Calculate the drive output from the drive PID controller.
//     final double driveOutput =
// drivePIDController.calculate(driveEncoder.getRate(), desiredState.speedMetersPerSecond);

//     final double finalDriveFeedforward = driveFeedforward.calculate(desiredState.speedMetersPerSecond);

//     // Calculate the turning motor output from the turning PID controller.
//     final double turnOutput =
//         turningPIDController.calculate(
//             turningEncoder.getDistance(), desiredState.angle.getRadians());

//     final double finalTurnFeedforward =
//         turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);

//     driveMotor.setVoltage(driveOutput + finalDriveFeedforward);
//     turningMotor.setVoltage(turnOutput + finalTurnFeedforward);

    this.desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule drive encoders. */
  public void resetEncoder() {
    driveEncoder.setPosition(0);
  }
}
