// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import frc.robot.Constants;

public class SwerveModule {

  // private static final double kModuleMaxAngularVelocity
  // =Constants.DriveConstants;
  // private static final double kModuleMaxAngularAcceleration =
  // 2 * Math.PI; // radians per second squared

  private final PWMSparkMax driveMotor;
  private final PWMSparkMax turningMotor;

  private final Encoder driveEncoder;
  private final Encoder turningEncoder;

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController drivePIDController = new PIDController(Constants.SwerveConstants.PROPORTIONAL_COEFFICIENT,
      Constants.SwerveConstants.INTEGRAL_COEFFICIENT, Constants.SwerveConstants.DERIVATIVE_COEFFICIENT);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController turningPIDController = new ProfiledPIDController(
      Constants.SwerveConstants.PROPORTIONAL_COEFFICIENT,
      Constants.SwerveConstants.INTEGRAL_COEFFICIENT,
      Constants.SwerveConstants.DERIVATIVE_COEFFICIENT,
      new TrapezoidProfile.Constraints(
          Constants.SwerveConstants.MODULE_MAX_ANGULAR_VELOCITY,
          Constants.SwerveConstants.MODULE_MAX_ANGULAR_ACCELERATION));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.STATIC_GAIN_DRIVE, Constants.SwerveConstants.VELOCITY_GAIN_DRIVE);
  private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.STATIC_GAIN_TURN, Constants.SwerveConstants.VELOCITY_GAIN_TURN);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder
   * and turning encoder.
   *
   * @param driveMotorChannel      PWM output for the drive motor.
   * @param turningMotorChannel    PWM output for the turning motor.
   * @param driveEncoderChannelA   DIO input for the drive encoder channel A
   * @param driveEncoderChannelB   DIO input for the drive encoder channel B
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int driveEncoderChannelA,
      int driveEncoderChannelB,
      int turningEncoderChannelA,
      int turningEncoderChannelB) {
    driveMotor = new PWMSparkMax(driveMotorChannel);
    turningMotor = new PWMSparkMax(turningMotorChannel);

    driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
    turningEncoder = new Encoder(turningEncoderChannelA, turningEncoderChannelB);

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    driveEncoder.setDistancePerPulse(
        2 * Math.PI * Constants.SwerveConstants.WHEEL_RADIUS / Constants.SwerveConstants.ENCODER_RES);

    // Set the distance (in this case, angle) in radians per pulse for the turning
    // encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    turningEncoder.setDistancePerPulse(2 * Math.PI / Constants.SwerveConstants.ENCODER_RES);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        driveEncoder.getRate(), new Rotation2d(turningEncoder.getDistance()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        driveEncoder.getDistance(), new Rotation2d(turningEncoder.getDistance()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d encoderRotation = new Rotation2d(turningEncoder.getDistance());

    // Optimize the reference state to avoid spinning further than 90 degrees
    desiredState.optimize(encoderRotation);

    // Scale speed by cosine of angle error. This scales down movement perpendicular
    // to the desired
    // direction of travel that can occur when modules change directions. This
    // results in smoother
    // driving.
    desiredState.cosineScale(encoderRotation);

    // Calculate the drive output from the drive PID controller.

    final double driveOutput = drivePIDController.calculate(driveEncoder.getRate(), desiredState.speedMetersPerSecond);

    final double finalDriveFeedforward = driveFeedforward.calculate(desiredState.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput = turningPIDController.calculate(
        turningEncoder.getDistance(), desiredState.angle.getRadians());

    final double finalTurnFeedforward = turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);

    driveMotor.setVoltage(driveOutput + finalDriveFeedforward);
    turningMotor.setVoltage(turnOutput + finalTurnFeedforward);
  }
}
