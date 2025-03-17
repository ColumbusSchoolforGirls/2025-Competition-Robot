// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import static frc.robot.Constants.ControllerConstants.DRIVE_CONTROLLER;

public class Drivetrain {
  private Limelight limelight;

  private double gyroDifference;
  private double targetAngle;
  private double driveDifference;
  private double targetDistance;

  private final Translation2d frontLeftLocation = new Translation2d(DriveConstants.TRANSLATION_2D_OFFSET,
      -DriveConstants.TRANSLATION_2D_OFFSET);
  private final Translation2d frontRightLocation = new Translation2d(DriveConstants.TRANSLATION_2D_OFFSET,
      DriveConstants.TRANSLATION_2D_OFFSET);
  private final Translation2d backLeftLocation = new Translation2d(-DriveConstants.TRANSLATION_2D_OFFSET,
      -DriveConstants.TRANSLATION_2D_OFFSET);
  private final Translation2d backRightLocation = new Translation2d(-DriveConstants.TRANSLATION_2D_OFFSET,
      DriveConstants.TRANSLATION_2D_OFFSET);

  private final SwerveModule frontLeft = new SwerveModule(DriveConstants.FL_DRIVE_ID, DriveConstants.FL_TURN_ID,
      DriveConstants.FL_DIO, DriveConstants.FL_CHASSIS_ANGULAR_OFFSET);
  private final SwerveModule backLeft = new SwerveModule(DriveConstants.BL_DRIVE_ID, DriveConstants.BL_TURN_ID,
      DriveConstants.BL_DIO, DriveConstants.BL_CHASSIS_ANGULAR_OFFSET);
  private final SwerveModule frontRight = new SwerveModule(DriveConstants.FR_DRIVE_ID, DriveConstants.FR_TURN_ID,
      DriveConstants.FR_DIO, DriveConstants.FR_CHASSIS_ANGULAR_OFFSET);
  private final SwerveModule backRight = new SwerveModule(DriveConstants.BR_DRIVE_ID, DriveConstants.BR_TURN_ID,
      DriveConstants.BR_DIO, DriveConstants.BR_CHASSIS_ANGULAR_OFFSET);

  private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      kinematics,
      gyro.getRotation2d(),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          backLeft.getPosition(),
          backRight.getPosition()
      });

  public Drivetrain(Limelight limelight) {
    this.limelight = limelight;
  }

  public void getDriveEncoders() {
    frontLeft.getDrivePositionMeters();
    frontRight.getDrivePositionMeters();
    backLeft.getDrivePositionMeters();
    backRight.getDrivePositionMeters();
  }

  public void resetTurnEncoders() {
    frontLeft.resetRelativeTurnEncoder();
    frontRight.resetRelativeTurnEncoder();
    backLeft.resetRelativeTurnEncoder();
    backRight.resetRelativeTurnEncoder();
  }

  public void driverResetTurnEncoders() {
    if (DRIVE_CONTROLLER.getBButtonPressed()) {
      resetTurnEncoders();
    }
  }

  public void setZero() {
    if (DRIVE_CONTROLLER.getAButtonPressed()) {
      frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));

    }

    frontLeft.updateModule();
    frontRight.updateModule();
    backLeft.updateModule();
    backRight.updateModule();
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
    SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(
        ChassisSpeeds.discretize(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, rot, gyro.getRotation2d())
                : new ChassisSpeeds(xSpeed, ySpeed, rot),
            periodSeconds));
    setModuleStates(swerveModuleStates);
  }

  /** Sets the wheels into an X formation to prevent movement. */
  public void setX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    odometry.update(
        gyro.getRotation2d(),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });
  }

  /* Updates the front left swerve module on dashboard FOR TESTING. */
  public void updateSmartDashboard() {
    frontLeft.updateSmartDashboard();
  }

  /** Returns the currently-estimated pose of the robot. */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        },
        pose);
  }

  /** Update the odometry in the periodic block. */
  public void periodic() {
    odometry.update(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            backLeft.getPosition(),
            backRight.getPosition()
        });

    SmartDashboard.putNumber("BL Encoder", (backLeft.turnRelativeEncoder.getPosition() * 180) / 3.14159);
    SmartDashboard.putNumber("FL Encoder", (frontLeft.turnRelativeEncoder.getPosition() * 180) / 3.14159);
    SmartDashboard.putNumber("BR Encoder", (backRight.turnRelativeEncoder.getPosition() * 180) / 3.14159);
    SmartDashboard.putNumber("FR Encoder", (frontRight.turnRelativeEncoder.getPosition() * 180) / 3.14159);

    SmartDashboard.putNumber("BL AbsEncoder", backLeft.turnAbsoluteEncoder.get() * 360);
    SmartDashboard.putNumber("FL AbsEncoder", frontLeft.turnAbsoluteEncoder.get() * 360);
    SmartDashboard.putNumber("BR AbsEncoder", backRight.turnAbsoluteEncoder.get() * 360);
    SmartDashboard.putNumber("FR AbsEncoder", frontRight.turnAbsoluteEncoder.get() * 360);

    SmartDashboard.putNumber("BL TargetRel", backLeft.getAbsoluteEncoderAngle() * 180 / Math.PI);
    SmartDashboard.putNumber("FL TargetRel", frontLeft.getAbsoluteEncoderAngle() * 180 / Math.PI);
    SmartDashboard.putNumber("BR TargetRel", backRight.getAbsoluteEncoderAngle() * 180 / Math.PI);
    SmartDashboard.putNumber("FR TargetRel", frontRight.getAbsoluteEncoderAngle() * 180 / Math.PI);
  }

  public void resetEncoders() {
    frontLeft.resetEncoder();
    frontRight.resetEncoder();
    backLeft.resetEncoder();
    backRight.resetEncoder();
  }

  public void setBrakeMode() {
    frontLeft.setBrakeMode();
    frontRight.setBrakeMode();
    backLeft.setBrakeMode();
    backRight.setBrakeMode();
  }

  public void setCoastMode() {
    frontLeft.setCoastMode();
    frontRight.setCoastMode();
    backLeft.setCoastMode();
    backRight.setCoastMode();
  }

  public void zeroHeading() {
    gyro.reset();
  }

  /** Returns the heading of the robot in degrees from -180 to 180. */
  public double getHeading() {
    return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
  }

  /** Returns the turn rate of the robot in degrees per second. */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.GyroReversed ? -1.0 : 1.0);
  }

  public void driveInit() {
    zeroHeading();
    resetEncoders();
    resetTurnEncoders();
    setBrakeMode();
  }

  // This is for auto turning
  public void setAutoTargetAngle(double targetAngle) {
    this.targetAngle = targetAngle;
  }

  public boolean turnComplete() {
    gyroDifference = (getHeading() - targetAngle);

    return Math.abs(gyroDifference) < Constants.DriveConstants.TURN_TOLERANCE;
  }

  double stallStart = 0.0;

  public boolean driveComplete() {
    driveDifference = targetDistance - odometry.getPoseMeters().getX();
    if (Math.abs(driveDifference) < Constants.DriveConstants.DISTANCE_TOLERANCE) {
      stallStart = 0.0;
      return true;
    }

    // Prevents the robot from burning out driving continuously into a wall // TODO:
    // check if this works // TODO: velocityZ is probably wrong
    if (gyro.getVelocityZ() < 0.01) {
      if (stallStart != 0.0) {
        if (Timer.getFPGATimestamp() - stallStart > 0.5) {
          return true;
        }
      } else {
        stallStart = Timer.getFPGATimestamp();
      }
    } else {
      stallStart = 0.0;
    }
    return false;
  }

  public void startTurn(double angle) {
    this.targetAngle = (angle + getHeading());
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void startDrive(double distanceMeters) {
    resetEncoders();
    targetDistance = distanceMeters;
  }

  public void gyroTurn(double periodSeconds) {
    gyroDifference = (getHeading() - targetAngle);

    if (Math.abs(gyroDifference) < Constants.DriveConstants.TURN_TOLERANCE) {
      drive(0, 0, 0, true, periodSeconds);
    } else if (gyroDifference < 0) {
      drive(0, 0, 0.0035 * Math.abs(gyroDifference) + 0.05, true, periodSeconds);
    } else if (gyroDifference > 0) {
      drive(0, 0, -0.0035 * Math.abs(gyroDifference) - 0.05, true, periodSeconds);
    }
  }

  public void autoAlignLimelight(double periodSeconds) {
    final var rot_limelight = limelight.limelight_aim_proportional();
    final var forward_limelight = limelight.limelight_range_proportional();
    // while using Limelight, turn off field-relative driving.
    boolean fieldRelative = false;
    this.drive(forward_limelight, 0.0, rot_limelight, fieldRelative, periodSeconds);
  }

  public void teleopAutoAlign(double periodSeconds) {
    if (DRIVE_CONTROLLER.getXButtonPressed()) {
      autoAlignLimelight(periodSeconds);

    }
  }

  public boolean isLimelightAligned() {
    double tx = limelight.getTX();
    double ty = limelight.getTY();
    // TODO: tune these on robot
    return (tx < 0.1 && ty < 0.1);
  }
}
