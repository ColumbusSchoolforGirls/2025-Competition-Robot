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
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;


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
      DriveConstants.TRANSLATION_2D_OFFSET);
  private final Translation2d frontRightLocation = new Translation2d(DriveConstants.TRANSLATION_2D_OFFSET,
      -DriveConstants.TRANSLATION_2D_OFFSET);
  private final Translation2d backLeftLocation = new Translation2d(-DriveConstants.TRANSLATION_2D_OFFSET,
      DriveConstants.TRANSLATION_2D_OFFSET);
  private final Translation2d backRightLocation = new Translation2d(-DriveConstants.TRANSLATION_2D_OFFSET,
      -DriveConstants.TRANSLATION_2D_OFFSET);

  private final SwerveModule frontLeft = new SwerveModule(DriveConstants.FL_DRIVE_ID, DriveConstants.FL_TURN_ID,
      DriveConstants.FL_DIO, DriveConstants.FL_CHASSIS_ANGULAR_OFFSET);
  private final SwerveModule frontRight = new SwerveModule(DriveConstants.FR_DRIVE_ID, DriveConstants.FR_TURN_ID,
      DriveConstants.FR_DIO, DriveConstants.FR_CHASSIS_ANGULAR_OFFSET);
  private final SwerveModule backLeft = new SwerveModule(DriveConstants.BL_DRIVE_ID, DriveConstants.BL_TURN_ID,
      DriveConstants.BL_DIO, DriveConstants.BL_CHASSIS_ANGULAR_OFFSET);
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
    frontLeft.getDrivePositionInches();
    frontRight.getDrivePositionInches();
    backLeft.getDrivePositionInches();
    backRight.getDrivePositionInches();
  }

  public void resetTurnEncoders() {
    frontLeft.resetRelativeTurnEncoder();
    frontRight.resetRelativeTurnEncoder();
    backLeft.resetRelativeTurnEncoder();
    backRight.resetRelativeTurnEncoder();
  }

  public void setZero() {
    if (DRIVE_CONTROLLER.getAButtonPressed()) {
      frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
      backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));

  }

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

      SmartDashboard.putNumber("BL Encoder", backLeft.turnRelativeEncoder.getPosition());
      SmartDashboard.putNumber("FL Encoder", frontLeft.turnRelativeEncoder.getPosition());
      SmartDashboard.putNumber("BR Encoder", backRight.turnRelativeEncoder.getPosition());
      SmartDashboard.putNumber("FR Encoder", frontRight.turnRelativeEncoder.getPosition());

      SmartDashboard.putNumber("BLAngOfset", backLeft.turnAbsoluteEncoder.get()*360);
      SmartDashboard.putNumber("FLAngOfset", frontLeft.turnAbsoluteEncoder.get()*360);
      SmartDashboard.putNumber("BRAngOfset", backRight.turnAbsoluteEncoder.get()*360 );
      SmartDashboard.putNumber("FRAngOfset", frontRight.turnAbsoluteEncoder.get()*360); 

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

  // TODO: use after aligning with AprilTag (to make sure we're at 0)?
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
    // TODO: make the robot align with the AprilTag first to make sure we're at 0
    zeroHeading();
    resetEncoders();
    resetTurnEncoders();
    setBrakeMode();
  }
//This is for auto turning
  public void setAutoTargetAngle(double targetAngle) {
    this.targetAngle = targetAngle;


  }

  
    public boolean turnComplete() {
        gyroDifference = (getHeading() - targetAngle);

        return Math.abs(gyroDifference) < Constants.DriveConstants.TURN_TOLERANCE;
    }

  public boolean driveComplete() {
        driveDifference = targetDistance - frontLeft.getDrivePositionInches(); //change to make better
        if (Math.abs(driveDifference) < Constants.DriveConstants.DISTANCE_TOLERANCE) {
            return true;    
          //if (frontLeftEncoder.getVelocity() < 0.03) { // to prevent skidding bc of turning before drive is complete
                //return true;
            //}
        //} else if (Math.abs(gyro.getVelocityY()) < Constants.AUTO_DRIVE_VELOCITY_THRESHHOLD && Timer.getFPGATimestamp() - startAutoDriveTime > 0.5) { // change: test //change                                                                                
            //System.out.println("Drive stalled - TESTING");
            //return true;
        }
        return false;
    }

    public void startTurn(double angle) {
        this.targetAngle = (angle + getHeading());
    }

    public void resetGyro() {
        gyro.reset();
    }

    public void startDrive(double distanceInches) {
        resetEncoders();
        targetDistance = distanceInches;
    }

    public void gyroTurn(double periodSeconds) {
        gyroDifference = (getHeading() - targetAngle);

        if (Math.abs(gyroDifference) < Constants.DriveConstants.TURN_TOLERANCE) {
            drive(0,0,0,true, periodSeconds);
        } else if (gyroDifference < 0) {
          drive(0,0,  0.0035 * Math.abs(gyroDifference) + 0.05,true, periodSeconds) ;
        } else if (gyroDifference > 0) {
          drive(0,0, -0.0035 * Math.abs(gyroDifference) - 0.05,true, periodSeconds);
        }
    }

    public void autoAlignLimelight(double periodSeconds) {
      final var rot_limelight = limelight.limelight_aim_proportional();
      final var forward_limelight = limelight.limelight_range_proportional();
      //while using Limelight, turn off field-relative driving.
      boolean fieldRelative = false;
      this.drive(forward_limelight, 0.0, rot_limelight, fieldRelative, periodSeconds);
    }

    public boolean isLimelightAligned() {
      double tx = limelight.getTX();
      double ty = limelight.getTY();
      // TODO: tune these on robot
      return (tx < 0.1 && ty < 0.1);
    }
}
