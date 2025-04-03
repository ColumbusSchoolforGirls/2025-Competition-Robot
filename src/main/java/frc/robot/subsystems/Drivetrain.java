// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ControllerConstants.DRIVE_CONTROLLER;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

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
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;


public class Drivetrain {
  private Limelight limelight;

  private double gyroDifference;
  private double alignTurnDifference;
  private double targetAngle;
  private double targetAlignAngle;
  private double driveDifference;
  private double targetDistance;
  private double startDriveTime;
  private double autoDriveTimeLimit;
  private double positiveFactor;
  private double autoSpeed;

  AlignAction step = AlignAction.NOT_RUNNING;

  enum AlignAction {
    TURN, STRAFE_AND_RANGE, DRIVE_FORWARD, DRIVE_SIDEWAYS, NOT_RUNNING, STOPPED, TURN_AGAIN
  }

  AlignAction[] alignActions = {};

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
    SmartDashboard.putString("step", step.toString());
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
    return gyro.getYaw();//gyro.getRotation2d().getDegrees();
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
    step = AlignAction.NOT_RUNNING;
  }

  // This is for auto turning
  public void setAutoTargetAngle(double targetAngle) {
    this.targetAngle = targetAngle;
  }

  public boolean turnComplete() {
    gyroDifference = (getHeading() - targetAngle);

    return Math.abs(gyroDifference) < Constants.DriveConstants.TURN_TOLERANCE;
  }

  public boolean turnToAprilTagComplete() {
    alignTurnDifference = (getHeading() - targetAlignAngle);

    boolean complete = Math.abs(alignTurnDifference) < Constants.DriveConstants.TURN_TOLERANCE;
    if (complete) {
      System.out.println("Turn (limelight) complete");
    }

    return complete;
  }

  double stallStart = 0.0;

  public void testShuffle() {
    SmartDashboard.putNumber("FL Drive", frontLeft.getDrivePositionMeters());
    SmartDashboard.putNumber("FR Drive", frontRight.getDrivePositionMeters());
    SmartDashboard.putNumber("BL Drive", backLeft.getDrivePositionMeters());
    SmartDashboard.putNumber("BR Drive", backRight.getDrivePositionMeters());
  }

  public boolean driveComplete() {
    driveDifference = targetDistance - Math.abs(frontLeft.getDrivePositionMeters());
    if ((Math.abs(driveDifference) < Constants.DriveConstants.DISTANCE_TOLERANCE) || (Timer.getFPGATimestamp() - startDriveTime > autoDriveTimeLimit))  {
      stallStart = 0.0;
      System.out.println("Reached drive target");
      return true;
    }
    System.out.println(driveDifference);

    // the IMU velocty functions are inconsistent and not reliable, do not use, maybe use this as an outline for current spike detection
  //   if (Math.abs(gyro.getVelocityY()) < 0.01) {
  //     if (stallStart != 0.0) {
  //       if (Timer.getFPGATimestamp() - stallStart > 0.2) {
  //         System.out.println("!STALL!");
  //         return true;
  //       }
  //     } else {
  //       System.out.println("Start stall");
  //       stallStart = Timer.getFPGATimestamp();
  //     }
  //   } else {
  //     stallStart = 0.0;
  //   }
    return false;
  }

  public void autoDrive(double periodSeconds) {
    driveDifference = targetDistance - Math.abs(frontLeft.getDrivePositionMeters());

    if (Math.abs(driveDifference) > Constants.DriveConstants.DISTANCE_TOLERANCE) {
      if (Math.abs(driveDifference) < 0.5) {
        autoSpeed -= 0.12;
      } else {
        if (autoSpeed < AutoConstants.MAX_AUTO_SPEED) {
          autoSpeed += 0.1;
        }
      }
      autoSpeed = Math.max(autoSpeed, AutoConstants.MIN_AUTO_SPEED); // Speed can never switch signs on us
      drive(autoSpeed*positiveFactor, 0, 0, false, periodSeconds);
    }
  }

  public void autoSideDrive(double periodSeconds) {
    driveDifference = targetDistance - Math.abs(frontLeft.getDrivePositionMeters());

    if (Math.abs(driveDifference) > Constants.DriveConstants.DISTANCE_TOLERANCE) {
      if (Math.abs(driveDifference) < 0.5) {
        autoSpeed -= 0.12;
      } else {
        if (autoSpeed < AutoConstants.MAX_AUTO_SPEED) {
          autoSpeed += 0.1;
        }
      }
      autoSpeed = Math.max(autoSpeed, AutoConstants.MIN_AUTO_SPEED); // Speed can never switch signs on us
      drive(0, autoSpeed*positiveFactor, 0, false, periodSeconds);
    }
  }

  // public void limelightAlignForwardDrive(double periodSeconds) {
  //   limelightAlignDriveDifference = 0.787 - Math.abs(frontLeft.getDrivePositionMeters());
  //   if (Math.abs(limelightAlignDriveDifference) > Constants.DriveConstants.DISTANCE_TOLERANCE) {
  //     drive(0.3, 0.0, 0, false, periodSeconds); //figure out speeds
  //   }
  // }

  // public void limelightAlignSideDrive(double periodSeconds, double targetSideAlignDistance) {
  //   limelightAlignDriveDifference = targetSideAlignDistance - Math.abs(frontLeft.getDrivePositionMeters()); //maybe change 0.5
  //   if (Math.abs(limelightAlignDriveDifference) > Constants.DriveConstants.DISTANCE_TOLERANCE) {
  //     drive(0.0, -0.2, 0, false, periodSeconds); //figure out speeds
  //   }

  // }

  public void startTurn(double angle) {
    this.targetAngle = (angle + getHeading());
  }

  public void updateDistanceAndAngleValues() {
    SmartDashboard.putNumber("gyro angle", getHeading());
    SmartDashboard.putNumber("distance", frontLeft.getDrivePositionMeters());
  }

  public void resetGyro() {
    gyro.reset();
  }

  public void startDrive(double distanceMeters, double timeLimit) {
    resetEncoders();
    targetDistance = Math.abs(distanceMeters);
    autoDriveTimeLimit = timeLimit;
    if (distanceMeters < 0) {
      positiveFactor = -1;
    } else {
      positiveFactor = 1;
    }
    startDriveTime = Timer.getFPGATimestamp();
    autoSpeed = AutoConstants.MIN_AUTO_SPEED;
  }

  public void autoAlignTurn(double periodSeconds, double target) {
    targetAlignAngle = target;
    alignTurnDifference = (getHeading() - targetAlignAngle);
    
    if (Math.abs(alignTurnDifference) < Constants.DriveConstants.TURN_TOLERANCE) {
      drive(0, 0, 0, false, periodSeconds);
    } else if (alignTurnDifference < 0) {
      drive(0, 0, 0.05 * Math.abs(alignTurnDifference) + 0.5, false, periodSeconds);
    } else if (alignTurnDifference > 0) {
      drive(0, 0, -0.05 * alignTurnDifference - 0.5, false, periodSeconds); // 0.1 * al
    }
  }

  public void gyroTurn(double periodSeconds) {
    gyroDifference = (getHeading() - targetAngle);

    if (Math.abs(gyroDifference) < Constants.DriveConstants.TURN_TOLERANCE) {
      drive(0, 0, 0, false, periodSeconds);
    } else if (gyroDifference < 0) {
      drive(0, 0, 0.05 * Math.abs(gyroDifference) + 0.2, false, periodSeconds);
    } else if (gyroDifference > 0) {
      drive(0, 0, -0.05 * gyroDifference - 0.2, false, periodSeconds);
    }
  }

  // public void autoAlignLimelightAngle(double periodSeconds) {
  //   //final var rot_limelight = limelight.limelight_aim_proportional();
  //   boolean fieldRelative = false;
  //   if (!isLimelightAngleAligned()) {
  //     this.drive(0.0, 0.0, rot_limelight, fieldRelative, periodSeconds);
  //   } else if (isLimelightAngleAligned()) {
  //     drive(0, 0, 0, false, periodSeconds);
  //   }
  // }

  public void resetAlignStateWithButton() {
    if (DRIVE_CONTROLLER.getXButton()) {
      resetAlignState();
    }
  }

  public void resetAlignState() {
    step = AlignAction.NOT_RUNNING;
  }

  public boolean triggersPressed() {
    return DRIVE_CONTROLLER.getLeftTriggerAxis() > Constants.ControllerConstants.TRIGGER_DEADZONE || DRIVE_CONTROLLER.getRightTriggerAxis() > Constants.ControllerConstants.TRIGGER_DEADZONE;
  }

  public void strafeAndRange(double periodSeconds) {
    final var strafe_limelight = limelight.limlight_strafe_proportional();
    final var forward_limelight = limelight.limelight_range_proportional();
    boolean fieldRelative = false;
    this.drive(forward_limelight, strafe_limelight, 0.0, fieldRelative, periodSeconds);
  }

  public void setAlignStateNotRunning() {
    step = AlignAction.NOT_RUNNING;
  }

  public void autoAlignLimelight(double periodSeconds, boolean isAuto) {
    //final var rot_limelight = limelight.limelight_aim_proportional();
    // while using Limelight, turn off field-relative driving.

    if (step == AlignAction.NOT_RUNNING) {
      if (triggersPressed() || isAuto) { //TODO: need to make sure auto can be implemented
        step = AlignAction.TURN;
        System.out.println("step:" + step);
      } 
    } else if (step == AlignAction.TURN) {
      if (((triggersPressed() || isAuto) && turnToAprilTagComplete())) {
        step = AlignAction.STRAFE_AND_RANGE;
        System.out.println("step:" + step);
      } else if (!triggersPressed() && !isAuto) {
        step = AlignAction.STOPPED;
        stop(periodSeconds);
        System.out.println("step:" + step);
      }
      turnToAprilTag(periodSeconds);
    } else if (step == AlignAction.STRAFE_AND_RANGE) {
      //turnToAprilTag(periodSeconds);
      if (((triggersPressed() || isAuto) && isLimelightAligned() && isLimelightStrafeAligned())) {
        //resetTurnEncoders();
        step = AlignAction.STOPPED; // TURN_AGAIN
        stop(periodSeconds);
        System.out.println("step:" + step);
      } else if ((!triggersPressed() && !isAuto)) {
        step = AlignAction.STOPPED;
        stop(periodSeconds);
        System.out.println("step:" + step);
      }
      if (limelight.getAprilTagID() == -1) {
        step = AlignAction.STOPPED;
        stop(periodSeconds);
        System.out.println("step:" + step);
      }
      strafeAndRange(periodSeconds);
    } else if (step == AlignAction.TURN_AGAIN) {
      if (turnToAprilTagComplete() || (!triggersPressed() && !isAuto)) {
        step = AlignAction.STOPPED;
        stop(periodSeconds);
        System.out.println("step:" + step);
      }
      turnToAprilTag(periodSeconds);
    }
  }

  public boolean isStepStopped() {
    return step == AlignAction.STOPPED;
  }

  public void turnToAprilTag(double periodSeconds) {
    if (limelight.getAprilTagID() == 10 || limelight.getAprilTagID() == 21) {
      autoAlignTurn(periodSeconds, 0);
    } else if (limelight.getAprilTagID() == 20 || limelight.getAprilTagID() == 11) {
      autoAlignTurn(periodSeconds, -60); //change w testing
    } else if (limelight.getAprilTagID() == 22 || limelight.getAprilTagID() == 9) {
      autoAlignTurn(periodSeconds, 60); //change w testing
    } else if (limelight.getAprilTagID() == 19 || limelight.getAprilTagID() == 6) {
      autoAlignTurn(periodSeconds, 0); //change w testing //-150
    } else if (limelight.getAprilTagID() == 17 || limelight.getAprilTagID() == 8) {
      autoAlignTurn(periodSeconds, 150); //change w testing
    } else if (limelight.getAprilTagID() == 18 || limelight.getAprilTagID() == 7) {
      autoAlignTurn(periodSeconds, 180); 
    }
  }

  public void teleopAutoAlign() {
    if (DRIVE_CONTROLLER.getRightTriggerAxis() > Constants.ControllerConstants.TRIGGER_DEADZONE) {
      limelight.setRightBranchPipeline();
      //autoAlignLimelight(periodSeconds);
    } else if (DRIVE_CONTROLLER.getLeftTriggerAxis() > Constants.ControllerConstants.TRIGGER_DEADZONE) {
      limelight.setLeftBranchPipeline();
      //autoAlignLimelight(periodSeconds);
    }
  }

  // public boolean isLimelightAngleAligned() {
  //   double rot = limelight.getRotation(); //rotation
  //   return (Math.abs(rot) < Constants.DriveConstants.ROTATION_TOLERANCE);
  // }

  public boolean isLimelightStrafeAligned() {
    double tx = limelight.getBestTX();
    boolean aligned = (Math.abs(tx) < Constants.DriveConstants.TX_TOLERANCE);
    if (aligned) {
      System.out.println("STRAFE Aligned");
    }
    return aligned;
  }

  public boolean isLimelightAligned() {
    // double tx = limelight.getTX();
    // double ty = limelight.getTY();
    double ta = limelight.getLinearFilterTA();
    // TODO: tune these on robot
    boolean aligned = (Math.abs(ta) >= Constants.DriveConstants.TARGET_TA_VALUE - 0.02);
    if (aligned) {
      System.out.println("Aligned");
    }
    return aligned;
  }

  public void stop(double periodSeconds) {
    drive(0, 0, 0, false, periodSeconds);
  }
}
