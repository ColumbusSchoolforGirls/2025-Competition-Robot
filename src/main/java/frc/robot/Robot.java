// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ControllerConstants.DRIVE_CONTROLLER; // Noah HATES this, but says it's not a bad use . . .

import java.util.HashMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;


public class Robot extends TimedRobot {
  private final Drivetrain swerve = new Drivetrain();
  private final Limelight limelight = new Limelight();
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  @Override
  public void robotInit() {
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    swerve.updateOdometry();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }


    double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = limelight.getTX() * kP; //TODO: Add the limelight string back when we have the exact Apriltag ID

    // convert to radians per second for our drive method
    targetingAngularVelocity *= Constants.DriveConstants.MAX_ANGULAR_SPEED;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = limelight.getTY() * kP;//TODO: Add the limelight string back when we have the exact Apriltag ID
    targetingForwardSpeed *= Constants.DriveConstants.MAX_SPEED;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  private void drive(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed =
        -xspeedLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getLeftY(), 0.02))
            * Constants.DriveConstants.MAX_SPEED;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
        -yspeedLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getLeftX(), 0.02))
            * Constants.DriveConstants.MAX_SPEED;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot =
        -rotLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getRightX(), 0.02))
            * Constants.DriveConstants.MAX_ANGULAR_SPEED;

    // while the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
    if(DRIVE_CONTROLLER.getAButton())
    {
        final var rot_limelight = limelight_aim_proportional();
        rot = rot_limelight;

        final var forward_limelight = limelight_range_proportional();
        xSpeed = forward_limelight;

        //while using Limelight, turn off field-relative driving.
        fieldRelative = false;
    }

   swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }


  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final double xSpeed =
        -xspeedLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getLeftY(), 0.02))
            * Constants.DriveConstants.MAX_SPEED;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final double ySpeed =
        -yspeedLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getLeftX(), 0.02))
            * Constants.DriveConstants.MAX_SPEED;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final double rot =
        -rotLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getRightX(), 0.02))
            * Constants.DriveConstants.MAX_ANGULAR_SPEED;

    swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());

    SmartDashboard.putNumber("LimelightTX", this.limelight.getTX());
    SmartDashboard.putNumber("LimelightTY", this.limelight.getTY());
    SmartDashboard.putNumber("LimelightTA", this.limelight.getTA());
  }

   /** This function is called once when the robot is disabled. */
   @Override
   public void disabledInit() {
   }
 
   /** This function is called periodically when disabled. */
   @Override
   public void disabledPeriodic() {
   }
 
   /** This function is called once when test mode is enabled. */
   @Override
   public void testInit() {
   }
 
   /** This function is called periodically during test mode. */
   @Override
   public void testPeriodic() {
   }
 
   /** This function is called once when the robot is first started up. */
   @Override
   public void simulationInit() {
   }
 
   /** This function is called periodically whilst in simulation. */
   @Override
   public void simulationPeriodic() {
   }
} 

