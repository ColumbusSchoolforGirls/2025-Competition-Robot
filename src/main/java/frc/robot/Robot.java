// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ControllerConstants.DRIVE_CONTROLLER; // Noah HATES this, but says it's not a bad use . . .

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.CoralSystem;
import frc.robot.subsystems.Climber;


public class Robot extends TimedRobot {
  private final Drivetrain swerve = new Drivetrain();
  private final Limelight limelight = new Limelight();
  private final CoralSystem coralSystem = new CoralSystem();
  private final Climber climber = new Climber();

  private final AutoPaths autoPaths = new AutoPaths();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  @Override
  public void robotInit() {
    // Starts recording to data log
    DataLogManager.start(); // TODO: maybe remove one, because logs double
    
    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());

    autoPaths.setAutoPaths();
  }

  @Override
  public void robotPeriodic() {
    coralSystem.elevator();
    climber.climb();
    limelight.updateLimelight();
  }

  @Override
  public void autonomousInit() {
    swerve.setBrakeMode();
  }

  @Override
  public void autonomousPeriodic() {
    driveWithJoystick(false);
    swerve.updateOdometry();
  }

  @Override
  public void teleopInit() {
    swerve.setBrakeMode();
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(true);
  }
  
  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed =
        -xspeedLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getLeftY(), 0.02))
            * Constants.DriveConstants.MAX_SPEED;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    double ySpeed =
        -yspeedLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getLeftX(), 0.02))
            * Constants.DriveConstants.MAX_SPEED;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot =
        -rotLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getRightX(), 0.02))
            * Constants.DriveConstants.MAX_ANGULAR_SPEED;

    // While the A-button is pressed, overwrite some of the driving values with the output of our limelight methods
    if(DRIVE_CONTROLLER.getAButton())
    {
        rot = limelight.limelight_aim_proportional();

        // Translational value for alignment, ySpeed isn't needed
        xSpeed = limelight.limelight_range_proportional();

        // While using Limelight, turn off field-relative driving
        fieldRelative = false;
    }

    swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
  }

   /** This function is called once when the robot is disabled. */
   @Override
   public void disabledInit() {
    swerve.setCoastMode();
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

