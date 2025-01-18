// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants; 

public class Robot extends TimedRobot {
  private final XboxController controller = new XboxController(0);
  private final Drivetrain swerve = new Drivetrain();

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  @Override
  public void robotInit(){

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

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final double xSpeed =
        -xspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftY(), 0.02))
            * Constants.DriveConstants.MAX_SPEED;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final double ySpeed =
        -yspeedLimiter.calculate(MathUtil.applyDeadband(controller.getLeftX(), 0.02))
            * Constants.DriveConstants.MAX_SPEED;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final double rot =
        -rotLimiter.calculate(MathUtil.applyDeadband(controller.getRightX(), 0.02))
            * Constants.DriveConstants.MAX_ANGULAR_SPEED;

    swerve.drive(xSpeed, ySpeed, rot, fieldRelative, getPeriod());
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
