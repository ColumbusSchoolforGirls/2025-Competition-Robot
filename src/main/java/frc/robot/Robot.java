// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ControllerConstants.DRIVE_CONTROLLER; // Noah HATES this, but says it's not a bad use . . .

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.CoralSystem;
import frc.robot.subsystems.Climber;

public class Robot extends TimedRobot {
  private final Limelight limelight = new Limelight();
  private final CoralSystem coralSystem = new CoralSystem();
  private final Climber climber = new Climber();
  private final Drivetrain swerve = new Drivetrain(limelight);

  private final AutoPaths autoPaths = new AutoPaths();
  AutoStep[] autoActions = {}; // TODO: add autoActions = buildPath() in autoInit
  int state;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  @Override
  public void robotInit() {

    // // Starts recording to data log
    // DataLogManager.start(); // TODO: maybe remove one, because logs double

    // // Record both DS control and joystick data
    // DriverStation.startDataLog(DataLogManager.getLog());

    swerve.driveInit();
    autoPaths.autoShuffleboardStartup();
  }

  @Override
  public void robotPeriodic() {
    // coralSystem.elevator();
    // climber.climb();
    // limelight.updateLimelight();
    swerve.updateOdometry();
    swerve.periodic();

  }

  @Override
  public void autonomousInit() {
    swerve.setBrakeMode();
  }

  public void goToNextState() {
    state++;

    if (state >= autoActions.length) {
      return;
    }

    AutoStep currentAction = autoActions[state];

    if (currentAction.getAction() == AutoAction.TURN) {
      // float initialTurnAngle = autoPaths.getInitialTurnAngle();
      swerve.startTurn(currentAction.getValue());

    } else if (currentAction.getAction() == AutoAction.DRIVE) {
      swerve.startDrive(currentAction.getValue()); // value = autopaths.getInitialDriveDistance()

    } else if (currentAction.getAction() == AutoAction.GO_TO_REEF_AND_RAISE_ELEVATOR) {
      coralSystem.setAutoTargetHeight(autoPaths.getAutoTargetHeight());
      swerve.startDrive(currentAction.getValue()); // autoPaths.getDriveToReefDistance()

    } else if (currentAction.getAction() == AutoAction.SHOOT_CORAL) {
      coralSystem.autoShoot();

    } else if (currentAction.getAction() == AutoAction.ADDITIONAL_DRIVE_ACTIONS) {
      coralSystem.setAutoTargetHeight(0);
      // swerve.startTurn(autoPaths.getInitialEndAutoTargetAngle());
      autoPaths.getDriveDistance();
      // swerve.startDrive(autoPaths.getInitialEndAutoTargetDistance());
      
    } else if (currentAction.getAction() == AutoAction.STOP) {
      // TODO: add stop all
    }
    // TODO: add all the other actions
  }

  @Override
  public void autonomousPeriodic() {

    if (autoPaths.currentAutoAction == AutoAction.INITIAL_DRIVE) {
      if (swerve.driveComplete()) {
        goToNextState();
      }

    } else if (autoPaths.currentAutoAction == AutoAction.TURN_TOWARD_REEF) {
      if (swerve.turnComplete()) {
        goToNextState();
      }
    
    } else if (autoPaths.currentAutoAction == AutoAction.GO_TO_REEF_AND_RAISE_ELEVATOR) {
      if (coralSystem.elevatorComplete() && swerve.driveComplete()) {
        goToNextState();
      }
      // TODO: run AprilTag auto alignment

    } else if (autoPaths.currentAutoAction == AutoAction.SHOOT_CORAL) {
      if (coralSystem.autoShootComplete()) {
        goToNextState();
      }

    } else if (autoPaths.currentAutoAction == AutoAction.ADDITIONAL_DRIVE_ACTIONS) {
      if (swerve.turnComplete() && swerve.driveComplete()) { 
        goToNextState();
      } 

    } else {
      autoPaths.currentAutoAction = AutoAction.STOP; 
    }
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
    final double xSpeed = -xspeedLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getLeftY(), 0.1))
        * Constants.DriveConstants.MAX_SPEED;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final double ySpeed = -yspeedLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getLeftX(), 0.1))
        * Constants.DriveConstants.MAX_SPEED;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final double rot = -rotLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getRightX(), 0.1))
        * Constants.DriveConstants.MAX_ANGULAR_SPEED;

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
