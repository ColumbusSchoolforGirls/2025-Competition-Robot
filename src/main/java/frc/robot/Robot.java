// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ControllerConstants.AUX;
import static frc.robot.Constants.ControllerConstants.DRIVE_CONTROLLER; // Noah HATES this, but says it's not a bad use . . .

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.CoralSystem;
import frc.robot.subsystems.Climber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  //private final Limelight limelightCage = new Limelight("limelight-cage"); //TODO: make this camera onyl show up with a button 
  private final Limelight limelightCoral = new Limelight("limelight-coral");

  private final CoralSystem coralSystem = new CoralSystem(limelightCoral);
  private final Climber climber = new Climber();
  private final Drivetrain swerve = new Drivetrain(limelightCoral);

  private final AutoPaths autoPaths = new AutoPaths();
  ArrayList<AutoStep> autoActions = new ArrayList<>();

  int state;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  private boolean fieldRelative = false;

  @Override
  public void robotInit() {
    // // Starts recording to data log
    // DataLogManager.start(); // TODO: maybe remove one, because logs double

    // // Record both DS control and joystick data
    // DriverStation.startDataLog(DataLogManager.getLog());
    swerve.driveInit();
    autoPaths.autoShuffleboardStartup();
    coralSystem.resetElevatorEncoders();
  }

  @Override
  public void robotPeriodic() {
    // coralSystem.elevator();
    // climber.climb();
    limelightCoral.updateLimelight();
    swerve.updateOdometry();
    swerve.periodic();
    swerve.updateSmartDashboard();

    if (AUX.getXButtonPressed()) {
      fieldRelative = false;
    } // TODO:Change Button if needed AND move to periodic
  }

  @Override
  public void autonomousInit() {
    swerve.setBrakeMode();
    swerve.resetTurnEncoders();
    autoActions = autoPaths.buildPath();
    System.out.println("Auto path: ");
    autoActions.forEach(action -> System.out.println(action));
    state = -1;
    goToNextState();
  }

  public void goToNextState() {
    state++;
    if (state >= autoActions.size()) {
      System.out.println("End of auto path :)");
      return;
    }

    AutoStep currentAction = new AutoStep(AutoAction.STOP);
    if (state < autoActions.size()) {
      currentAction = autoActions.get(state);
    } //TODO: make into funciton later
    System.out.println("New action: " + currentAction);

    switch (currentAction.getAction()) {
      case DRIVE:
        swerve.startDrive(currentAction.getValue()); // value = autopaths.getInitialDriveDistance()
        break;
      case TURN:
        // float initialTurnAngle = autoPaths.getInitialTurnAngle();
        swerve.startTurn(currentAction.getValue());
        break;
      case ALIGN:
        swerve.autoAlignLimelight(getPeriod());
        break;
      case DRIVE_AND_ELEVATOR:
        //coralSystem.setAutoTargetHeight(currentAction.getValue()); // or autoPaths.getAutoTargetHeight() BUT this is better for abstraction
        // swerve.startTurn(autoPaths.getInitialEndAutoTargetAngle());
        autoPaths.getDriveDistance();
        // swerve.startDrive(autoPaths.getInitialEndAutoTargetDistance());
        break;
      case SHOOT:
        coralSystem.autoShoot();
        break;
      case STOP:
        break;
      default:
        break;
    }
  }

  @Override
  public void autonomousPeriodic() {
    coralSystem.stopElevatorWithLimitSwitch();
    coralSystem.resetEncodersWithLimitSwitch();

    AutoStep currentAction = new AutoStep(AutoAction.STOP);
    if (state < autoActions.size()) {
      currentAction = autoActions.get(state);
    }
    SmartDashboard.putString("auto action", currentAction.toString());
    SmartDashboard.putNumber("auto state", state);
    switch (currentAction.getAction()) {
      case DRIVE:
        if (swerve.driveComplete()) {
          goToNextState();
        }
        swerve.autoDrive(getPeriod());
        break;
      case TURN:
        if (swerve.turnComplete()) {
          goToNextState();
        }
        break;
      case ALIGN:
        // TODO: implement AprilTag alignment
        break;
      case DRIVE_AND_ELEVATOR:
        if (coralSystem.elevatorComplete() && swerve.driveComplete()) {
          goToNextState();
        }
        break;
      case SHOOT:
        if (coralSystem.autoShootComplete()) {
          goToNextState();
        }
        break;
      case STOP:
      swerve.stop(getPeriod());
        break;
      default:
        break;
    }
  }

  @Override
  public void teleopInit() {
    swerve.setBrakeMode();
    swerve.resetTurnEncoders();
    coralSystem.resetElevatorEncoders();
    coralSystem.setShootMotor();
    climber.setClimbZero();
    climber.setCoast();
  }

  public void isFieldRelative() {
    
    if (DRIVE_CONTROLLER.getYButtonPressed()) {
      fieldRelative = !fieldRelative;
      SmartDashboard.putBoolean("Field Relative?", false);
    } 

  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(false);
    coralSystem.shoot();
    swerve.driverResetTurnEncoders();
    climber.climb();
    coralSystem.elevator(0.7, -0.1);
    swerve.teleopAutoAlign(getPeriod());
    isFieldRelative(); //TODO: fix


  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final double xSpeed = -xspeedLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getLeftY(), 0.1))
        * Constants.DriveConstants.MAX_SPEED;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default. // nah positive
    // now
    final double ySpeed = yspeedLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getLeftX(), 0.1))
        * Constants.DriveConstants.MAX_SPEED;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default. // uh no, positive now
    final double rot = rotLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getRightX(), 0.1))
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
    SmartDashboard.putBoolean("A", false);
    SmartDashboard.putBoolean("B", false);
    SmartDashboard.putBoolean("X", false);
    SmartDashboard.putBoolean("Y", false);
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
    if (AUX.getAButton()) {
      SmartDashboard.putBoolean("A", true);
    } else {
      SmartDashboard.putBoolean("A", false);
    }
    if (AUX.getBButton()) {
      SmartDashboard.putBoolean("B", true);
    } else {
      SmartDashboard.putBoolean("B", false);
    }
    if (AUX.getXButton()) {
      SmartDashboard.putBoolean("X", true);
    } else {
      SmartDashboard.putBoolean("X", false);
    }
    if (AUX.getYButton()) {
      SmartDashboard.putBoolean("Y", true);
    } else {
      SmartDashboard.putBoolean("Y", false);
    }
  }
}
