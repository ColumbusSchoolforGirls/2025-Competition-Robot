// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.ControllerConstants.AUX;
import static frc.robot.Constants.ControllerConstants.DRIVE_CONTROLLER; // Noah HATES this, but says it's not a bad use . . .

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CoralSystem;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;

public class Robot extends TimedRobot {
  //private final Limelight limelightCage = new Limelight("limelight-cage"); //TODO: make this camera onyl show up with a button 
  private final Limelight limelightCoral = new Limelight("limelight-coral");

  private final CoralSystem coralSystem = new CoralSystem(limelightCoral);
  private final Climber climber = new Climber();
  private final Drivetrain swerve = new Drivetrain(limelightCoral);

  private final AutoPaths autoPaths = new AutoPaths();
  ArrayList<AutoStep> autoActions = new ArrayList<>();

  int state;
  double startWaitTime;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);
  // private boolean fieldRelative = false;

  @Override
  public void robotInit() {
    // // Starts recording to data log
    // DataLogManager.start(); // TODO: maybe remove one, because logs double

    // // Record both DS control and joystick data
    // DriverStation.startDataLog(DataLogManager.getLog());
    swerve.driveInit();
    autoPaths.autoShuffleboardStartup();
    coralSystem.resetElevatorEncoders();
    coralSystem.setShootMotor();
    swerve.setAlignStateNotRunning();
  }

  @Override
  public void robotPeriodic() {
    limelightCoral.updateLimelight();
    swerve.updateOdometry();
    swerve.periodic();
    swerve.updateSmartDashboard();
    swerve.testShuffle();

    SmartDashboard.putBoolean("limit swtich", coralSystem.isElevatorLimitReached());


    // if (AUX.getXButtonPressed()) {
    //   fieldRelative = false;
    // }

    swerve.updateDistanceAndAngleValues();
  }

  @Override
  public void autonomousInit() {
    swerve.driveInit();
    limelightCoral.setRightBranchPipeline();
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
    }
    System.out.println("New action: " + currentAction);

    swerve.stop(getPeriod());

    switch (currentAction.getAction()) {
      case DRIVE:
        swerve.startDrive(currentAction.getValue(), currentAction.getTimeLimit());
        break;
      case TURN:
        swerve.startTurn(currentAction.getValue());
        break;
      case ALIGN:
        swerve.resetAlignState();
        break;
      case ELEVATOR:
        coralSystem.startElevatorAutoTimer();
        break;
      case SHOOT:
        coralSystem.autoShootStart();
        break;
      case WAIT:
        startWaitTime = Timer.getFPGATimestamp();
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

    // for testing
    SmartDashboard.putString("auto action", currentAction.toString());
    SmartDashboard.putNumber("auto state", state);

    switch (currentAction.getAction()) {
      case DRIVE:
        if (swerve.driveComplete()) {
          goToNextState();
        } else {
          swerve.autoDrive(getPeriod());
        }
        break;
      case TURN:
        if (swerve.turnComplete()) {
          goToNextState();
        } else {
          swerve.gyroTurn(getPeriod());
        }
        break;
      case ALIGN: //TODO: need to add left and right align
        if (swerve.isLimelightAligned() && swerve.isLimelightStrafeAligned()) {
          goToNextState();
        } else {
          swerve.autoAlignLimelight(getPeriod(), true);
        }
        break;
      case ELEVATOR:
        if (coralSystem.autoElevatorComplete(currentAction.getValue())) {
          goToNextState();
        } else {
          coralSystem.elevator(0.85, -0.25, true, currentAction.getValue());
        }
        break;
      case SHOOT:
        if (coralSystem.autoShootComplete()) {
          goToNextState();
        } else {
          coralSystem.autoShoot();
          coralSystem.elevator(0.85, -0.25, true, currentAction.getValue());
        }
        break;
      case WAIT: 
        if (Timer.getFPGATimestamp() - startWaitTime > 5) {
          startWaitTime = 0;
          System.out.println("Wait complete");
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
    climber.setClimbZero();
    climber.setCoast();
  }

  // public void isFieldRelative() {
    
  //   if (DRIVE_CONTROLLER.getYButtonPressed()) {
  //     fieldRelative = !fieldRelative;
  //     SmartDashboard.putBoolean("Field Relative?", false);
  //   } 

  // }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(false); 
    coralSystem.shoot();
    swerve.driverResetTurnEncoders();
    climber.climb();
    coralSystem.elevator(0.85, -0.25, false, 0);

    if (!swerve.isStepStopped()) {
      swerve.teleopAutoAlign();
      swerve.autoAlignLimelight(getPeriod(), false);
    }
    // isFieldRelative(); //TODO: fix
    swerve.resetAlignStateWithButton();

  }

  private void driveWithJoystick(boolean fieldRelative) {

    double xSpeed = -xspeedLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getLeftY(), ControllerConstants.JOYSTICK_DEADZONE))
    * Constants.DriveConstants.MAX_SPEED;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default. // nah positive
    // now
    double ySpeed = yspeedLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getLeftX(), ControllerConstants.JOYSTICK_DEADZONE))
        * Constants.DriveConstants.MAX_SPEED;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default. // uh no, positive now
    double rot = rotLimiter.calculate(MathUtil.applyDeadband(DRIVE_CONTROLLER.getRightX(), ControllerConstants.JOYSTICK_DEADZONE))
        * Constants.DriveConstants.MAX_ANGULAR_SPEED;

    if (DRIVE_CONTROLLER.getLeftBumperButton()) {
      xSpeed *= Constants.DriveConstants.CRAWL_SPEED; //if you click the left bumper you go at a slow scaled speed
      ySpeed *= Constants.DriveConstants.CRAWL_SPEED; 
      rot *= Constants.DriveConstants.CRAWL_SPEED; 
    }
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.


    //swerve.drive(0.8, 0, 0, fieldRelative, getPeriod()); FOR TESTING OFFSET
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
