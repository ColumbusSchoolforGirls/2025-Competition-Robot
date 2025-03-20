package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;
import frc.robot.Constants.CoralConstants;
import static frc.robot.Constants.ControllerConstants.AUX;

public class CoralSystem {

    // public SparkPIDController coralPidController;
    // https://docs.wpilib.org/en/2020/docs/software/wpilib-overview/3rd-party-libraries.html

    public final WPI_TalonSRX shootMotor = new WPI_TalonSRX(CoralConstants.SHOOT_ID);
    public final SparkMax elevatorMotor = new SparkMax(CoralConstants.ELEVATOR_ID, MotorType.kBrushless);

    private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

    private static DigitalInput elevatorLimitSwitch = new DigitalInput(CoralConstants.ELEVATOR_LIMIT_SWITCH_CHANNEL); 

    double startShootTime;

    public Limelight limelight;

    public CoralSystem(Limelight limelight) {
        this.limelight = limelight;
        this.elevatorMotor.configure(Configs.MAXSwerveModule.elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }

    /** Returns the current height of elevator in inches. */
    private double getHeight() {
        return -(elevatorEncoder.getPosition() * CoralConstants.ELEVATOR_INCHES_PER_ROTATION)*2; //spinning opposite direction 
    }

    public void setElevator(double power) { //inverts
        elevatorMotor.set(-power);
        SmartDashboard.putNumber("elevator power", power);
    }

     /**
     * Sets the elevator encoder tick count to 0. Only for use on robot-init when
     * the elevator has been reset to bottom.
     */
    public void resetElevatorEncoders() {
        elevatorEncoder.setPosition(0);
    }

    public void setShootMotor() {
        shootMotor.setNeutralMode(NeutralMode.Coast);
        shootMotor.setInverted(true);
    }

    public double controllerInputTargetHeight() {
        if (AUX.getAButton()) {
            return CoralConstants.L2_HEIGHT;
        } else if (AUX.getBButton()) {
            return CoralConstants.L3_HEIGHT;
        } else if (AUX.getYButton()) {
            return CoralConstants.L4_HEIGHT;
        } else {
            return CoralConstants.L2_HEIGHT;
        }
    }

    private double getDifference() {
        return controllerInputTargetHeight()*2 - getHeight();
    }


    // public void setAutoTargetHeight(double targetHeight) { //TODO: change this?
    //     this.targetHeight = targetHeight;
    // }

    public boolean elevatorComplete() {
        return Math.abs(getDifference()) < CoralConstants.ELEVATOR_TOLERANCE;
    }

    private boolean isElevatorLimitReached() {
        return elevatorLimitSwitch.get();
    }

    public void stopElevatorWithLimitSwitch() {
        if (isElevatorLimitReached()) {
            setElevator(0);
        }
    }

    public void resetEncodersWithLimitSwitch() {
        if (isElevatorLimitReached()) {
            resetElevatorEncoders();
        }
    }

    public void elevator(double normalElevatorSpeed, double downwardElevatorSpeed) {

        double scaledElevatorSpeedSlope = CoralConstants.ELEVATOR_SCALE_FACTOR * getDifference();

        resetEncodersWithLimitSwitch();
        
        if (AUX.getRightBumperButton()) {
            driveElevator(normalElevatorSpeed);

        } else {

            if (getDifference() < -CoralConstants.ELEVATOR_TOLERANCE) {
                setElevator(downwardElevatorSpeed);

                stopElevatorWithLimitSwitch();

            } else { //trapezoid drive

                if (Math.abs(getDifference()) < CoralConstants.ELEVATOR_TOLERANCE) {
                    setElevator(0);
                } else if (getDifference() < CoralConstants.NORMAL_ELEVATOR_SPEED_DIFFERENCE) {
                    setElevator((scaledElevatorSpeedSlope * normalElevatorSpeed) + CoralConstants.MINIMUM_ELEVATOR_SPEED_NEEDED);
                } else {
                    setElevator(normalElevatorSpeed);
                }


            }
        }
        

    }

    public void driveElevator(double normalElevatorSpeed) {
        double elevatorSpeed = -AUX.getLeftY(); //it is inverted
;
        if (Math.abs(elevatorSpeed) > CoralConstants.AUX_DEADZONE) {
            setElevator(elevatorSpeed * normalElevatorSpeed);
        } else if (Math.abs(elevatorSpeed) < CoralConstants.AUX_DEADZONE) {
            setElevator(0);
        }
  
    }

    // public void driveElevator(double normalElevatorSpeed) {

    // }

    // TODO: Change this to time-based if needed (driver visibility)
    public void shoot() {

        if (AUX.getRightTriggerAxis() > CoralConstants.AUX_DEADZONE) {
            shootMotor.set(CoralConstants.SHOOT_MOTOR_SPEED);
        } else {
            shootMotor.set(0);
        }
    }

    public void autoShoot() {

        shootMotor.set(CoralConstants.SHOOT_MOTOR_SPEED);
        startShootTime = Timer.getFPGATimestamp();
    }

    public boolean autoShootComplete() {

        if (Timer.getFPGATimestamp() - startShootTime > CoralConstants.SHOOT_TIME) {
            shootMotor.set(0);
            return true;
        }
        return false;
    }
}
