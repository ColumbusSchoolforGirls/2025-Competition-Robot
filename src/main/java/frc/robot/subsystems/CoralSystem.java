package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.CoralConstants;
import static frc.robot.Constants.ControllerConstants.AUX;


public class CoralSystem {

    // public SparkPIDController coralPidController;
    //https://docs.wpilib.org/en/2020/docs/software/wpilib-overview/3rd-party-libraries.html

    public final WPI_TalonSRX shootMotor = new WPI_TalonSRX(CoralConstants.SHOOT_ID);
    public final SparkMax elevatorMotor = new SparkMax(CoralConstants.ELEVATOR_ID, MotorType.kBrushless);

    private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

    private double targetHeight;
    private double difference;

    /** Returns the current height of elevator in inches.*/
    private double getHeight() {
        return elevatorEncoder.getPosition() * CoralConstants.TICKS_TO_INCHES;
    }

    public void setShootMotorCoast() {
        shootMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void controllerInputTargetHeight() {

        if (AUX.getAButtonPressed()) {
            targetHeight = CoralConstants.L2_HEIGHT;
        } else if (AUX.getBButtonPressed()) {
            targetHeight = CoralConstants.L3_HEIGHT;
        } else if (AUX.getYButtonPressed()) {
            targetHeight = CoralConstants.L4_HEIGHT;
        }
    }

    public void setAutoTargetHeight(double targetHeight){
        this.targetHeight = targetHeight;
        
    }


    public boolean elevatorComplete() {
        difference = (targetHeight - getHeight());

        return Math.abs(difference) < CoralConstants.ELEVATOR_TOLERANCE;
    }

    public void elevator() {
        difference = (targetHeight - getHeight());

        if (difference < CoralConstants.ELEVATOR_TOLERANCE) {
            elevatorMotor.set(0);
        } else if (difference > 0) {
            elevatorMotor.set(CoralConstants.ELEVATOR_SPEED);
        } else if (difference < 0) {
            elevatorMotor.set(-CoralConstants.ELEVATOR_SPEED);
        }
    }

    // TODO: Change this to time-based if needed (driver visibility)
    public void shoot() {
        if (AUX.getXButton()) {
            shootMotor.set(CoralConstants.SHOOT_MOTOR_SPEED);
        } else {
            shootMotor.set(0);
        }
    }

    //TODO: implement into auto

    // TODO: add limelight stuff
}
