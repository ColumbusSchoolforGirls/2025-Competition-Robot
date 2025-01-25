package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.ControllerConstants;


public class CoralSystem {

    // public SparkPIDController coralPidController;
    //https://docs.wpilib.org/en/2020/docs/software/wpilib-overview/3rd-party-libraries.html

    public final WPI_TalonSRX shootMotor = new WPI_TalonSRX(CoralConstants.SHOOT_ID);
    public final SparkMax elevatorMotor = new SparkMax(CoralConstants.ELEVATOR_ID, MotorType.kBrushless);

    public static final XboxController aux = new XboxController(ControllerConstants.AUX_ID);

    private final RelativeEncoder elevatorEncoder = elevatorMotor.getEncoder();

    private double targetHeight;
    private double difference;
    double startTime; //TODO: why is startTime not private?

    /** Returns the current height of elevator in inches.*/
    public double getHeight() {
        return elevatorEncoder.getPosition() * CoralConstants.TICKS_TO_INCHES;
    }

    public void setShootMotorCoast() {
        shootMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void updateTargetHeight() {

        if (aux.getAButtonPressed()) {
            targetHeight = CoralConstants.L2_HEIGHT;
        } else if (aux.getBButtonPressed()) {
            targetHeight = CoralConstants.L3_HEIGHT;
        } else if (aux.getYButtonPressed()) {
            targetHeight = CoralConstants.L4_HEIGHT;
        }
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

    public void shoot() {
        if (aux.getXButtonPressed()) {
            shootMotor.set(CoralConstants.SHOOT_MOTOR_SPEED);
        }
    }

    //TODO: implement into auto

    // TODO: add limelight stuff

    // set states

}
