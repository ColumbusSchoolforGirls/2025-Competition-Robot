package frc.robot.subsystems;

import java.util.IllegalFormatPrecisionException;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

public class CoralSystem {

    // public SparkPIDController coralPidController;
    //https://docs.wpilib.org/en/2020/docs/software/wpilib-overview/3rd-party-libraries.html

    double startTime;

    private double difference;

    public WPI_TalonSRX shootMotor = new WPI_TalonSRX(Constants.CoralConstants.SHOOT_ID);

    private double targetHeight;

    public double getHeight() {
        return 0; //encoder things for height value //return height
    }

    public void updateTargetHeight() {

        if (aux.getAButtonPressed()) {
            targetHeight = Constants.CoralConstants.L2_HEIGHT;

        } else if (aux.getBButtonPressed()) {
            targetHeight = Constants.CoralConstants.L3_HEIGHT;

        } else if (aux.getYButtonPressed()) {
            targetHeight = Constants.CoralConstants.L4_HEIGHT;

        }

    }

    public boolean elevatorComplete() {
        difference = (targetHeight - getHeight());

        return Math.abs(difference) < Constants.CoralConstants.ELEVATOR_TOLERANCE;
    }

    public void elevator() {
        difference = (targetHeight - getHeight());

        if (difference < Constants.CoralConstants.ELEVATOR_TOLERANCE) {
            // elevatorSpeed = 0;
        } else if (difference > 0) {
            // elevatorSpeed = 0-1
        } else if (difference < 0) {
            // elevatorSpeed = some -1-0
        }

    }

    public void shoot() {
        if (aux.getXButtonPressed()) {
            shootMotor.set(Constants.CoralConstants.SHOOT_MOTOR_SPEED);
        }

    }

    // TODO: add coral motor and elevator motors

    public static XboxController aux = new XboxController(1); // 1 is the aux controller

    // TODO: add coral system encoders?

    // TODO: add limelight stuff

    // set states

}
