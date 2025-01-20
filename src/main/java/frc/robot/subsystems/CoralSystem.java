package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;

public class CoralSystem {

    enum CoralAction {
        STOPPED_L1, ELEVATOR_L2, ELEVATOR_L3, ELEVATOR_L4, INTAKE, SHOOT
    }

    CoralAction state = CoralAction.STOPPED_L1;
    String CoralState = "STOPPED L1";

    //public SparkPIDController coralPidController;

    public static DigitalInput intakeLimitSwitch = new DigitalInput(Constants.CoralConstants.INTAKE_LIMIT_SWITCH_CHANNEL);
 
    double startTime;

    //TODO: add coral motor and elevator motors

    public static XboxController aux = new XboxController(1); // 1 is the aux controller

    //TODO: add coral system encoders?

    //TODO: add limelight stuff

    public boolean isCoralDetected() {
        return intakeLimitSwitch.get(); //might have to change if wired opposite again like last year
    }

    //set states

}
