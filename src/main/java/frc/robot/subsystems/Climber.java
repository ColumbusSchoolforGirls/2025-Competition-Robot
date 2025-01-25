package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ControllerConstants;


public class Climber {

    private final WPI_TalonSRX climbMotor = new WPI_TalonSRX(ClimbConstants.CLIMBER_ID);

    public static final XboxController aux  = new XboxController(ControllerConstants.AUX_ID);
    public static final XboxController reverseClimbController = new XboxController(ControllerConstants.DRIVING_ID);

    public void setClimb() {
        climbMotor.set(0);
    }

    public void setCoast() {
        climbMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void climb() {
        if (aux.getLeftTriggerAxis() > ControllerConstants.TRIGGER_DEADZONE) {
            climbMotor.set(ClimbConstants.CLIMB_SPEED);
        } else if (reverseClimbController.getRightTriggerAxis() > ControllerConstants.TRIGGER_DEADZONE) {
            climbMotor.set(-ClimbConstants.CLIMB_SPEED);
        } else {
            climbMotor.set(0);
        }
    }
}
