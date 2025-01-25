package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ControllerConstants;

public class Climber {

    WPI_TalonSRX climbMotor = new WPI_TalonSRX(ClimbConstants.CLIMBER_ID);

    public static XboxController aux  = new XboxController(ControllerConstants.AUX_ID);
    public static XboxController reverseClimbController = new XboxController(ControllerConstants.DRIVING_ID);

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
