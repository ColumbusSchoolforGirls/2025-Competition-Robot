package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj.XboxController;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ControllerConstants;
import static frc.robot.Constants.ControllerConstants.AUX;


public class Climber {

    private final WPI_TalonSRX climbMotor = new WPI_TalonSRX(ClimbConstants.CLIMBER_ID);

    public static final XboxController reverseClimbController = new XboxController(ControllerConstants.REVERSE_CLIMBER_AUX_PORT);

    public void setClimbZero() {
        climbMotor.set(0); //huh? Elaborate?- Yinan & Lillie
    }

    public void setCoast() {
        climbMotor.setNeutralMode(NeutralMode.Coast);
    }

    public void setClimb(double power) {
        climbMotor.set(-power);
    }

    public void climb() {
        if (AUX.getLeftTriggerAxis() > ControllerConstants.TRIGGER_DEADZONE) {
            setClimb(ClimbConstants.CLIMB_SPEED);
        } else if (reverseClimbController.getRightTriggerAxis() > ControllerConstants.TRIGGER_DEADZONE) {
            setClimb(-ClimbConstants.CLIMB_SPEED);
        } else {
            climbMotor.set(0);
        }
    }
}
