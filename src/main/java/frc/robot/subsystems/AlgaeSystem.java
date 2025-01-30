package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Timer;

import frc.robot.Constants.ControllerConstants;
import static frc.robot.Constants.ControllerConstants.AUX;
import frc.robot.Constants.AlgaeConstants;


public class AlgaeSystem {
    private final WPI_TalonSRX upperMotor = new WPI_TalonSRX(AlgaeConstants.ALGAE_UPPER_ID);
    private final WPI_TalonSRX lowerMotor = new WPI_TalonSRX(AlgaeConstants.ALGAE_LOWER_ID);

    private double startTime;
    private boolean algaeLowerArmUp = true;
    private boolean algaeUpperArmUp = true;

    public void stopAlgaeArms() { // TODO: probably unneeded
        upperMotor.set(0);
        lowerMotor.set(0);
    }

    public void setCoast() {
        upperMotor.setNeutralMode(NeutralMode.Coast);
        lowerMotor.setNeutralMode(NeutralMode.Coast);
    }

    private void setAlgaeArm(WPI_TalonSRX motor, int direction) {
        startTime = Timer.getFPGATimestamp();
            if (Timer.getFPGATimestamp() - startTime < AlgaeConstants.ALGAE_TIME) {
                motor.set(direction * AlgaeConstants.ALGAE_SPEED);
            } else {
                motor.set(0);
            }
    }

    public void algae() {
        // Set the lower arm to go up
        if ((AUX.getLeftTriggerAxis() > ControllerConstants.TRIGGER_DEADZONE) && !algaeLowerArmUp) {
            setAlgaeArm(lowerMotor, 1);
            algaeLowerArmUp = true;
        }
        // Set the lower arm to go down
        else if (AUX.getLeftBumperPressed() && algaeUpperArmUp) {
            setAlgaeArm(lowerMotor, -1);
            algaeLowerArmUp = false;
        }

        // Set the upper arm to go up
        else if ((AUX.getRightTriggerAxis() > ControllerConstants.TRIGGER_DEADZONE) && !algaeUpperArmUp) {
            setAlgaeArm(upperMotor, 1);
            algaeUpperArmUp = true;
        }
        // Set the upper arm to go down
        else if (AUX.getRightBumperPressed() && algaeUpperArmUp) {
            setAlgaeArm(upperMotor, -1);
            algaeUpperArmUp = false;
        }
    } // I hate this. PLEASE clean it up I CAN'T
}
