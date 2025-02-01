package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.XboxController;


public class Constants {

    public static final class SwerveConstants {
        //feed forward constants //TODO: change values if we use feedforward
        public static final int STATIC_GAIN_DRIVE = 1;
        public static final int VELOCITY_GAIN_DRIVE = 3;
        public static final int STATIC_GAIN_TURN = 1;
        public static final double VELOCITY_GAIN_TURN = 0.5;

        //Encoder calculations
        public static final double GEAR_RATIO = 8.14;
        public static final int WHEEL_CIRCUMFERENCE = 4;
    }

    public static final class DriveConstants {  

        public static final double MAX_SPEED = 3.0; // 3 meters per second
        public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second
        
        public static final double TRANSLATION_2D_OFFSET = 0.381;
        
        // Chassis configuration
        public static final double kTrackWidth = 0.381; // (meters) //TODO: change units
        // Distance between centers of right and left wheels on robot
        public static final double kWheelBase = 0.381; // (meters) //TODO: change units
        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

        public static final int FL_DRIVE_ID = 2;
        public static final int FL_TURN_ID = 3;
        public static final int FL_DIO = 1;
        public static final int FL_CHASSIS_ANGULAR_OFFSET = 2; // probably some pi value

        public static final int FR_DRIVE_ID = 4;
        public static final int FR_TURN_ID = 5;
        public static final int FR_DIO = 1;
        public static final int FR_CHASSIS_ANGULAR_OFFSET = 2;

        public static final int BL_DRIVE_ID = 6;
        public static final int BL_TURN_ID = 7;
        public static final int BL_DIO = 1;
        public static final int BL_CHASSIS_ANGULAR_OFFSET = 2;

        public static final int BR_DRIVE_ID = 8;
        public static final int BR_TURN_ID = 9;
        public static final int BR_DIO = 1;
        public static final int BR_CHASSIS_ANGULAR_OFFSET = 2;

        //Gyro Reversed
        public static final boolean kGyroReversed = false; //TODO: is this always false?????
    }   

    public static final class CoralConstants { //TODO: need to change all of the values!

        public static final int ELEVATOR_TOLERANCE = 1; //TODO: Need to change when testing elevator
        
        public static final double L2_HEIGHT = 24.5;
        public static final double L3_HEIGHT = 34.5;
        public static final double L4_HEIGHT = 44.5;

        public static final int SHOOT_ID = 10; 
        public static final int ELEVATOR_ID = 9;

        public static final double SHOOT_MOTOR_SPEED = 0.5;
        public static final double ELEVATOR_SPEED = 0.5;

        public static final double TICKS_TO_INCHES = 0.02;
    }

    public static final class ClimbConstants {

        public static final int CLIMBER_ID = 1; //TODO: change id

        public static final double CLIMB_SPEED = 0.5;
    }

    public static final class ControllerConstants {

        public static final XboxController DRIVE_CONTROLLER = new XboxController(0);
        public static final XboxController AUX = new XboxController(1);

        public static final int REVERSE_CLIMBER_ID = 2;

        public static final double TRIGGER_DEADZONE = 0.1;
    }

    public static final class AlgaeConstants {

        public static final int ALGAE_UPPER_ID = 3;
        public static final int ALGAE_LOWER_ID = 4;

        public static final double ALGAE_SPEED = 0.5;

        public static final double ALGAE_TIME = 2; //TODO: change time

    }
}
