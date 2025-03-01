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

        // The MAXSwerve module can be configured with one of three pinion gears: 12T,
        // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
        // more teeth will result in a robot that drives faster).
        public static final int DrivingMotorPinionTeeth = 14;

        public static final double FreeSpeedRpm = 5676;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double DrivingMotorFreeSpeedRps = FreeSpeedRpm / 60;
        public static final double WheelDiameterMeters = 0.0762;
        public static final double WheelCircumferenceMeters = WheelDiameterMeters * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double DrivingMotorReduction = (45.0 * 22) / (DrivingMotorPinionTeeth * 15);
        public static final double DriveWheelFreeSpeedRps = (DrivingMotorFreeSpeedRps * WheelCircumferenceMeters)
            / DrivingMotorReduction;

        //Encoder calculations
        public static final double GEAR_RATIO = 8.14;
        public static final double WHEEL_DIAMETER = 4 / Math.PI; //inches

        public static final double drivingFactor = Constants.SwerveConstants.WheelDiameterMeters * Math.PI
        / Constants.SwerveConstants.DrivingMotorReduction;
        public static final double steeringFactor = 150/7;
        public static final double turningFactor = (2 * Math.PI)/steeringFactor;
        public static final double drivingVelocityFeedForward = 1 / Constants.SwerveConstants.DriveWheelFreeSpeedRps;

    }

    public static final class DriveConstants {  

        public static final double MAX_SPEED = 3.0; // 3 meters per second
        public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second
        
        public static final double TRANSLATION_2D_OFFSET = 0.3048; // 12 inches to meters

        public static final int FL_DRIVE_ID = 2;
        public static final int FL_TURN_ID = 3;
        public static final int FL_DIO = 2;
        public static final double FL_CHASSIS_ANGULAR_OFFSET = 0.802*Math.PI*2; //TODO: need to check after Chole fix (in radians)

        public static final int FR_DRIVE_ID = 4;
        public static final int FR_TURN_ID = 5;
        public static final int FR_DIO = 3;
        public static final double FR_CHASSIS_ANGULAR_OFFSET = 0.294*Math.PI*2;

        public static final int BL_DRIVE_ID = 6;
        public static final int BL_TURN_ID = 7;
        public static final int BL_DIO = 1;
        public static final double BL_CHASSIS_ANGULAR_OFFSET = 0.918*Math.PI*2; //TODO: check later 

        public static final int BR_DRIVE_ID = 8;
        public static final int BR_TURN_ID = 9;
        public static final int BR_DIO = 0;
        public static final double BR_CHASSIS_ANGULAR_OFFSET = 0.950*Math.PI*2;

        public static final int TURN_TOLERANCE = 2; //TODO: need to change when testing turning
        public static final double DISTANCE_TOLERANCE = 0.1; //TODO: need to change when testing distance

        //Gyro Reversed
        public static final boolean GyroReversed = false; //TODO: is this always false?????
    }   

    public static final class AutoConstants {
        public static final double SIDE_DISTANCE = 0; //TODO: change distance
        public static final double MIDDLE_DISTANCE = 0; //TODO: change distance
        public static final double LEAVE_ONLY_DISTANCE = 0; //TODO: change distance
    }

    public static final class CoralConstants { //TODO: need to change all of the values!

        public static final int ELEVATOR_TOLERANCE = 1; //TODO: Need to change when testing elevator
        
        public static final double L2_HEIGHT = 24.5;
        public static final double L3_HEIGHT = 34.5;
        public static final double L4_HEIGHT = 44.5;

        public static final int SHOOT_ID = 10; // TODO: change id
        public static final int ELEVATOR_ID = 14;

        public static final double SHOOT_MOTOR_SPEED = 0.5;
        public static final double ELEVATOR_SPEED = 0.5;

        public static final double TICKS_TO_INCHES = 0.02;

        public static final double SHOOT_TIME = 1.5; //TODO: change time
    }

    public static final class ClimbConstants {

        public static final int CLIMBER_ID = 11; //TODO: change id

        public static final double CLIMB_SPEED = 0.5;
    }

    public static final class ControllerConstants {

        public static final XboxController DRIVE_CONTROLLER = new XboxController(0);
        public static final XboxController AUX = new XboxController(1);

        public static final int REVERSE_CLIMBER_AUX_PORT = 2;

        public static final double TRIGGER_DEADZONE = 0.1;
    }

    public static final class AlgaeConstants {

        public static final int ALGAE_UPPER_ID = 12;
        public static final int ALGAE_LOWER_ID = 13;

        public static final double ALGAE_SPEED = 0.5;

        public static final double ALGAE_TIME = 2; //TODO: change time

    }
}
