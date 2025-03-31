package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class Constants {

    public static final class SwerveConstants {
        // feed forward constants //TODO: change values if we use feedforward
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

        // Encoder calculations
        public static final double GEAR_RATIO = 8.14;
        public static final double WHEEL_DIAMETER = 4 / Math.PI; // inches

        public static final double drivingFactor = Constants.SwerveConstants.WheelDiameterMeters * Math.PI
                / Constants.SwerveConstants.DrivingMotorReduction;
        public static final double steeringFactor = 150 / 7;
        public static final double turningFactor = (2 * Math.PI) / steeringFactor;
        public static final double drivingVelocityFeedForward = 1 / Constants.SwerveConstants.DriveWheelFreeSpeedRps;

        public static final double TURN_RESET_VELOCITY = 1; // TODO: Definetly need to measure
    }

    public static final class DriveConstants {

        public static final double BRANCH_DISTANCE = 0.33; //13 inches in meters

        public static final double ROTATION_TOLERANCE = 3;

        public static final double MAX_SPEED = 3.0; // 3 meters per second
        public static final double MAX_ANGULAR_SPEED = Math.PI * 2; // 1 rotation per second

        public static final double TRANSLATION_2D_OFFSET = 0.3048; // 12 inches to meters

        // NOTE: All angles must be thought of as clockwise negative (-) because of the gear-pulley system.
        // All angular offsets are in angles.
        public static final int FL_DRIVE_ID = 2;
        public static final int FL_TURN_ID = 3;
        public static final int FL_DIO = 8;
        public static final double FL_CHASSIS_ANGULAR_OFFSET = (52.7 * Math.PI) / 180;  //-25

        public static final int FR_DRIVE_ID = 4;
        public static final int FR_TURN_ID = 5;
        public static final int FR_DIO = 7;
        public static final double FR_CHASSIS_ANGULAR_OFFSET = (2.9 * Math.PI) / 180; //3.4

        public static final int BL_DRIVE_ID = 6;
        public static final int BL_TURN_ID = 7;
        public static final int BL_DIO = 9;
        public static final double BL_CHASSIS_ANGULAR_OFFSET = (85 * Math.PI) / 180; //85

        public static final int BR_DRIVE_ID = 8;
        public static final int BR_TURN_ID = 9;
        public static final int BR_DIO = 6;
        public static final double BR_CHASSIS_ANGULAR_OFFSET = -178 * Math.PI / 180;

        public static final int TURN_TOLERANCE = 2; // TODO: need to change when testing turning
        public static final double DISTANCE_TOLERANCE = 0.02; // TODO: need to change when testing distance

        // Gyro
        public static final boolean GyroReversed = false; // TODO: is this always false?????
        public static final double STALL_SPEED = 0.1; // TODO change speed if needed, greater?

        public static final double TX_TOLERANCE = 2.0;
        public static final double TY_TOLERANCE = 2.0;
        public static final double TARGET_TA_VALUE = 2.7;

        public static final double METERS_TO_INCHES = 39.37; //inches/meter

        public static final double CRAWL_SPEED = 0.5;

        public static final double MAX_DRIVE_AUTO_TIME = 8.0;

    }

    public static final class AutoConstants {
        public static final double INITIAL_DISTANCE = 58.5; // inches
        public static final double LEAVE_ONLY_DISTANCE = 1.0;//10.0/DriveConstants.METERS_TO_INCHES; // inches

        public static final double MAX_AUTO_SPEED = 2.5;
        public static final double MIN_AUTO_SPEED = 0.2;

    }

    public static final class CoralConstants {

        public static final double ELEVATOR_TOLERANCE = 0.25; // TODO: Need to change when testing elevator

        // Height in inches of coral node from default height (0 is L2).
        public static final double L2_HEIGHT = 0; // 31.875 from floor
        public static final double L3_HEIGHT = 15/2; //14.5 47.625 from floor //divided in half because of the elevator
        public static final double L4_HEIGHT = 37/2; // 72 from floor; //divided in half because of the elevator

        public static final int SHOOT_ID = 10;
        public static final int ELEVATOR_ID = 14;

        public static final double SHOOT_MOTOR_SPEED = 1.0;

        public static final double SHOOT_TIME = 1.0; // TODO: change time
        public static final double MAX_ELEVATOR_AUTO_TIME = 3;

        public static final double AUX_DEADZONE = 0.05;

        public static final double ELEVATOR_GEAR_RATIO = 20; // 20:1 gear ratio
        public static final double ELEVATOR_DRUM_CIRCUMFERENCE = 0.959 * Math.PI; //0.942 * Math.PI; // inches per rev
        public static final double ELEVATOR_INCHES_PER_ROTATION = ELEVATOR_DRUM_CIRCUMFERENCE / ELEVATOR_GEAR_RATIO; // inches per rev

        public static final double ELEVATOR_SCALE_FACTOR = 0.316667; //1/12 + 0.05 so max at 12+ difference - need to ajust if we change the NORMAL ELEVATOR SPEED DIFFERENCE // TODO: test
        public static final double MINIMUM_ELEVATOR_SPEED_NEEDED = 0.05; //5% power
        public static final double NORMAL_ELEVATOR_SPEED_DIFFERENCE = 3; //inches //TODO: test and change, start with 12 inches

        public static final int ELEVATOR_LIMIT_SWITCH_CHANNEL = 0;

    }

    public static final class ClimbConstants {

        public static final int CLIMBER_ID = 15;

        public static final double CLIMB_SPEED = 1.0;
    }

    public static final class ControllerConstants {

        public static final XboxController DRIVE_CONTROLLER = new XboxController(0);
        public static final XboxController AUX = new XboxController(1);

        public static final int REVERSE_CLIMBER_AUX_PORT = 2;

        public static final double TRIGGER_DEADZONE = 0.05;
        public static final double JOYSTICK_DEADZONE = 0.05;
    }

    public static final class AlgaeConstants {

        public static final int ALGAE_UPPER_ID = 12;
        public static final int ALGAE_LOWER_ID = 13;

        public static final double ALGAE_SPEED = 0.5;

        public static final double ALGAE_TIME = 2;
    }
}
