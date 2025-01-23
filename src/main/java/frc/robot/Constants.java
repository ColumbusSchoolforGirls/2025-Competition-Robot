package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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

        // TODO: change ports when the electrical board is finalized
        public static final int FL_DRIVE_ID = 1;
        public static final int FL_TURN_ID = 2;
        public static final int FL_DIO = 1;
        public static final int FL_CHASSIS_ANGULAR_OFFSET = 2; // probably some pi value

        public static final int FR_DRIVE_ID = 1;
        public static final int FR_TURN_ID = 2;
        public static final int FR_DIO = 1;
        public static final int FR_CHASSIS_ANGULAR_OFFSET = 2;

        public static final int BL_DRIVE_ID = 1;
        public static final int BL_TURN_ID = 2;
        public static final int BL_DIO = 1;
        public static final int BL_CHASSIS_ANGULAR_OFFSET = 2;

        public static final int BR_DRIVE_ID = 1;
        public static final int BR_TURN_ID = 2;
        public static final int BR_DIO = 1;
        public static final int BR_CHASSIS_ANGULAR_OFFSET = 2;

        //Gyro Reversed
        public static final boolean kGyroReversed = false; //TODO: is this always false?????
        
    }   

    public static final class CoralConstants {

        public static final int INTAKE_LIMIT_SWITCH_CHANNEL = 1; //TODO: change value

    }

}
