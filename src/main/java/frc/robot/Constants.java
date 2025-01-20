package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.robot.subsystems.Drivetrain;

public class Constants {

    public static final class SwerveConstants {
        public static final double WHEEL_RADIUS = 0.0508; //TODO: change value
        public static final double MODULE_MAX_ANGULAR_VELOCITY = DriveConstants.MAX_ANGULAR_SPEED;
        public static final double MODULE_MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // radians per second squared

        //feed forward constants //TODO: change values if we use feedforward
        public static final int STATIC_GAIN_DRIVE = 1;
        public static final int VELOCITY_GAIN_DRIVE = 3;
        public static final int STATIC_GAIN_TURN = 1;
        public static final double VELOCITY_GAIN_TURN = 0.5;

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
    }   

}
