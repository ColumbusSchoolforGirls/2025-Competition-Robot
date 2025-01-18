package frc.robot;

import frc.robot.subsystems.Drivetrain;

public class Constants {

    public static final class SwerveConstants {
    public static final double WHEEL_RADIUS = 0.0508; //TODO: change value 
    public static final int ENCODER_RES = 4096; //TODO: change value
    public static final double MODULE_MAX_ANGULAR_VELOCITY = DriveConstants.MAX_ANGULAR_SPEED;
    public static final double MODULE_MAX_ANGULAR_ACCELERATION = 2 * Math.PI; // radians per second squared

    //PID Constants //TODO: change values
    public static final int PROPORTIONAL_COEFFICIENT = 1;
    public static final int INTEGRAL_COEFFICIENT = 0;
    public static final int DERIVATIVE_COEFFICIENT = 0;

    //feed forward constants //TODO: change values
    public static final int STATIC_GAIN_DRIVE = 1;
    public static final int VELOCITY_GAIN_DRIVE = 3;
    public static final int STATIC_GAIN_TURN = 1;
    public static final double VELOCITY_GAIN_TURN = 0.5;

    }

    public static final class DriveConstants {  

    public static final double MAX_SPEED = 3.0; // 3 meters per second
    public static final double MAX_ANGULAR_SPEED = Math.PI; // 1/2 rotation per second
    
    public static double TRANSLATION_2D_OFFSET = 0.381;
    
    //Motor controller and encoder ports and channels //TODO: change values

    }

}
