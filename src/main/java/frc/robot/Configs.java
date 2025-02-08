package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.SwerveConstants;

public class Configs {

    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(SwerveConstants.drivingFactor) // meters
                    .velocityConversionFactor(SwerveConstants.drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(SwerveConstants.drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.encoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                //     .inverted(true)
                    .positionConversionFactor(SwerveConstants.turningFactor) // radians
                    .velocityConversionFactor(SwerveConstants.turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, SwerveConstants.turningFactor);

                    turningConfig.closedLoop.maxMotion.allowedClosedLoopError(0.05);


        }
    }
}
