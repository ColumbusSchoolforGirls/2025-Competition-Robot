package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralSystem;
import frc.robot.Constants.AutoConstants;


public class AutoPaths { 
    private final CoralSystem coralSystem = new CoralSystem();

    public enum StartingPosition {
        LEFT, RIGHT, MIDDLE
    }

    public enum ReefFace {
        NORTH, NORTHEAST, SOUTHEAST, SOUTH, SOUTHWEST, NORTHWEST
    }

    public enum LeftOrRight {
        LEFT, RIGHT
    }

    // L3 is always blocked at start
    public enum CoralLevel {
        TROUGH, L2, L3, L4
    }

    public AutoAction currentAutoAction;

    /**
     * Returns the initial drive distance in meters based on the starting position chooser.
     */
    // TODO: implement
    public double getInitialDriveDistance() {
        StartingPosition startingPosition = positionChooser.getSelected();
        Boolean leaveOnly = getIfSelected("LEAVE ONLY");
        if (leaveOnly) {
            return AutoConstants.LEAVE_ONLY_DISTANCE;
        } else {
            if (startingPosition == StartingPosition.LEFT || startingPosition == StartingPosition.RIGHT) {
                return AutoConstants.SIDE_DISTANCE;
            } else if (startingPosition == StartingPosition.MIDDLE) {
                return AutoConstants.MIDDLE_DISTANCE;
            } else {
                return 0;
            }
        }
    }

    public double getEndDriveDistance() { // TODO: immplementation, maybe (for after alignment) change to a constant distance?
        StartingPosition startingPosition = positionChooser.getSelected();
        if (startingPosition == StartingPosition.LEFT || startingPosition == StartingPosition.RIGHT) {
            return 0; // TODO: change to some positive value
        } else if (positionChooser.getSelected() == StartingPosition.MIDDLE) {
            return 0; // TODO: change to some positive value
        } else {
            return 0;
        }
    }

     // TODO: implement this method
     public double getDistanceToReefFromStation() {
        return 0; // Placeholder value, replace with actual logic
    }

    // Add a limelight align after using this
    public float getInitialTurnAngle() {
        StartingPosition startingPosition = positionChooser.getSelected();
        if (startingPosition == StartingPosition.LEFT) {
            return 60; // TODO: change to some positive (counter) value
        } else if (startingPosition == StartingPosition.RIGHT) {
            return -60; // TODO: change to some negative (clockwise) value
        } else if (startingPosition == StartingPosition.MIDDLE) {
            return 0;
        } else {
            return 0;
        }
    }

   
    public double getAutoTargetHeight() {
        CoralLevel coralLevel = coralLevelChooser.getSelected();
        switch (coralLevel) {
            case TROUGH:
                return CoralConstants.L2_HEIGHT;
            case L2:
                return CoralConstants.L2_HEIGHT;
            case L3:
                return CoralConstants.L3_HEIGHT;
            case L4:
                return CoralConstants.L4_HEIGHT;
            default:
                return CoralConstants.L2_HEIGHT;
        }
    }

    public double getDriveDistance() {
        return 0; // TODO: change value
    }


    // Choosers for the shuffleboard
    private final SendableChooser<StartingPosition> positionChooser = new SendableChooser<>();
    private final SendableChooser<ReefFace> reefFaceChooser = new SendableChooser<>();
    private final SendableChooser<LeftOrRight> leftOrRightChooser = new SendableChooser<>();
    private final SendableChooser<CoralLevel> coralLevelChooser = new SendableChooser<>();

    private <K extends Enum<K>> void createChooser(SendableChooser<K> chooser, K[] values, String chooserName) {
        for (K value : values) {
            chooser.addOption(value.name(), value);
        }
        SmartDashboard.putData(chooserName, chooser);
    }

    private boolean getIfSelected(String key) {
        return SmartDashboard.getBoolean(key, false);
    }

    public void autoShuffleboardStartup() { // TODO: if time allows (HAH) write this using Shuffleboard documentation instead!!, definite to-do next year
        createChooser(positionChooser, StartingPosition.values(), "Start Position");
        createChooser(reefFaceChooser, ReefFace.values(), "Reef Face");
        createChooser(leftOrRightChooser, LeftOrRight.values(), "Left or Right");
        createChooser(coralLevelChooser, CoralLevel.values(), "Coral Level");
        //TODO: add go to reef again buttons?
        SmartDashboard.putBoolean("LEAVE ONLY", false);
        SmartDashboard.putBoolean("To Reef", false);
        SmartDashboard.putBoolean("Place Coral", false);
        SmartDashboard.putBoolean("To Station", false);

        this.currentAutoAction = null;
    }

    // TODO: make able to select to go to reef a second time
    public ArrayList<AutoStep> buildPath() {
        ArrayList<AutoStep> path = new ArrayList<>();
        int value = 0; // TODO: PLACEHOLDER, add constants or get() for each
        
        if (getIfSelected("LEAVE ONLY")) {
            path.add(new AutoStep(AutoAction.DRIVE, AutoConstants.LEAVE_ONLY_DISTANCE)); // TODO: change to a turn and drive? LEAVE ONLY turn value
            return path;
        }

        if (getIfSelected("TO REEF")) {
            path.addAll(Arrays.asList(
                new AutoStep(AutoAction.TURN, getInitialTurnAngle()),
                new AutoStep(AutoAction.DRIVE, getInitialDriveDistance()),
                new AutoStep(AutoAction.ALIGN),
                new AutoStep(AutoAction.DRIVE, getEndDriveDistance())));
        }
        
        if (getIfSelected("PLACE CORAL")) {
            path.add(new AutoStep(AutoAction.SHOOT));
            path.set(1, new AutoStep(AutoAction.DRIVE_AND_ELEVATOR, getInitialDriveDistance(), getAutoTargetHeight()));
        }

        if (getIfSelected("TO STATION")) {
            path.addAll(Arrays.asList(
                new AutoStep(AutoAction.DRIVE, getDistanceToReefFromStation()),
                new AutoStep(AutoAction.TURN, value), // TODO: CHANGE
                new AutoStep(AutoAction.ALIGN),
                new AutoStep(AutoAction.DRIVE, getEndDriveDistance())));
        }

        return path;
    }
}


// get values for autoPath value from startingPosition math