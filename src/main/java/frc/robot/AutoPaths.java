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

    public double getDriveToReefDistance() {
        StartingPosition startingPosition = positionChooser.getSelected();
        if (startingPosition == StartingPosition.LEFT || startingPosition == StartingPosition.RIGHT) {
            return 0; // TODO: change to some positive value
        } else if (positionChooser.getSelected() == StartingPosition.MIDDLE) {
            return 0; // TODO: change to some positive value
        } else {
            return 0;
        }
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

    // Updates the currentAutoAction. This handles all state transitions. All
    // actions are in Robot.java
    // public AutoAction goToNextState() {
    //     if (this.currentAutoAction == null) {
    //         return AutoAction.STOP;
    //     } else if (this.currentAutoAction == AutoAction.TURN_TOWARD_REEF) {
    //         return AutoAction.GO_TO_REEF_AND_RAISE_ELEVATOR;
    //     } else if (this.currentAutoAction == AutoAction.GO_TO_REEF_AND_RAISE_ELEVATOR) {
    //         return AutoAction.SHOOT_CORAL;
    //     } else if (this.currentAutoAction == AutoAction.SHOOT_CORAL) {
    //         return AutoAction.ADDITIONAL_DRIVE_ACTIONS;
    //     } else if (this.currentAutoAction == AutoAction.ADDITIONAL_DRIVE_ACTIONS) {
    //         return AutoAction.STOP;
    //     }
    //     // TODO: Implement the rest of this
    //     return AutoAction.STOP;
    // }

    // public void goToNextState2() {
    //     switch (this.currentAutoAction) {
    //         case LEAVE_ONLY:
    //             // implement
    //             this.currentAutoAction = AutoAction.INITIAL_DRIVE;
    //         case INITIAL_DRIVE:
    //             // implement
    //             this.currentAutoAction = AutoAction.TURN_TOWARD_REEF;
    //         case TURN_TOWARD_REEF:
    //             // implement
    //             this.currentAutoAction = AutoAction.GO_TO_REEF;
    //         case GO_TO_REEF:
    //             // implement
    //             this.currentAutoAction = AutoAction.SHOOT_CORAL;
    //         case SHOOT_CORAL:
    //             // implement
    //             this.currentAutoAction = AutoAction.STATION;

    //     }
    // }

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

    public void autoShuffleboardStartup() {
        createChooser(positionChooser, StartingPosition.values(), "Start Position");
        createChooser(reefFaceChooser, ReefFace.values(), "Reef Face");
        createChooser(leftOrRightChooser, LeftOrRight.values(), "Left or Right");
        createChooser(coralLevelChooser, CoralLevel.values(), "Coral Level");
        //TODO: add more end position options
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
            path.add(new AutoStep(AutoAction.DRIVE, value)); // TODO: change to a turn and drive? LEAVE ONLY turn value
            return path;
        }

        if (getIfSelected("TO REEF")) {
            path.addAll(Arrays.asList(new AutoStep(AutoAction.DRIVE, value), new AutoStep(AutoAction.TURN, value), new AutoStep(AutoAction.DRIVE, value)));
        }
        
        if (getIfSelected("PLACE CORAL")) {
            path.add(new AutoStep(AutoAction.SHOOT, value));
        }

        if (getIfSelected("TO STATION")) {
            // TODO: STATION is a placeholder, change to actual action (probably drives and turns)
            path.addAll(Arrays.asList(new AutoStep(AutoAction.DRIVE, value), new AutoStep(AutoAction.TURN, value), new AutoStep(AutoAction.DRIVE, value)));
        }

        return path;
    }
}


// get values for autoPath value from startingPosition math