package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.AutoConstants;

public class AutoPaths {

    public enum StartingPosition {
        LEFT, RIGHT, MIDDLE // from drive station persepctive
    }

    public enum ReefFace {
        N, NE, SE, S, SW, NW // cardinal directions from the drive station perspective
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
     * Returns the initial drive distance in meters based on the starting position
     * chooser.
     */
    // TODO: implement
    public double getTurnRadiusDistance() { // TODO: immplementation, maybe (for after alignment) change to a constant distance?
        StartingPosition startingPosition = positionChooser.getSelected();
        if (startingPosition == StartingPosition.LEFT || startingPosition == StartingPosition.RIGHT) {
            return 0; // Constants.AutoConstants._RADIUS_DISTANCE; // TODO: change to some positive value
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
            return 50;
        } else if (startingPosition == StartingPosition.RIGHT) {
            return -50; 
        } else if (startingPosition == StartingPosition.MIDDLE) {
            return 0;
        } else {
            return 0;
        }
    }

    public double getTurnAngleToStation() {
        return 0;
    }

    public double getAtStationTurnAngle() {
        return 0;
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
        StartingPosition startingPosition = positionChooser.getSelected();
        if (startingPosition == StartingPosition.LEFT || startingPosition == StartingPosition.RIGHT) {
            return 2.2;
        } else if (startingPosition == StartingPosition.MIDDLE) {
            return 0;
        } else {
            return 0;
        }
    }

    // Choosers for the shuffleboard
    private final SendableChooser<StartingPosition> positionChooser = new SendableChooser<>();
    private final SendableChooser<ReefFace> reefFaceChooser = new SendableChooser<>();
    private final SendableChooser<LeftOrRight> leftOrRightChooser = new SendableChooser<>();
    private final SendableChooser<CoralLevel> coralLevelChooser = new SendableChooser<>();
    private final SendableChooser<ReefFace> reefFaceChooser2 = new SendableChooser<>();
    private final SendableChooser<CoralLevel> coralLevelChooser2 = new SendableChooser<>();
    private final SendableChooser<LeftOrRight> leftOrRIghtChooser2 = new SendableChooser<>();

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
        createChooser(leftOrRightChooser, LeftOrRight.values(), "L or R");
        createChooser(coralLevelChooser, CoralLevel.values(), "Coral Level");
        // To return to the reef after getting a second coral
        createChooser(reefFaceChooser2, ReefFace.values(), "2nd Reef Face");
        createChooser(coralLevelChooser2, CoralLevel.values(), "2nd Coral Level");
        createChooser(leftOrRIghtChooser2, LeftOrRight.values(), "2nd L or R");

        // SmartDashboard.putBoolean("LEAVE ONLY", false);
        // SmartDashboard.putBoolean("To Reef", false);
        // SmartDashboard.putBoolean("Place Coral", false);
        // SmartDashboard.putBoolean("To Station", false);

        // SmartDashboard.putBoolean("To Reef AGAIN", false);
        // SmartDashboard.putBoolean("Place Coral AGAIN", false);
        // SmartDashboard.putBoolean("To Station AGAIN", false);

        this.currentAutoAction = null;
    }

    public ArrayList<AutoStep> buildPath() {
        ArrayList<AutoStep> path = new ArrayList<>();

        if (getIfSelected("LEAVE ONLY")) {
            path.add(new AutoStep(AutoAction.DRIVE, AutoConstants.LEAVE_ONLY_DISTANCE));
            //path.add(new AutoStep(AutoAction.TURN, 90));
            //path.add(new AutoStep(AutoAction.ALIGN));
            return path;
        }

        if (getIfSelected("TO REEF")) {
            path.addAll(Arrays.asList(
                new AutoStep(AutoAction.DRIVE, getDriveDistance()),
                new AutoStep(AutoAction.TURN, getInitialTurnAngle()),
                new AutoStep(AutoAction.ALIGN),
                new AutoStep(AutoAction.DRIVE, 0.3), //TODO: test: 30 centimeters to reef after aligning??
                new AutoStep(AutoAction.ELEVATOR, getAutoTargetHeight())));
        } else {
            return path;
        }

        if (getIfSelected("PLACE CORAL")) {
            path.add(new AutoStep(AutoAction.SHOOT));
        } else {
            return path;
        }

        if (getIfSelected("TO STATION")) {
            path.addAll(Arrays.asList(
                    new AutoStep(AutoAction.DRIVE, getTurnRadiusDistance()),
                    new AutoStep(AutoAction.TURN, getTurnAngleToStation()), // TODO: turn should be backward (back up to station)
                    new AutoStep(AutoAction.DRIVE, getDistanceToReefFromStation()),
                    new AutoStep(AutoAction.TURN, getAtStationTurnAngle()),
                    new AutoStep(AutoAction.DRIVE, getTurnRadiusDistance())));
        } else {
            return path;
        }

        if (getIfSelected("TO REEF AGAIN")) {
            path.addAll(Arrays.asList(
                    new AutoStep(AutoAction.TURN, getAtStationTurnAngle()), // TODO: change this? might need to be a different value
                    //new AutoStep(AutoAction.DRIVE, getDistanceToReefFromStation()),
                    new AutoStep(AutoAction.ALIGN)));
                    //new AutoStep(AutoAction.DRIVE, getTurnRadiusDistance())));
        } else {
            return path;
        }

        if (getIfSelected("PLACE CORAL AGAIN")) {
            path.add(new AutoStep(AutoAction.SHOOT));
        } else {
            return path;
        }

        if (getIfSelected("TO STATION AGAIN")) {
            path.addAll(Arrays.asList(
                    new AutoStep(AutoAction.DRIVE, getTurnRadiusDistance()),
                    new AutoStep(AutoAction.TURN, getTurnAngleToStation()), // drive backward to station
                    new AutoStep(AutoAction.DRIVE, getDistanceToReefFromStation()),
                    new AutoStep(AutoAction.TURN, getAtStationTurnAngle()),
                    new AutoStep(AutoAction.DRIVE, getTurnRadiusDistance())));
        }

        return path;
    }
}
