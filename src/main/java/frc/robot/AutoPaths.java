package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
        LEFT, RIGHT //TODO: put into auto
    }
    public enum CoralNumber {
        ONE, TWO
    }

    // L3 is always blocked at start
    public enum CoralLevel {
        TROUGH, L2, L3, L4
    }

    public AutoAction currentAutoAction;

    // isn't used
    public double getTurnRadiusDistance() { // TODO: immplementation, maybe (for after alignment) change to a constant distance?
        StartingPosition startingPosition = positionChooser.getSelected();
        if (startingPosition == StartingPosition.LEFT || startingPosition == StartingPosition.RIGHT) {
            return 0; // Constants.AutoConstants._RADIUS_DISTANCE; // TODO: change to some positive value
        } else if (positionChooser.getSelected() == StartingPosition.MIDDLE) {
            return 0;
        } else {
            return 0;
        }
    }

    // TODO: implement this method
    public double getDistanceToReefFromStation() {
        return -4.3; // Placeholder value, replace with actual logic
    }

    // Add a limelight align after using this
    public float getInitialTurnAngle() {
        StartingPosition startingPosition = positionChooser.getSelected();
        CoralNumber coralNumber = coralNumberChooser.getSelected();
        int angle = 0;
        if (coralNumber == CoralNumber.ONE) {
            angle = 60;
        } else if (coralNumber == CoralNumber.TWO) {
            angle = 125;
        } else {
            angle = 125; // to be safe for now, edit
        }
        if (startingPosition == StartingPosition.MIDDLE) {
            return 0;
        } else if (startingPosition == StartingPosition.LEFT) {
            return -angle;
        } else if (startingPosition == StartingPosition.RIGHT) {
            return angle;
        } else {
            return 0;
        }
    }

    public double getXAxisSpeed() {
        LeftOrRight leftOrRight = leftOrRightChooser.getSelected();
        if (leftOrRight == LeftOrRight.LEFT) {
            return Constants.DriveConstants.BRANCH_DISTANCE;
        } else if (leftOrRight == LeftOrRight.RIGHT) {
            return 0.0;
        } else {
            return 0;
        }
    }

    public double getSidewaysDistance() {
        return 0.9;
    }

    public double getAtStationTurnAngle() {
        return 0;
    }

    public double getAutoTargetHeight() {
        CoralLevel coralLevel = coralLevelChooser.getSelected();

        switch (coralLevel) {
            case TROUGH:
                return CoralConstants.L2_HEIGHT; //no
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
        CoralNumber coralNumber = coralNumberChooser.getSelected();
        if (startingPosition == StartingPosition.MIDDLE) {
            return 0;
        } else if (startingPosition == StartingPosition.LEFT || startingPosition == StartingPosition.RIGHT) {
            if (coralNumber == CoralNumber.ONE) {
                return 1.5;
            } else if (coralNumber == CoralNumber.TWO) {
                return 4.9;
            } else {
                return 4.9; // TODO: CHANGE, to be safe for now
            }
        } else {
            return 0;
        }
    }

    // Shuffleboard
    ShuffleboardTab tab = Shuffleboard.getTab("Auto");
    GenericEntry leaveOnly, toReef, placeCoral, toStation, toReefAgain, placeCoralAgain, toStationAgain;

    private final SendableChooser<StartingPosition> positionChooser = new SendableChooser<>();
    // private final SendableChooser<ReefFace> reefFaceChooser = new SendableChooser<>();
    private final SendableChooser<LeftOrRight> leftOrRightChooser = new SendableChooser<>();
    private final SendableChooser<CoralLevel> coralLevelChooser = new SendableChooser<>();
    // private final SendableChooser<ReefFace> reefFaceChooser2 = new SendableChooser<>();
    private final SendableChooser<CoralLevel> coralLevelChooser2 = new SendableChooser<>();
    //private final SendableChooser<LeftOrRight> leftOrRIghtChooser2 = new SendableChooser<>();
    private final SendableChooser<CoralNumber> coralNumberChooser = new SendableChooser<>();

    private <K extends Enum<K>> void createChooser(SendableChooser<K> chooser, K[] values, String chooserName, int w, int h, int x, int y) {
        for (K value : values) {
            chooser.addOption(value.name(), value);
        }
        // SmartDashboard.putData(chooserName, chooser);
        tab.add(chooserName, chooser).withSize(w, h).withPosition(x, y);
    }

    private boolean getIfSelected(GenericEntry entry) {
        return entry.getBoolean(false);
    }

    public void autoShuffleboardStartup() {
        createChooser(positionChooser, StartingPosition.values(), "Start Position", 2, 1, 0, 0);
        createChooser(coralLevelChooser, CoralLevel.values(), "Coral Level", 1, 1, 1, 1);
        createChooser(coralLevelChooser2, CoralLevel.values(), "2nd Coral Level", 1, 1, 6, 1);
        createChooser(coralNumberChooser, CoralNumber.values(), "# Coral", 2, 1, 5, 0);

        leaveOnly = tab.add("LEAVE ONLY", false).withWidget("Toggle Button").withSize(2,1).withPosition(2,0).getEntry();
        toReef = tab.add("To Reef", false).withWidget("Toggle Button").withSize(1,1).withPosition(0,1).getEntry();
        placeCoral = tab.add("Place Coral", false).withWidget("Toggle Button").withSize(2,1).withPosition(0,2).getEntry();
        toStation = tab.add("To Station", false).withWidget("Toggle Button").withSize(2,1).withPosition(0,3).getEntry();

        toReefAgain = tab.add("To Reef AGAIN", false).withWidget("Toggle Button").withSize(1,1).withPosition(5,1).getEntry();
        placeCoralAgain = tab.add("Place Coral AGAIN", false).withWidget("Toggle Button").withSize(2,1).withPosition(5,2).getEntry();
        toStationAgain = tab.add("To Station AGAIN", false).withWidget("Toggle Button").withSize(2,1).withPosition(5,3).getEntry();
        
        // SmartDashboard.putBoolean("LEAVE ONLY", false);
        // SmartDashboard.putBoolean("To Reef", false);
        // SmartDashboard.putBoolean("Place Coral", false);
        // SmartDashboard.putBoolean("To Station", false);

        // SmartDashboard.putBoolean("To Reef AGAIN", false);
        // SmartDashboard.putBoolean("Place Coral AGAIN", false);
        // SmartDashboard.putBoolean("To Station AGAIN", false);

        this.currentAutoAction = null;
    }

    // public void testAuto() {
    //     System.out.println("CORAL LEVEL -----------------: " + coralLevelChooser.getSelected());
    // }

    // TODO: CHANGE ALL VALUES BACK TO OG CONSTANT
    public ArrayList<AutoStep> buildPath() {
        ArrayList<AutoStep> path = new ArrayList<>();

        if (getIfSelected(leaveOnly)) {
            // path.add(new AutoStep(AutoAction.DRIVE, AutoConstants.LEAVE_ONLY_DISTANCE, 3));
            path.add(new AutoStep(AutoAction.ELEVATOR, CoralConstants.L4_HEIGHT));
            path.add(new AutoStep(AutoAction.SHOOT, CoralConstants.L4_HEIGHT));
            path.add(new AutoStep(AutoAction.ELEVATOR, CoralConstants.L2_HEIGHT));
            return path;
        }

        if (getIfSelected(toReef)) {
            path.addAll(Arrays.asList(
                new AutoStep(AutoAction.DRIVE, getDriveDistance(), 8),
                new AutoStep(AutoAction.TURN, getInitialTurnAngle()),
                new AutoStep(AutoAction.ALIGN),
                new AutoStep(AutoAction.DRIVE, 0.25, 1.25), //TODO: test: 30 centimeters to reef after aligning??
                new AutoStep(AutoAction.ELEVATOR, getAutoTargetHeight())));
        } else {
            return path;
        }

        if (getIfSelected(placeCoral)) {
            path.add(new AutoStep(AutoAction.SHOOT, getAutoTargetHeight()));
            //path.add(new AutoStep(AutoAction.ELEVATOR, CoralConstants.L2_HEIGHT)); // add back after testing
        } else {
            return path;
        }

        if (getIfSelected(toStation)) {
            path.addAll(Arrays.asList(
                    new AutoStep(AutoAction.DRIVE, -1, 8),
                    new AutoStep(AutoAction.TURN, -60),
                    // new AutoStep(AutoAction.DRIVE, getSidewaysDistance()), // actually... drive right 0.9 meters
                    new AutoStep(AutoAction.DRIVE, getDistanceToReefFromStation()+2.3, 8), // backward!!!
                    new AutoStep(AutoAction.TURN, 54),
                    new AutoStep(AutoAction.DRIVE, -1.2, 2.5),
                    new AutoStep(AutoAction.WAIT, 2)));
        } else {
            return path;
        }

        //Second coral! (second pt of auto)
        if (getIfSelected(toReefAgain)) {
            path.addAll(Arrays.asList(
                    new AutoStep(AutoAction.DRIVE, 0.5, 8),
                    new AutoStep(AutoAction.TURN, -54), // getAtStationTurnAngle()
                    new AutoStep(AutoAction.DRIVE, 1.5, 8),
                    new AutoStep(AutoAction.TURN, 60), // getAtStationTurnAngle()
                    new AutoStep(AutoAction.ALIGN),
                    new AutoStep(AutoAction.DRIVE, 0.5, 8)));
        } else {
            return path;
        }

        if (getIfSelected(placeCoralAgain)) {
            path.add(new AutoStep(AutoAction.SHOOT, getAutoTargetHeight()));
        } else {
            return path;
        }

        if (getIfSelected(toStationAgain)) {
            path.addAll(Arrays.asList(
                new AutoStep(AutoAction.DRIVE, -1, 8),
                new AutoStep(AutoAction.TURN, -60),
                // new AutoStep(AutoAction.DRIVE, getSidewaysDistance()), // actually... drive right 0.9 meters
                new AutoStep(AutoAction.DRIVE, getDistanceToReefFromStation()+2.3, 8), // backward!!!
                new AutoStep(AutoAction.TURN, 54),
                new AutoStep(AutoAction.DRIVE, -1.2, 2.5)));
        }
        return path;
    }
}
