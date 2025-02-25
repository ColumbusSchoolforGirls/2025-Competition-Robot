package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralSystem;


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
     * Gets the turn angle based on the starting position chooser.
     * 
     * @return
     */
    // TODO: implement
    public double getInitialDriveDistance() {
        StartingPosition startingPosition = positionChooser.getSelected();
        if (startingPosition == StartingPosition.LEFT || startingPosition == StartingPosition.RIGHT) {
            return 0; // TODO: change to some positive value
        } else if (startingPosition == StartingPosition.MIDDLE) {
            return 0;
        } else {
            return 0;
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

    public float getInitialTurnAngle() {
        StartingPosition startingPosition = positionChooser.getSelected();
        if (startingPosition == StartingPosition.LEFT) {
            return 0; // TODO: change to some positive (counter) value
        } else if (startingPosition == StartingPosition.RIGHT) {
            return 0; // TODO: change to some negative (clockwise) value
        } else if (startingPosition == StartingPosition.MIDDLE) {
            return 0;
        }
        return 0;
    }

    // TODO: add L3 as an option
    public double getAutoTargetHeight() {
        CoralLevel coralLevel = coralLevelChooser.getSelected();
        if (coralLevel == CoralLevel.TROUGH) {
            return Constants.CoralConstants.L2_HEIGHT;
        } else if (coralLevel == CoralLevel.L2) {
            return Constants.CoralConstants.L2_HEIGHT;
        } else if (coralLevel == CoralLevel.L4) {
            return Constants.CoralConstants.L4_HEIGHT;
        }
        return Constants.CoralConstants.L2_HEIGHT;
    }

    public double getDriveDistance() {
        return 0; // TODO: change value
    }

    public void placeCoral() {
        coralSystem.autoShoot();

        if (coralSystem.autoShootComplete()) {
            goToNextState();
        }
    }

    // Updates the currentAutoAction. This handles all state transitions. All
    // actions are in Robot.java
    public AutoAction goToNextState() {
        if (this.currentAutoAction == null) {
            return AutoAction.TURN_TOWARD_REEF;
        } else if (this.currentAutoAction == AutoAction.TURN_TOWARD_REEF) {
            return AutoAction.GO_TO_REEF_AND_RAISE_ELEVATOR;
        } else if (this.currentAutoAction == AutoAction.GO_TO_REEF_AND_RAISE_ELEVATOR) {
            return AutoAction.SHOOT_CORAL;
        } else if (this.currentAutoAction == AutoAction.SHOOT_CORAL) {
            return AutoAction.ADDITIONAL_DRIVE_ACTIONS;
        } else if (this.currentAutoAction == AutoAction.ADDITIONAL_DRIVE_ACTIONS) {
            return AutoAction.STOP;
        }
        // TODO: Implement the rest of this
        return AutoAction.STOP;
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

    public <K extends Enum<K>> K getSelected(SendableChooser<K> chooser) {
        return chooser.getSelected();
    }

    public boolean getIfSelected(String key) {
        return SmartDashboard.getBoolean(key, false);
    }

    public void autoShuffleboardStartup() {
        createChooser(positionChooser, StartingPosition.values(), "Start Position");
        createChooser(reefFaceChooser, ReefFace.values(), "Reef Face");
        createChooser(leftOrRightChooser, LeftOrRight.values(), "Left or Right");
        createChooser(coralLevelChooser, CoralLevel.values(), "Coral Level");

        SmartDashboard.putBoolean("LEAVE ONLY", false);
        SmartDashboard.putBoolean("To Reef", false);
        SmartDashboard.putBoolean("Place Coral", false);
        SmartDashboard.putBoolean("To Station", false);

        this.currentAutoAction = null;
    }
}
