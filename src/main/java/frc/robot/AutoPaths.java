package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutoPaths { // obviously need to add more
    public enum StartingPosition {
        LEFT, 
        RIGHT, 
        CENTER
    }

    public enum CoralPosition {
        L1, L2, L4
    }

    boolean returnToDriverStation, knockOffAlgae;

    // the drop down menu to choose a path on the dashboard
    private final SendableChooser<StartingPosition> startingPositionChooser = new SendableChooser<StartingPosition>();
    private final SendableChooser<String> coralPositionChooser = new SendableChooser<String>();
    private final SendableChooser<Boolean> returnToDriverStationFlag = new SendableChooser<Boolean>();
    private final SendableChooser<Boolean> knockOffAlgaeFlag = new SendableChooser<Boolean>();

    public AutoAction currentAutoAction;


    public AutoPaths() {
        SmartDashboard.putData("Starting Position", startingPositionChooser); // actually puts them on the dashboard
                                                                              // after they are added to m_chooser
        for (StartingPosition p : StartingPosition.values()) {
            startingPositionChooser.addOption(p.name(), p);
        }

        SmartDashboard.putData("Coral Position", coralPositionChooser);
        for (CoralPosition coralPosition: CoralPosition.values()) {
            coralPositionChooser.addOption(coralPosition.name(), coralPosition.name());
        }

        this.currentAutoAction = null;
    }

    /**
     * Gets the initial turn angle based on the starting position chooser.
     * @return
     */
    // TODO: implement
    public float getInitialTurnAngle() {
        StartingPosition startingPosition = startingPositionChooser.getSelected(); 
        if (startingPosition == StartingPosition.LEFT) {

       } else if (startingPosition == StartingPosition.RIGHT) {

       } else if (startingPosition == StartingPosition.CENTER) {

       }
       return 0;
    }

    // Updates the currentAutoAction. This handles all state transitions. All actions are in Robot.java
    public AutoAction goToNextState() {
        if (this.currentAutoAction == null) {
            return AutoAction.TURN_TOWARD_REEF;
        } 
        // TODO: Implement the rest of this
        return AutoAction.STOP;
    }
}
