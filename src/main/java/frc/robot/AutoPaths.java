package frc.robot;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoralConstants;
import frc.robot.subsystems.CoralSystem;


public class AutoPaths { 

    private final CoralSystem coralSystem;

    public enum StartingPosition {
        LEFT, 
        RIGHT, 
        CENTER
    }

    public enum CoralPosition {
        L1, L2, L4 // L1 and L2 are at the same height but different alighment, L3 is always the blocked, L2 is sometimes blocked
    }

    boolean knockOffAlgae;

    // the drop down menu to choose a path on the dashboard
    private final SendableChooser<StartingPosition> startingPositionChooser = new SendableChooser<StartingPosition>();
    private final SendableChooser<String> coralPositionChooser = new SendableChooser<String>();
    private final SendableChooser<String> endPositionChooser = new SendableChooser<String>(); 
    private final SendableChooser<Boolean> knockOffAlgaeFlag = new SendableChooser<Boolean>();
    // for endPositionChooser add which coral station to go to or remain stationary. 
    public AutoAction currentAutoAction;


    public AutoPaths(CoralSystem coralSystem) {
        this.coralSystem = coralSystem;
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

    public double getAutoTargetHeight(){
        CoralPosition coralPosition = CoralPosition.valueOf(coralPositionChooser.getSelected());
        if (coralPosition == CoralPosition.L1) {
            return Constants.CoralConstants.L2_HEIGHT;
        } else if (coralPosition == CoralPosition.L2) {
            return Constants.CoralConstants.L2_HEIGHT;
        } else if (coralPosition == CoralPosition.L4) {
            return Constants.CoralConstants.L4_HEIGHT;
        }
        return Constants.CoralConstants.L2_HEIGHT;
    }
    // coral alignment is the same for L2 and L4, different for L1 --> put this into a function



    // Updates the currentAutoAction. This handles all state transitions. All actions are in Robot.java
    public AutoAction goToNextState() {
        if (this.currentAutoAction == null) {
            return AutoAction.TURN_TOWARD_REEF;
        } 
        // TODO: Implement the rest of this
        return AutoAction.STOP;
    }
}
