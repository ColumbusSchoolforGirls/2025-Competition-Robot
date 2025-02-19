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
        L1, L2, L4 // L1 and L2 are at the same height but different alighment, L3 is always the blocked, L2 is sometimes blocked
    }

    public enum EndPosition{
        leftCoralStation, rightCoralStation, stayAtCoral, awayFromCoral // awayFromCoral is in between the 2 coral stations
    }

    boolean knockOffAlgae;

    // the drop down menu to choose a path on the dashboard
    private final SendableChooser<StartingPosition> startingPositionChooser = new SendableChooser<StartingPosition>();
    private final SendableChooser<String> coralPositionChooser = new SendableChooser<String>();
    private final SendableChooser<String> endPositionChooser = new SendableChooser<String>(); 
    private final SendableChooser<Boolean> knockOffAlgaeFlag = new SendableChooser<Boolean>();
    // for endPositionChooser add which coral station to go to or remain stationary. 
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

        SmartDashboard.putData("End Position", endPositionChooser);
        for (EndPosition endPosition: EndPosition.values()) {
            coralPositionChooser.addOption(endPosition.name(), endPosition.name());
        }

        this.currentAutoAction = null;


    }

    /**
     * Gets the initial turn angle based on the starting position chooser.
     * @return
     */
    // TODO: implement
    
    public double getInitialDriveDistance() {
        StartingPosition startingPosition = startingPositionChooser.getSelected();
        if (startingPosition == StartingPosition.LEFT || startingPosition == StartingPosition.RIGHT) {
            return 0; //TODO: change to some positive value
        } else if (startingPosition == StartingPosition.CENTER) {
            return 0;
        } else {
            return 0;
        }
    }

    public double getDriveToReefDistance() {
        StartingPosition startingPosition = startingPositionChooser.getSelected();
        if (startingPosition == StartingPosition.LEFT || startingPosition == StartingPosition.RIGHT) {
            return 0; //TODO: change to some positive value
        } else if (startingPositionChooser.getSelected() == StartingPosition.CENTER) {
            return 0; //TODO: change to some positive value
        } else {
            return 0;
        }
    }
    
    public float getInitialTurnAngle() {
        StartingPosition startingPosition = startingPositionChooser.getSelected(); 
        if (startingPosition == StartingPosition.LEFT) {
            return 0; //TODO: change to some positive (counter) value
       } else if (startingPosition == StartingPosition.RIGHT) {
            return 0; //TODO: change to some negative (clockwise) value
       } else if (startingPosition == StartingPosition.CENTER) {
            return 0;
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

    public double getInitialEndAutoTargetAngle(){ //TODO: change values
        EndPosition endPosition = EndPosition.valueOf(endPositionChooser.getSelected());
        StartingPosition startingPosition = startingPositionChooser.getSelected(); 
        if (startingPosition == StartingPosition.LEFT) {
            if (endPosition == EndPosition.leftCoralStation) {
                return 0;
            } else if (endPosition == EndPosition.rightCoralStation) {
                return 0;
            } else if (endPosition == EndPosition.stayAtCoral) {
                return 0;
            } else if (endPosition == EndPosition.awayFromCoral) {
                return 0;
            }
        } else if (startingPosition == StartingPosition.RIGHT) {
            if (endPosition == EndPosition.leftCoralStation) {
                return 0;
            } else if (endPosition == EndPosition.rightCoralStation) {
                return 0;
            } else if (endPosition == EndPosition.stayAtCoral) {
                return 0;
            } else if (endPosition == EndPosition.awayFromCoral) {
                return 0;
            }
        } else if (startingPosition == StartingPosition.CENTER) {
            if (endPosition == EndPosition.leftCoralStation) {
                return 0;
            } else if (endPosition == EndPosition.rightCoralStation) {
                return 0;
            } else if (endPosition == EndPosition.stayAtCoral) {
                return 0;
            } else if (endPosition == EndPosition.awayFromCoral) {
                return 0;
            }
        }
        return 0;
    }

    public double getMiddleEndAutoTargetAngle(){ //TODO: change values
        EndPosition endPosition = EndPosition.valueOf(endPositionChooser.getSelected());
        StartingPosition startingPosition = startingPositionChooser.getSelected(); 
        if (startingPosition == StartingPosition.LEFT) {
            if (endPosition == EndPosition.leftCoralStation) {
                return 0;
            } else if (endPosition == EndPosition.rightCoralStation) {
                return 0;
            } else if (endPosition == EndPosition.stayAtCoral) {
                return 0;
            } else if (endPosition == EndPosition.awayFromCoral) {
                return 0;
            }
        } else if (startingPosition == StartingPosition.RIGHT) {
            if (endPosition == EndPosition.leftCoralStation) {
                return 0;
            } else if (endPosition == EndPosition.rightCoralStation) {
                return 0;
            } else if (endPosition == EndPosition.stayAtCoral) {
                return 0;
            } else if (endPosition == EndPosition.awayFromCoral) {
                return 0;
            }
        } else if (startingPosition == StartingPosition.CENTER) {
            if (endPosition == EndPosition.leftCoralStation) {
                return 0;
            } else if (endPosition == EndPosition.rightCoralStation) {
                return 0;
            } else if (endPosition == EndPosition.stayAtCoral) {
                return 0;
            } else if (endPosition == EndPosition.awayFromCoral) {
                return 0;
            }
        }
        return 0;
    }

    public double getInitialEndAutoTargetDistance(){ //TODO: change values
        EndPosition endPosition = EndPosition.valueOf(endPositionChooser.getSelected());
        if (endPosition == EndPosition.leftCoralStation) {
            return 0;
        } else if (endPosition == EndPosition.rightCoralStation) {
            return 0;
        } else if (endPosition == EndPosition.stayAtCoral) {
            return 0;
        } else if (endPosition == EndPosition.awayFromCoral) {
            return 0;
        }
        return 0;
    }

    public double getMiddleEndAutoTargetDistance() { //TODO: change values
        EndPosition endPosition = EndPosition.valueOf(endPositionChooser.getSelected());
        if (endPosition == EndPosition.leftCoralStation) {
            return 0;
        } else if (endPosition == EndPosition.rightCoralStation) {
            return 0;
        } else if (endPosition == EndPosition.stayAtCoral) {
            return 0;
        } else if (endPosition == EndPosition.awayFromCoral) {
            return 0;
        }
        return 0;
    }

    public double getFinalEndAutoTargetDistance() { //TODO: change values
        EndPosition endPosition = EndPosition.valueOf(endPositionChooser.getSelected());
        if (endPosition == EndPosition.leftCoralStation) {
            return 0;
        } else if (endPosition == EndPosition.rightCoralStation) {
            return 0;
        } else if (endPosition == EndPosition.stayAtCoral) {
            return 0;
        } else if (endPosition == EndPosition.awayFromCoral) {
            return 0;
        }
        return 0;
    }

    public double getFinalEndAutoTargetAngle() {
        EndPosition endPosition = EndPosition.valueOf(endPositionChooser.getSelected());
        if (endPosition == EndPosition.leftCoralStation) {
            return 0;
        } else if (endPosition == EndPosition.rightCoralStation) {
            return 0;
        } else if (endPosition == EndPosition.stayAtCoral) {
            return 0;
        } else if (endPosition == EndPosition.awayFromCoral) {
            return 0;
        }
        return 0;
    }

    public void knockOffAlgae(){

        if (knockOffAlgaeFlag.getSelected() == true) {
            // do the knock off algae action
        } else {
            // do nothing
        }
    
    }

    public double getDriveDistance(){
        return 0; //TODO: change value
    }

    // Updates the currentAutoAction. This handles all state transitions. All actions are in Robot.java
    public AutoAction goToNextState() {
        if (this.currentAutoAction == null) {
            return AutoAction.TURN_TOWARD_REEF;
        } else if(this.currentAutoAction == AutoAction.TURN_TOWARD_REEF) {
            return AutoAction.GO_TO_REEF_AND_RAISE_ELEVATOR;
        } else if(this.currentAutoAction == AutoAction.GO_TO_REEF_AND_RAISE_ELEVATOR) {
            return AutoAction.SHOOT_CORAL;
        } else if(this.currentAutoAction == AutoAction.SHOOT_CORAL) {
            return AutoAction.ADDITIONAL_DRIVE_ACTIONS;
        } else if(this.currentAutoAction == AutoAction.ADDITIONAL_DRIVE_ACTIONS) {
            return AutoAction.STOP;
        }
        // TODO: Implement the rest of this
        return AutoAction.STOP;
    }


}
