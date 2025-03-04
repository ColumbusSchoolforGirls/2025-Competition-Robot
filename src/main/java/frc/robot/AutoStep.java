package frc.robot;

public class AutoStep {
    AutoAction action;
    int value;
    int elevatorHeight;

    public AutoStep(AutoAction action) {
        this.action = action;
    }

    public AutoStep(AutoAction action, int value) {
        this.action = action;
        this.value = value;
    }

    public AutoStep(AutoAction action, int value, int elevatorHeight) {
        this.action = action;
        this.value = value;
        this.elevatorHeight = elevatorHeight;
    }
    
    public AutoAction getAction() {
        return action;
    }

    public int getValue() {
        return value;
    }

    public int getElevatorHeight() {
        return elevatorHeight;
    }
}
