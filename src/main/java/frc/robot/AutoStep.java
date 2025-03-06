package frc.robot;

public class AutoStep {
    AutoAction action;
    double value;
    double elevatorHeight;

    public AutoStep(AutoAction action) {
        this.action = action;
    }

    public AutoStep(AutoAction action, double value) {
        this.action = action;
        this.value = value;
    }

    public AutoStep(AutoAction action, double value, double elevatorHeight) {
        this.action = action;
        this.value = value;
        this.elevatorHeight = elevatorHeight;
    }
    
    public AutoAction getAction() {
        return action;
    }

    public double getValue() {
        return value;
    }

    public double getElevatorHeight() {
        return elevatorHeight;
    }
}
