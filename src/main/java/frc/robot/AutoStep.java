package frc.robot;

public class AutoStep {
    AutoAction action;
    double value;
    double timeLimit;
    double elevatorHeight;

    public AutoStep(AutoAction action) {
        this.action = action;
    }

    public AutoStep(AutoAction action, double value) {
        this.action = action;
        this.value = value;
    }

    public AutoStep(AutoAction action, double value, double timeLimit) {
        this.action = action;
        this.value = value;
        this.timeLimit = timeLimit;
    }

    // only used when driving and using elevator at the same time
    public AutoStep(AutoAction action, double value, double timeLimit, double elevatorHeight) {
        this.action = action;
        this.value = value;
        this.timeLimit = timeLimit;
        this.elevatorHeight = elevatorHeight;
    }
    
    public AutoAction getAction() {
        return action;
    }

    public double getValue() {
        return value;
    }

    public double getTimeLimit() {
        return timeLimit;
    }

    public double getElevatorHeight() { 
        return elevatorHeight;
    }

    @Override
    public String toString() {
        return String.format("AutoAction(%s)",action.name());
    }
}
