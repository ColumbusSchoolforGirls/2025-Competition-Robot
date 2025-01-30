package frc.robot;

public class AutoStep {
    AutoAction action;
    int value;

    public AutoStep(AutoAction action, int value) {
        this.action = action;
        this.value = value;
    }

    public AutoStep(AutoAction action) {
        this.action = action;
    }

    public int getValue() {
        return value;
    }

    public AutoAction getAction() {
        return action;
    }
    
}
