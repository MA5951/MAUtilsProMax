
package com.MAutils.RobotControl;

/*
 * Represents a state for a specific subsystem of the robot.
 */
public class State {

    public final String stateName; 
    private StateSubsystem subsystem;
    private Runnable onStateSet = () -> {
    };
    private Runnable onStateEnd = () -> {
    };

    public State(String state_name, Runnable onStateSet, StateSubsystem subsystem) {
        this.stateName = state_name;
        this.subsystem = subsystem;
        this.onStateSet = onStateSet;
    }

    public State(String state_name, StateSubsystem subsystem) {
        this.stateName = state_name;
        this.subsystem = subsystem;
    }

    public State(String state_name) {
        this.stateName = state_name;
    }

    public State(String state_name, Runnable onStateSet,Runnable onStateEnd) {
        this.stateName = state_name;
        this.onStateSet = onStateSet;
        this.onStateEnd = onStateEnd;
    }

    public State(String state_name,Runnable onStateEnd) {
        this.stateName = state_name;
        this.onStateEnd = onStateEnd;
    }

    public void setSystem(StateSubsystem subsystem) {
        if (this.subsystem == null) {
            this.subsystem = subsystem;
        } else {
            throw new IllegalArgumentException("Subsystem already set for state " + stateName);
        }
    }

    public void setOnStateSet(Runnable onStateSet) {
        this.onStateSet = onStateSet;
    }

    public void setOnStateEnd(Runnable onStateEnd) {
        this.onStateEnd = onStateEnd;
    }

    public StateSubsystem getSubsystem() {
        return subsystem;
    }

    public void runRunnable() {
        onStateSet.run();
    }

    public void runEndRunnable() {
        onStateEnd.run();
    }

}
