
package com.MAutils.RobotControl;

import java.util.ArrayList;

public class MRobotState {

    public static final MRobotState NONE = new MRobotState("NONE");
    public static final MRobotState IDLE = new MRobotState("IDLE");

    private static ArrayList<StateSubsystem> subsystemsArry = new ArrayList<StateSubsystem>();

    private State[] subsystemStates;
    private final String stateName;
    private Runnable onStateSet = () -> {

    };

    private Runnable onStateEnd = () -> {

    };

    public static void addSubsystem(StateSubsystem subsystem) {
        if (!subsystemsArry.contains(subsystem)) {
            subsystemsArry.add(subsystem);
        }
    }

    public MRobotState(String name, State... subsystemStates) {
        this.subsystemStates = subsystemStates;
        stateName = name;
    }

    public MRobotState(String name, Runnable onStateSet, State... subsystemStates) {
        this.subsystemStates = subsystemStates;
        stateName = name;
        this.onStateSet = onStateSet;

    }

    public MRobotState(String name, Runnable onStateSet,Runnable onStateEnd, State... subsystemStates) {
        this.subsystemStates = subsystemStates;
        stateName = name;
        this.onStateSet = onStateSet;
        this.onStateEnd = onStateEnd;
    }

    


    public String getStateName() {
        return stateName;
    }

    public Runnable getOnStateEnd() {
        return onStateEnd;
    }

    public void setState() {
        DeafultRobotContainer.setRobotState(this); 
        onStateSet.run();

        for (StateSubsystem subsystem : subsystemsArry) {
            for (State state : subsystemStates) {
                if (state.getSubsystem().getName().equals(subsystem.getName())) { // TODO change to eauals the object
                                                                                  // not the string
                    subsystem.setState(state);
                } // TODO maybe add else go to idle
            }
        }
    }

}
