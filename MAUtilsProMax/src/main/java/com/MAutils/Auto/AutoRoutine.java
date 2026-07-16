
package com.MAutils.Auto;

import com.MAutils.Logger.TelemetryLogger;
import com.MAutils.RobotControl.MRobotState;

import edu.wpi.first.math.geometry.Pose2d;


/*
 * Represents an autonomous routine made up of a series of RobotStates.
 */
public class AutoRoutine {

    private MRobotState[] states;
    private String name;
    private int currentStateIndex = -1; 
    private Pose2d startingPose;
    private boolean resetPose;

    public AutoRoutine(String name, Pose2d startPose, boolean resetPose ,MRobotState... states) {
        this.name = name;
        this.startingPose = startPose;
        this.resetPose = resetPose;

        if (states.length == 0) {
            this.states = new MRobotState[] { MRobotState.NONE }; //TODO change to idle 
            TelemetryLogger.logAuto("Warning: AutoRoutin: " + name + " has no states defined, setting to NONE");
        } else {
            this.states = states;
        }
    }

    public String getName() {
        return name;
    }

    public Pose2d getStartingPose() {
        return startingPose;
    }

    public MRobotState getCurrentState() {
        return states[currentStateIndex];
    }

    public boolean shouldResetPose() {
        return resetPose;
    }

    public MRobotState popState() {
        currentStateIndex++; 
        if (currentStateIndex < states.length) {
            return states[currentStateIndex];
        } else {
            return MRobotState.NONE; //TODO IDLE
        }
    }

}
