
package com.MAutils.RobotControl;

import com.MAutils.Logger.MALog;
import com.MAutils.RobotControl.RobotControlConstants.SystemMode;
import com.MAutils.Subsystems.SelfTests.SelfSystemTest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * Abstract class for subsystems that manage states and system modes.
 */
public abstract class StateSubsystem extends SubsystemBase {

    private State currentState;
    private State lastState;
    private SystemMode systemMode;
    public final String subsystemName;
    public final SelfSystemTest selfSystemTest;
    protected final String LOG_PATH;

    public StateSubsystem(String name, State... subsystemsStates) {
        super();

        for (State state : subsystemsStates) {
            state.setSystem(this);
        }

        selfSystemTest = new SelfSystemTest(this);

        currentState = new State("IDLE", this); //TODO why not init it in the constace at the subsystemsStates and just get it from the array 
        systemMode = SystemMode.AUTOMATIC;

        this.subsystemName = name;

        LOG_PATH = "Subsystems/" + subsystemName;

        createSelfTest();
    }

    public void setState(State state) {
        lastState = currentState;
        state.runRunnable();
        currentState = state;
    }

    public State getLastState() {
        return lastState;
    }

    public State getCurrentState() {
        return currentState;
    }

    public void setSystemMode(SystemMode mode) {
        this.systemMode = mode;
    }

    public SystemMode getSystemMode() {
        return systemMode;
    }

    public abstract void createSelfTest();

    public abstract boolean CAN_MOVE();

    @Override
    public void periodic() {
        MALog.log("/RobotControl/" + subsystemName + "/Current State", currentState.stateName);
        MALog.log("/RobotControl/" + subsystemName + "/System Function State", getSystemMode().name());
        MALog.log("/RobotControl/" + subsystemName + "/Can Move", CAN_MOVE());
    }

}
