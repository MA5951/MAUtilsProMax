
package com.MAutils.RobotControl;

import java.util.function.BooleanSupplier;

import com.MAutils.Auto.AutoManager;
import com.MAutils.RobotControl.RobotControlConstants.RobotMode;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * Class for creating state triggers that change robot states based on conditions.
 */
public class StateTrigger {

    private RobotMode robotMode;
    private BooleanSupplier condition;
    private MRobotState stateToSet;
    private boolean nextStateAuto;
    private MRobotState stateIn;
    public Command setCommand;

    private StateTrigger(BooleanSupplier condition, MRobotState stateToSet) {
        this.condition = condition;
        this.stateToSet = stateToSet;
        robotMode = null;
        stateIn = null;
    }

    public StateTrigger withRobotMode(RobotMode robotMode) {
        this.robotMode = robotMode;
        return this;
    }

    public StateTrigger withInRobotState(MRobotState robotState) {
        this.stateIn = robotState;
        return this;
    }

    public StateTrigger withNextAutoState(boolean nextState) {
        this.nextStateAuto = nextState;
        return this;
    }

    public Trigger build() {
        if (nextStateAuto && DriverStation.isAutonomous()) { //TODO this wouldt work, the build is calld once,splite the func form auto and telop
            setCommand = new InstantCommand(() -> AutoManager.popAutoState().setState());
        } else {
            setCommand = new InstantCommand(() -> stateToSet.setState());
        }

        if (robotMode != null && stateIn != null) {
            return new Trigger(() -> condition.getAsBoolean() && DeafultRobotContainer.getRobotState() == stateIn
                    && RobotControlConstants.getRobotMode() == robotMode)
                    .onTrue(setCommand);
        } else if (robotMode != null) {
            return new Trigger(() -> condition.getAsBoolean() && RobotControlConstants.getRobotMode() == robotMode)
                    .onTrue(setCommand);
        } else if (stateIn != null) {
            return new Trigger(() -> condition.getAsBoolean() && DeafultRobotContainer.getRobotState() == stateIn)
                    .onTrue(setCommand);
        }

        return new Trigger(condition)
                .onTrue(setCommand);
    }

    public static StateTrigger T(BooleanSupplier condition, MRobotState stateToSet) {
        return new StateTrigger(condition, stateToSet);
    } 

}
