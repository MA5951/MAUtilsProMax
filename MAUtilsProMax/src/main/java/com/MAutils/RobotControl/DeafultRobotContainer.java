package com.MAutils.RobotControl;



import com.MAutils.Controllers.MAController;
import com.MAutils.Controllers.PS5MAController;
import com.MAutils.Logger.TelemetryLogger;
import com.MAutils.Utils.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/*
 * Default robot container that manages robot states and controllers.
 */
public abstract class DeafultRobotContainer {

    protected static MRobotState currentRobotState = new MRobotState("IDLE");
    protected static MRobotState lastRobotState = currentRobotState;
    protected static MAController driverController = new PS5MAController(Constants.DRIVER_CONTROLLER_PORT);;
    protected static MAController operatorController = new PS5MAController(Constants.OPERATOR_CONTROLLER_PORT);;


    public DeafultRobotContainer() {
        setRobotState(MRobotState.IDLE);

        configAuto();
        configBinding();
    }

    public static void setDriverController(MAController controller) {
        driverController = controller;

    }

    public void addSystemCommand(SubsystemCommand command) {
        CommandScheduler.getInstance().setDefaultCommand(command.getCommandSubsystem(), command);
    }

    public static MAController getDriverController() {
        return driverController;
    }

    public static void setOperatorController(MAController controller) {
        operatorController = controller;
    }

    public static MAController getOperatorController() {
        return operatorController;
    }

    public static void setRobotState(MRobotState robotState) {
        if (robotState != currentRobotState) {
            currentRobotState.getOnStateEnd().run();
        }
        lastRobotState = currentRobotState;
        currentRobotState = robotState;
    }

    public static MRobotState getRobotState() {
        return currentRobotState;
    }

    public static MRobotState getLastRobotState() {
        return lastRobotState;
    }

    public static Trigger T(StateTrigger trigger) {
        return trigger.build();
    }

    public abstract void configAuto();

    public abstract void configBinding();

}
