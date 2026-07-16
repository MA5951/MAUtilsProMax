
package com.MAutils.Auto;

import com.MAutils.Logger.MALog;
import com.MAutils.Logger.TelemetryLogger;
import com.MAutils.PoseEstimation.PoseEstimationMA;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.RobotControl.MRobotState;

import edu.wpi.first.wpilibj.Timer;


/*
 * Manages the autonomous routines, including initializing and periodic updates.
 * 
 * Each routine is made up of a series of RobotStates that are executed in order.
 */
public class AutoManager {
    private static final double STARTING_DISTANCE = 0.2;

    private static AutoChooser autoChooser = new AutoChooser();
    private static Timer autoTime = new Timer(); 
    private static MRobotState nexState;

    //TODO i dont understan how this work with pathplaner and how this work with a auto of move in astright line for X sec/dis 
    //ther need to be at the swerve/supersturcr level a way to combain them , its can also be at the auro Routin class and insted of robot state creat a "autoAction" that hold the path planer and swerve contoler
    //i must say i dont see how any part of the auto connect in a normal way includ the on the fly. the Routin misse data 
    //i didnt read all the calc in the planner code, i guesse chatgpt go over it but at the end all need to combain to swerve conroller and swerve stat and the next pose you need to go to / path
    
    public static void setRoutins(AutoRoutine... routins) {
        AutoChooser.setAutoOptions(routins);
    }

    public static MRobotState popAutoState() { //TODO need to add 
        nexState = geAutoRoutin().popState();
        TelemetryLogger.logAuto("Popped Auto State: " + nexState.getStateName());
        return nexState;
    }

    public static boolean atStartingPose() {
        return geAutoRoutin().getStartingPose().getTranslation().getDistance(PoseEstimator.getCurrentPose().getTranslation()) < STARTING_DISTANCE;
    }

    private static AutoRoutine geAutoRoutin() {
        return autoChooser.getSelectedAuto(); 
    }

    public static void autoInit() { 
        TelemetryLogger.logAuto("Starting Auto: " + geAutoRoutin().getName());
        MALog.log("Auto/Starting Pose", geAutoRoutin().getStartingPose());

        if (geAutoRoutin().shouldResetPose()) {
            PoseEstimator.resetPose(geAutoRoutin().getStartingPose());
            TelemetryLogger.logAuto("Resetting Pose to: " + geAutoRoutin().getStartingPose().toString());
        }

        autoTime.start();

        popAutoState().setState();//Note: sets the first state automaticlly, the next states are being popped in the state trigger
    }

    public static void autoPeriodic() {
        MALog.log("Auto/Current State",geAutoRoutin().getCurrentState().getStateName());

        if (geAutoRoutin().getCurrentState() == MRobotState.NONE && autoTime.isRunning()) {
            autoTime.stop();
            TelemetryLogger.logAuto("Finished Auto: " + geAutoRoutin().getName() + " in " + autoTime.get() + " secounds");
        }
    }
}
