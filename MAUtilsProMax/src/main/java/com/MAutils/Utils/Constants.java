
package com.MAutils.Utils;

import edu.wpi.first.wpilibj.DriverStation;

public class Constants {
    public static final String MAUTILS_VERSION = "3.6.0";

    public enum SimulationType {
        SIM,
        REPLAY;
    }
    //TODO add base on the other commants constnace

    public static final double LOOP_TIME = 0.02; // 20ms loop time

    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    public static final double EPSILON = 1e-9;


    public static final SimulationType SIMULATION_TYPE = SimulationType.REPLAY; 
    public static final boolean TUNING_MODE = true; 
    public static boolean COMPETITION_LOG = true;

    public static void cheackConstants() { //TODO this shoudnt belong to the constnats class put it in the superstucer class
        if (DriverStation.isFMSAttached() ) { //Nots: If in copetition mode, enable competition logging 
            COMPETITION_LOG = true;
        }
    }

}
