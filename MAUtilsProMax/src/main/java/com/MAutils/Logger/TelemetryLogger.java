
package com.MAutils.Logger;


/*
 * Utility class for logging telemetry data to different categories.
 */
public class TelemetryLogger {
    public static void logAuto(String value) {
        MALog.log("Telemetry/Auto/", value);
    }

    public static void logPoseEstimator(String value) {
        MALog.log("Telemetry/Pose Estimator/Estimator/", value);
    }

    public static void logSwerve(String value) {
        MALog.log("Telemetry/Pose Estimator/Swerve/", value);
    }

    public static void logVision(String value) {
        MALog.log("Telemetry/Pose Estimator/Vision/", value);
    }

    public static void logSignal(String value) {
        MALog.log("Telemetry/System/Signal", value);
    }

    public static void logRio(String key, double value) {
        MALog.log("Telemetry/System/Rio/"+ key, value);
    }   

    public static void logCanRio(String key, double value) {
        MALog.log("Telemetry/System/Can/Rio/"+ key, value);
    }   

    public static void logCanCanivore(String key, String value) {
        MALog.log("Telemetry/System/Can/Canivor/"+ key, value);
    }  
    
    public static void logPower(String key, double value) {
        MALog.log("Telemetry/System/Power/"+ key, value);
    }

    public static void logDriverStation(String key, String value) {
        MALog.log("Telemetry/Driver Station/"+ key, value);
    }

    public static void logSubsystems(String value) {
        MALog.log("Telemetry/Subsystems/Status", value);
    }

}
