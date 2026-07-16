
package com.MAutils.RobotControl;

import edu.wpi.first.wpilibj.RobotState;

/*
 * Constants and enums for robot control modes.
 */
public class RobotControlConstants {

    public enum RobotMode {
        DISABLED,
        AUTONOMOUS,
        TELEOP,
        TEST
    }


    public static RobotMode getRobotMode() {
        if (RobotState.isAutonomous()) {
            return RobotMode.AUTONOMOUS;
        } else if (RobotState.isTeleop()) {
            return RobotMode.TELEOP;
        } else if (RobotState.isTest()) {
            return RobotMode.TEST;
        }

        return RobotMode.DISABLED;
    }

    public enum SystemMode {
        MANUAL,
        AUTOMATIC
    }


}
