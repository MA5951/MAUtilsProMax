
package com.MAutils.Auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;


/*
 * Used to create a chooser for autonomous routines on the shuffleboard.
 */
public class AutoChooser {
    private static SendableChooser<AutoRoutine> routinChooser;
    public AutoChooser() {
        routinChooser = new SendableChooser<>();
        Shuffleboard.getTab("Auto").add("Auto Chooser", routinChooser);

    }

    public static void setAutoOptions(AutoRoutine... options) {
        for (AutoRoutine autoOption : options) {
            routinChooser.addOption(autoOption.getName(), autoOption);
        }

        routinChooser.setDefaultOption(options[0].getName(), options[0]); 

    }

    public static AutoRoutine getSelectedAuto() { 
        return routinChooser.getSelected();
    }

    public static String getAutonomousName() {
        return getSelectedAuto().getName();
    }

    public static Pose2d getSelectedAutoStartingPose() {
        return getSelectedAuto().getStartingPose();
    }

    
}