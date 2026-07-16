
package com.MAutils.AutoChooser;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class Autonomus {
    private Command autoCommand;
    private String autoName;
    private Pose2d startPose;
    private double yawOffset;

    public Autonomus(String autoName, Command autoCommand, Pose2d startPose, double yawOffset) {
        this.autoCommand = autoCommand;
        this.autoName = autoName;
        this.startPose = startPose;
        this.yawOffset = yawOffset;
    }

    public Command getAutoCommand() {
        return autoCommand;
    }

    public String getName() {
        return autoName;
    }

    public Pose2d getStartingPose() {
        return startPose;
    }

    public double getYawOffset() {
        return yawOffset;
    }
}
