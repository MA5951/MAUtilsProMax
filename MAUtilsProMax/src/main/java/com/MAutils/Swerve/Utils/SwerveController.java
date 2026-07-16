
package com.MAutils.Swerve.Utils;

import com.MAutils.Logger.MALog;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public abstract class SwerveController {

    private String name;
    protected ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

    public SwerveController(String name) {
        this.name = name;
    }

    public abstract void updateSpeeds();

    public void updateControllers() {
        updateSpeeds();
        logController();
    }

    public ChassisSpeeds getSpeeds() {
        return speeds;
    }

    public void logController() {
        MALog.log("Subsystems/Swerve/Controllers/" + name + "/Speeds", speeds);
    }

    public String getName() {
        return name;
    }

}
