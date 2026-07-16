
package com.MAutils.Swerve.Controllers;

import com.MAutils.Swerve.SwerveSystem;
import com.MAutils.Swerve.Utils.SwerveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class ModulesAngleController extends SwerveController {
    
    private final SwerveSystem swerveSystem;
    private SwerveModuleState[] swerveStates;

    public ModulesAngleController(SwerveSystem swerveSystem) {
        super("Modules Angle Controller");
        this.swerveSystem = swerveSystem;
        this.swerveStates = new SwerveModuleState[4];

    }


    public void setAngles(Rotation2d fl, Rotation2d fr, Rotation2d bl, Rotation2d br) {
        swerveStates[0] = new SwerveModuleState(0, fl);
        swerveStates[1] = new SwerveModuleState(0, fr);
        swerveStates[2] = new SwerveModuleState(0, bl);
        swerveStates[3] = new SwerveModuleState(0, br);
    }


    @Override
    public void updateSpeeds() {
        swerveSystem.runSwerveStates(swerveStates);
    }

    

}
