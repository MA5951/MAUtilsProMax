
package com.MAutils.Swerve.Controllers;

import java.util.function.Supplier;

import com.MAutils.Logger.MALog;
import com.MAutils.Swerve.SwerveSystem;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.MAutils.Swerve.Utils.ProfiledPIDController;
import com.MAutils.Swerve.Utils.SwerveController;
import com.MAutils.Utils.ConvUtil;

import edu.wpi.first.math.controller.PIDController;

public class ProfiledAngleAdjustController extends SwerveController {

    private ProfiledPIDController pidController;
    private Supplier<Double> angleSupplier;
    private double angleOffset = 0;

    public ProfiledAngleAdjustController(SwerveSystemConstants swerveSystem, ProfiledPIDController pidController) {
        super("Profiled Angle Adjust Controller");
        this.pidController = pidController;
        

        pidController.withSetpoint(angleOffset);
       

    }

    public ProfiledAngleAdjustController(ProfiledPIDController pidController, Supplier<Double> angleSupplier) {
        super("Profiled Angle Adjust Controller");
        this.pidController = pidController;
        this.angleSupplier = angleSupplier;

        pidController.withSetpoint(angleOffset);
    }

    public ProfiledAngleAdjustController withGyroSupplier(Supplier<Double> angleSupplier) { 
        this.angleSupplier = angleSupplier;
        return this;
    }

    public ProfiledAngleAdjustController withPIDController(ProfiledPIDController pidController) {
        this.pidController = pidController;
        return this;
    }

    public ProfiledAngleAdjustController withSetPoint(double setPoint) {
        pidController.withSetpoint(setPoint + angleOffset);
        return this;
    }

    public ProfiledAngleAdjustController withSetPoint(Supplier<Double> setPoint) {
        pidController.withSetpoint(setPoint.get() + angleOffset);
        return this;
    }

    public ProfiledAngleAdjustController withAngleOffset(double angleOffset) {
        this.angleOffset = angleOffset;
        return this;
    }

    public void updateSpeeds() {
        speeds.omegaRadiansPerSecond = ConvUtil.DegreesToRadians(pidController.calculate(angleSupplier.get()));
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    public double getSetpoint() {
        return pidController.getGoal().position;
    }

    @Override
    public void logController() {
        super.logController();
        MALog.log("Subsystems/Swerve/Controllers/Angle Adjust Controller/Set Point", getSetpoint());
        MALog.log("Subsystems/Swerve/Controllers/Angle Adjust Controller/At Point", pidController.atSetpoint());
        MALog.log("Subsystems/Swerve/Controllers/Angle Adjust Controller/Profile Set Point", pidController.getSetpoint().position);
    }
}
