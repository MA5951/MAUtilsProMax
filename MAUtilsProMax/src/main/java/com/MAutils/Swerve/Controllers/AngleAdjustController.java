
package com.MAutils.Swerve.Controllers;

import java.util.function.Supplier;

import com.MAutils.Logger.MALog;
import com.MAutils.Swerve.SwerveSystem;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.MAutils.Swerve.Utils.SwerveController;

import edu.wpi.first.math.controller.PIDController;

public class AngleAdjustController extends SwerveController {

    private PIDController pidController;
    private Supplier<Double> angleSupplier;
    private Supplier<Double> setPointSupplier;
    private double angleOffset = 0;

    public AngleAdjustController(SwerveSystemConstants swerveSystem, PIDController pidController) {
        super("Angle Adjust Controller");
        this.pidController = pidController;
        this.angleSupplier = () -> 0d; //TODO ?
        this.setPointSupplier = () -> 0d;

        pidController.setSetpoint(angleOffset);
        //withGyroSupplier(() -> SwerveSystem.getInstance(swerveSystem).getAbsYaw()); //TODO GALDO
        //TODO just make it look lika a normal code init angleSupplier like a normal person

    }

    public AngleAdjustController(PIDController pidController, Supplier<Double> angleSupplier) {
        super("Angle Adjust Controller");
        this.pidController = pidController;
        this.angleSupplier = angleSupplier;

        pidController.setSetpoint(angleOffset);
    }

    public AngleAdjustController withGyroSupplier(Supplier<Double> angleSupplier) { //TODO rename to setAngleSupplier
        this.angleSupplier = angleSupplier;
        return this;
    }

    public AngleAdjustController withPIDController(PIDController pidController) {
        this.pidController = pidController;
        return this;
    }

    public AngleAdjustController withSetPoint(double setPoint) {
        setPointSupplier = () -> setPoint;
        return this;
    }

    public AngleAdjustController withSetPoint(Supplier<Double> setPoint) {
        setPointSupplier = setPoint;
        return this;
    }

    public AngleAdjustController withAngleOffset(double angleOffset) {
        this.angleOffset = angleOffset;
        return this;
    }

    public void updateSpeeds() {
        pidController.setSetpoint(setPointSupplier.get() + angleOffset);
        speeds.omegaRadiansPerSecond = pidController.calculate(angleSupplier.get());
    }

    public boolean atSetpoint() {
        return pidController.atSetpoint();
    }

    public double getSetpoint() {
        return pidController.getSetpoint();
    }

    @Override
    public void logController() {
        super.logController();
        MALog.log("Subsystems/Swerve/Controllers/Angle Adjust Controller/Set Point", pidController.getSetpoint());
        MALog.log("Subsystems/Swerve/Controllers/Angle Adjust Controller/At Point", pidController.atSetpoint());
    }
}
