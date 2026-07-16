
package com.MAutils.Swerve.Utils;

public class PIDController extends edu.wpi.first.math.controller.PIDController {
    //TODO why you dont just work with the base PIDContorller?
    //TODO if you choose to use them so at least make sure we can use only the kp and not ki or kd(maybe)

    public PIDController(double kp, double ki, double kd) {
        super(kp, ki, kd);
    }

    public PIDController withSetpoint(double setpoint) {
        this.setSetpoint(setpoint);
        return this;
    }

    public PIDController withTolerance(double positionTolerance, double velocityTolerance) {
        this.setTolerance(positionTolerance, velocityTolerance); 
        return this;
    }

    public PIDController withTolerance(double positionTolerance) {
        this.setTolerance(positionTolerance);
        return this;
    }

    public PIDController withContinuesInput(double minimumInput, double maximumInput) {
        this.enableContinuousInput(minimumInput, maximumInput);
        return this;
    }
}
