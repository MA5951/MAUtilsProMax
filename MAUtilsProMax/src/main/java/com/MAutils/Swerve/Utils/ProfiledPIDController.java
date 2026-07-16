
package com.MAutils.Swerve.Utils;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

public class ProfiledPIDController extends edu.wpi.first.math.controller.ProfiledPIDController {

    public ProfiledPIDController(double Kp, double Ki, double Kd, Constraints constraints) {
        super(Kp, Ki, Kd, constraints);
    }

    public ProfiledPIDController withSetpoint(double setpoint) {
        this.setGoal(setpoint);
        return this;
    }

    public ProfiledPIDController withTolerance(double positionTolerance, double velocityTolerance) {
        this.setTolerance(positionTolerance, velocityTolerance);
        return this;
    }


    public ProfiledPIDController withTolerance(double positionTolerance) {
        this.setTolerance(positionTolerance);
        return this;
    }

    public ProfiledPIDController withContinuesInput(double minimumInput, double maximumInput) {
        this.enableContinuousInput(minimumInput, maximumInput);
        return this;
    }

    public ProfiledPIDController withConstraints(Constraints constraints) {
        this.setConstraints(constraints);
        return this;
    }

    public ProfiledPIDController withConstraints(double maxVelocity, double maxAcceleration) {
        this.setConstraints(new Constraints(maxVelocity, maxAcceleration));
        return this;
    }
}
