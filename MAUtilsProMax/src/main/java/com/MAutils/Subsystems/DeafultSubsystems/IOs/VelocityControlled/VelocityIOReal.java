package com.MAutils.Subsystems.DeafultSubsystems.IOs.VelocityControlled;

import com.MAutils.CanBus.StatusSignalsRunner;
import com.MAutils.Components.Motor;
import com.MAutils.Logger.MALog;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.VelocitySystemConstants;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.Interfaces.VelocitySystemIO;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.PowerControlled.PowerIOReal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

public class VelocityIOReal extends PowerIOReal implements VelocitySystemIO {
    //TODO same commants as postionIO

    private final VelocityVoltage velocityRequest = new VelocityVoltage(0);

    private StatusSignal<Double> motorError;
    private StatusSignal<Double> motorSetPoint;
    private double setPoint = 0;

    private VelocitySystemConstants systemConstants;

    public VelocityIOReal(VelocitySystemConstants systemConstants) {
        super(systemConstants.toPowerSystemConstants());
        this.systemConstants = systemConstants;

        motorError = systemConstants.master.motorController.getClosedLoopError(false);
        motorSetPoint = systemConstants.master.motorController.getClosedLoopReference(false);
        StatusSignalsRunner.registerSignals(systemConstants.master.canBusID, motorSetPoint, motorError);

        configMotors();
    }

    private  void configMotors() {
        motorConfig.Slot0.kP = systemConstants.getGainConfig().Kp;
        motorConfig.Slot0.kI = systemConstants.getGainConfig().Ki;
        motorConfig.Slot0.kD = systemConstants.getGainConfig().Kd;
        motorConfig.Slot0.kS = systemConstants.getGainConfig().Ks;
        motorConfig.Slot0.kV = systemConstants.getGainConfig().Kv;
        motorConfig.Slot0.kA = systemConstants.getGainConfig().Ka;



        motorConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;


        motorConfig.MotorOutput.Inverted = systemConstants.master.invert;
        systemConstants.master.motorController.getConfigurator().apply(motorConfig);

        for (Motor motor : systemConstants.MOTORS) {
            motorConfig.MotorOutput.Inverted = motor.invert;
            motor.motorController.getConfigurator().apply(motorConfig);
            motor.motorController.setControl(follower);
        }
    }

    @Override
    public void setSystemConstants(VelocitySystemConstants systemConstants) {
        this.systemConstants = systemConstants;
    }

    @Override
    public double getError() {
        return getSetPoint() - getVelocity();
    }

    @Override
    public double getSetPoint() {
        return setPoint;
    }

    @Override
    public boolean atPoint() {
        return Math.abs(getError()) < systemConstants.TOLERANCE;
    }

    @Override
    public void setVelocity(double Velocity) {
        if (Velocity > systemConstants.MAX_VELOCITY) {
            //TODO clamp it dowm dont throw exeption
            throw new IllegalArgumentException("Velocity exceeds maximum limit: " + systemConstants.MAX_VELOCITY);
        }

        setPoint = Velocity;

        systemConstants.master.motorController.setControl(velocityRequest.withVelocity(Velocity / 60)
                .withSlot(0) //TODO constance 
                .withLimitForwardMotion(getCurrent() > systemConstants.MOTOR_LIMIT_CURRENT)
                .withLimitReverseMotion(getCurrent() < -systemConstants.MOTOR_LIMIT_CURRENT));
    }

    @Override
    public void setVelocity(double Velocity, double feedForward) {
        if (Velocity > systemConstants.MAX_VELOCITY) {
            throw new IllegalArgumentException("Velocity exceeds maximum limit: " + systemConstants.MAX_VELOCITY);
        }
        setPoint = Velocity;

        systemConstants.master.motorController.setControl(velocityRequest.withVelocity(Velocity / 60)
                .withSlot(0)
                .withFeedForward(feedForward) //TODO need to check if this is add to the KV KA KS or insted of, also in postion
                .withLimitForwardMotion(getCurrent() > systemConstants.MOTOR_LIMIT_CURRENT)
                .withLimitReverseMotion(getCurrent() < -systemConstants.MOTOR_LIMIT_CURRENT));
    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();

        MALog.log(logPath + "/Set Point", getSetPoint());
        MALog.log(logPath + "/Error", getError());
        MALog.log(logPath + "/At Point", atPoint());

    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        motorConfig.Slot0.kP = systemConstants.getGainConfig().Kp;
        motorConfig.Slot0.kI = systemConstants.getGainConfig().Ki;
        motorConfig.Slot0.kD = systemConstants.getGainConfig().Kd;

        motorConfig.MotorOutput.Inverted = systemConstants.master.invert;
        systemConstants.master.motorController.getConfigurator().apply(motorConfig);
    }

}
