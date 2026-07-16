package com.MAutils.Subsystems.DeafultSubsystems.IOs.PositionControlled;

import com.MAutils.CanBus.StatusSignalsRunner;
import com.MAutils.Components.Motor;
import com.MAutils.Logger.MALog;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PositionSystemConstants;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.Interfaces.PositionSystemIO;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.PowerControlled.PowerIOReal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.units.measure.AngularVelocity;

public class PositionIOReal extends PowerIOReal implements PositionSystemIO {

    public PositionSystemConstants systemConstants;

    private MotionMagicVoltage motionMagicRequest = new MotionMagicVoltage(0);
    private PositionVoltage positionRequest = new PositionVoltage(0);

    private StatusSignal<Double> motorError;
    private StatusSignal<Double> motorSetPoint;


    private StatusSignal<Double> motionMagiPose; //TODO magic
    private StatusSignal<AngularVelocity> motionMagiVelocity; //TODO magic

    private double setPoint = 0; //TODO replace with the loopreference


//TODO the unit conversion is correct? need to spicipy more what etch const mean betwwen the factory and the gear 
    public PositionIOReal(PositionSystemConstants systemConstants) {
        super(systemConstants.toPowerSystemConstants());
        this.systemConstants = systemConstants;

        motorError = systemConstants.master.motorController.getClosedLoopError(false);
        motorSetPoint = systemConstants.master.motorController.getClosedLoopReference(false); //TODO this is not the right signal
        motionMagiPose = systemConstants.master.motorController.getClosedLoopReference(false); 
        motionMagiVelocity = systemConstants.master.motorController.getVelocity(false); //TODO this is not the right signal
        StatusSignalsRunner.registerSignals(systemConstants.master.canBusID, motorSetPoint, motorError, motionMagiPose, motionMagiVelocity);

        configMotors();
    }

    private void configMotors() { //TODO why you doing all of this config agin? you can get the motor config and just edit the part you need
        motorConfig.Feedback.SensorToMechanismRatio = systemConstants.GEAR;

        motorConfig.MotorOutput.NeutralMode = systemConstants.IS_BRAKE
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;

        motorConfig.Voltage.PeakForwardVoltage = systemConstants.PEAK_FORWARD_VOLTAGE;
        motorConfig.Voltage.PeakReverseVoltage = systemConstants.PEAK_REVERSE_VOLTAGE;

        motorConfig.CurrentLimits.StatorCurrentLimit = systemConstants.STATOR_CURRENT_LIMIT;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = systemConstants.CURRENT_LIMIT_ENABLED;

        motorConfig.Slot0.kP = systemConstants.getGainConfig().Kp;
        motorConfig.Slot0.kI = systemConstants.getGainConfig().Ki;
        motorConfig.Slot0.kD = systemConstants.getGainConfig().Kd;
        motorConfig.Slot0.kS = systemConstants.getGainConfig().Ks;
        motorConfig.Slot0.kV = systemConstants.getGainConfig().Kv;
        motorConfig.Slot0.kA = systemConstants.getGainConfig().Ka;
        motorConfig.Slot0.StaticFeedforwardSign = systemConstants.IS_MOTION_MAGIC
                ? StaticFeedforwardSignValue.UseVelocitySign
                : StaticFeedforwardSignValue.UseClosedLoopSign;

        motorConfig.MotionMagic.MotionMagicAcceleration = systemConstants.ACCELERATION;
        motorConfig.MotionMagic.MotionMagicCruiseVelocity = systemConstants.CRUISE_VELOCITY;
        motorConfig.MotionMagic.MotionMagicJerk = systemConstants.JERK;

        motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = systemConstants.RAMP_RATE;
        motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = systemConstants.RAMP_RATE; //TODO change to two diff const

        motorConfig.MotorOutput.Inverted = systemConstants.master.invert;
        systemConstants.master.motorController.getConfigurator().apply(motorConfig);
        systemConstants.master.motorController.setPosition(systemConstants.START_POSE / systemConstants.POSITION_FACTOR);

        for (Motor motor : systemConstants.MOTORS) {
            motorConfig.MotorOutput.Inverted = motor.invert;
            motor.motorController.getConfigurator().apply(motorConfig); //TODO i think the follower dont need to be config
            motor.motorController.setControl(follower); 
        }
    }

    @Override
    public void setSystemConstants(PositionSystemConstants systemConstants, boolean burnMotor) {
        this.systemConstants = systemConstants;
        
        if (burnMotor) {
            configMotors();
        }
        
    }

    @Override
    public void resetPosition(double pose) {
        systemConstants.master.motorController.setPosition(pose / systemConstants.POSITION_FACTOR/360);
    }

    @Override
    public double getError() {
        return setPoint - getPosition();
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
    public void setVoltage(double volt) {
        systemConstants.master.motorController.setControl(voltageRequest.withOutput(volt)
                .withEnableFOC(systemConstants.FOC)
                .withLimitForwardMotion(
                        getCurrent() > systemConstants.MOTOR_LIMIT_CURRENT
                                || getPosition() > systemConstants.MAX_POSE || getSetPoint() > systemConstants.MAX_POSE)
                .withLimitReverseMotion(getCurrent() < -systemConstants.MOTOR_LIMIT_CURRENT
                        || getPosition() < systemConstants.MIN_POSE || getSetPoint() < systemConstants.MIN_POSE));
    }

    @Override
    public void setPosition(double position, double voltageFeedForward) {
        //TODO smae as velocity clamp down the postion value
        setPoint = position;
        if (systemConstants.IS_MOTION_MAGIC) {
            systemConstants.master.motorController.setControl(
                    motionMagicRequest.withPosition(position / systemConstants.POSITION_FACTOR / 360)
                            .withSlot(0)
                            .withEnableFOC(systemConstants.FOC)
                            .withFeedForward(voltageFeedForward)
                            .withLimitForwardMotion(getCurrent() > systemConstants.MOTOR_LIMIT_CURRENT
                                    || getPosition() > systemConstants.MAX_POSE|| getSetPoint() > systemConstants.MAX_POSE)
                            .withLimitReverseMotion(getCurrent() < -systemConstants.MOTOR_LIMIT_CURRENT
                                    || getPosition() < systemConstants.MIN_POSE || getSetPoint() < systemConstants.MIN_POSE));
        } else {
            systemConstants.master.motorController.setControl(
                    positionRequest.withPosition(position / systemConstants.POSITION_FACTOR / 360)
                            .withSlot(0)
                            .withFeedForward(voltageFeedForward)
                            .withEnableFOC(systemConstants.FOC)
                            .withLimitForwardMotion(getCurrent() > systemConstants.MOTOR_LIMIT_CURRENT
                                    || getPosition() > systemConstants.MAX_POSE || getSetPoint() > systemConstants.MAX_POSE)
                            .withLimitReverseMotion(getCurrent() < -systemConstants.MOTOR_LIMIT_CURRENT
                                    || getPosition() < systemConstants.MIN_POSE || getSetPoint() < systemConstants.MIN_POSE));
         }
    }

    @Override
    public void setPosition(double position) {
        setPosition(position, systemConstants.getGainConfig().Kf);
    }

    @Override
    public void updatePeriodic() {
        super.updatePeriodic();

        MALog.log(logPath + "/Set Point", getSetPoint());
        MALog.log(logPath + "/Error", getError());
        MALog.log(logPath + "/At Point", atPoint());
        MALog.log(logPath + "/Forward Limit", getCurrent() > systemConstants.MOTOR_LIMIT_CURRENT
        || getPosition() > systemConstants.MAX_POSE || getSetPoint() > systemConstants.MAX_POSE);
        MALog.log(logPath + "/Reverse Limit", getCurrent() < -systemConstants.MOTOR_LIMIT_CURRENT
        || getPosition() < systemConstants.MIN_POSE || getSetPoint() < systemConstants.MIN_POSE); 

        MALog.log(logPath + "/Motion Magic/Set Point", motionMagiPose.getValueAsDouble());
        MALog.log(logPath + "/Motion Magic/Measurment", motionMagiVelocity.getValueAsDouble());
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
