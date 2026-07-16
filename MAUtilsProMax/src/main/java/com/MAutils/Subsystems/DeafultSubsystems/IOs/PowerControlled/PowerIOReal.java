package com.MAutils.Subsystems.DeafultSubsystems.IOs.PowerControlled;

import com.MAutils.CanBus.StatusSignalsRunner;
import com.MAutils.Components.Motor;
import com.MAutils.Logger.MALog;
import com.MAutils.Subsystems.DeafultSubsystems.Constants.PowerSystemConstants;
import com.MAutils.Subsystems.DeafultSubsystems.IOs.Interfaces.PowerSystemIO;
import com.MAutils.Utils.ConvUtil;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;

public class PowerIOReal implements PowerSystemIO {

    protected final VoltageOut voltageRequest = new VoltageOut(0);
    protected final TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    private final MotorOutputConfigs brakeConfig = new MotorOutputConfigs();
    protected StrictFollower follower;

    private StatusSignal<AngularVelocity> motorVelocity;
    private StatusSignal<Angle> motorPosition;
    private StatusSignal<Current> motorCurrent;
    private StatusSignal<Voltage> motorVoltage;

    protected final String logPath;

    private PowerSystemConstants systemConstants;

    public PowerIOReal(PowerSystemConstants systemConstants) {
        this.systemConstants = systemConstants;
        logPath = systemConstants.LOG_PATH == null ? "Subsystems/" + systemConstants.SYSTEM_NAME + "/IO"
                : systemConstants.LOG_PATH;

        configBaseMotors();

        motorVelocity = systemConstants.master.motorController.getVelocity(false);
        motorCurrent = systemConstants.master.motorController.getStatorCurrent(false);
        motorVoltage = systemConstants.master.motorController.getMotorVoltage(false);
        motorPosition = systemConstants.master.motorController.getPosition(false);
        StatusSignalsRunner.registerSignals(systemConstants.master.canBusID, motorVelocity, motorCurrent,
                motorVoltage, motorPosition);

        motorConfig.MotorOutput.Inverted = systemConstants.master.invert;
        systemConstants.master.motorController.getConfigurator().apply(motorConfig);

        follower = new StrictFollower(systemConstants.master.canBusID.id);

        for (Motor motor : systemConstants.MOTORS) {
            motorConfig.MotorOutput.Inverted = motor.invert;
            motor.motorController.getConfigurator().apply(motorConfig);
            motor.motorController.setControl(follower);
        }

    }

    public PowerSystemConstants getSystemConstants() {
        return systemConstants;
    }

    @Override
    public void setSystemConstants(PowerSystemConstants systemConstants) {
        this.systemConstants = systemConstants;
        
        configBaseMotors();
    }

    private  void configBaseMotors() {
        motorConfig.Feedback.SensorToMechanismRatio = systemConstants.GEAR;

        motorConfig.MotorOutput.NeutralMode = systemConstants.IS_BRAKE
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;

        motorConfig.Voltage.PeakForwardVoltage = systemConstants.PEAK_FORWARD_VOLTAGE;
        motorConfig.Voltage.PeakReverseVoltage = systemConstants.PEAK_REVERSE_VOLTAGE;

        motorConfig.CurrentLimits.StatorCurrentLimit = systemConstants.STATOR_CURRENT_LIMIT;
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = systemConstants.CURRENT_LIMIT_ENABLED;
        

        //TODO add limit revers and fowerd
        //TODO ramp rate
    }

    @Override
    public double getCurrent() {
        return motorCurrent.getValueAsDouble();
    }

    @Override
    public double getAppliedVolts() {
        return motorVoltage.getValueAsDouble();
    }

    @Override
    public double getVelocity() {
        return getRawVelocity();
    }

    @Override
    public double getPosition() {
        return getRawPosition() * systemConstants.POSITION_FACTOR;
    }

    @Override
    public double getRawVelocity() {
        return ConvUtil.RPStoRPM(motorVelocity.getValueAsDouble());
    }

    @Override
    public double getRawPosition() {
        return motorPosition.getValueAsDouble() * 360;
    }

    @Override
    public void setBrakeMode(boolean isBrake) {
        motorConfig.MotorOutput.NeutralMode = isBrake
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;

        motorConfig.MotorOutput.Inverted = systemConstants.master.invert;
        systemConstants.master.motorController.getConfigurator().apply(motorConfig);  

        MALog.log(logPath + "/Brake Config Master", Timer.getFPGATimestamp());  
        for (Motor motor : systemConstants.MOTORS) {
            motorConfig.MotorOutput.Inverted = motor.invert;
            MALog.log(logPath + "/Brake Config Slave", Timer.getFPGATimestamp()); 
            motor.motorController.getConfigurator().apply(motorConfig);
        }

       
    }

    @Override
    public void setVoltage(double volt) {
        systemConstants.master.motorController
                .setControl(voltageRequest.withOutput(volt).withEnableFOC(systemConstants.FOC)
                        .withLimitForwardMotion(getCurrent() > systemConstants.MOTOR_LIMIT_CURRENT)
                        .withLimitReverseMotion(getCurrent() < -systemConstants.MOTOR_LIMIT_CURRENT));
    }

    @Override
    public void updatePeriodic() {

        MALog.log(logPath + "/Velocity", getVelocity());
        MALog.log(logPath + "/Voltage", getAppliedVolts());
        MALog.log(logPath + "/Current", getCurrent());
        MALog.log(logPath + "/Position", getPosition());

        MALog.log(logPath + "/Forward Limit", getCurrent() > systemConstants.MOTOR_LIMIT_CURRENT);
        MALog.log(logPath + "/Reverse Limit", getCurrent() < -systemConstants.MOTOR_LIMIT_CURRENT);
    }

    @Override
    public boolean isMoving() {
        return ConvUtil.RPStoRPM(motorVelocity.getValueAsDouble()) > 1; //TODO change to constance
    }

    @Override
    public void restPosition(double position) {
        systemConstants.master.motorController.setPosition(position / systemConstants.POSITION_FACTOR);
    }

}
