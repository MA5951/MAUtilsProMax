
package com.MAutils.Swerve.IOs.SwerveModule;

import java.util.Queue;

import com.MAutils.CanBus.StatusSignalsRunner;
import com.MAutils.Swerve.IOs.PhoenixOdometryThread;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class SwerveModuleTalonFX implements SwerveModuleIO {

    protected final TalonFX driveTalon;
    protected final TalonFX turnTalon;
    protected final CANcoder cancoder;

    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    protected final TalonFXConfiguration driveTalonConfig = new TalonFXConfiguration();
    protected final TalonFXConfiguration turnTalonConfig = new TalonFXConfiguration();

    private final StatusSignal<Angle> drivePosition;
    private final Queue<Double> drivePositionQueue;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;

    private final StatusSignal<Angle> turnAbsolutePosition;
    private final StatusSignal<Angle> turnPosition;
    private final Queue<Double> turnPositionQueue;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnCurrent;

    private final SwerveSystemConstants constants;

    public SwerveModuleTalonFX(SwerveSystemConstants constants, int index) {
        this.constants = constants;
        driveTalon = new TalonFX(constants.MODULES_ID_ARRY[index].getDriveMotor().id,
                constants.MODULES_ID_ARRY[index].getDriveMotor().bus);
        turnTalon = new TalonFX(constants.MODULES_ID_ARRY[index].getSteerMotor().id,
                constants.MODULES_ID_ARRY[index].getSteerMotor().bus);
        cancoder = new CANcoder(constants.MODULES_ID_ARRY[index].getSteerEncoder().id,
                constants.MODULES_ID_ARRY[index].getSteerEncoder().bus);

        configDevices(index);

        drivePosition = driveTalon.getPosition(true);
        drivePositionQueue =
        PhoenixOdometryThread.getInstance(constants).registerSignal(driveTalon.getPosition()
        .clone());
        driveVelocity = driveTalon.getVelocity(false);
        driveAppliedVolts = driveTalon.getMotorVoltage(false);
        driveCurrent = driveTalon.getStatorCurrent(false);

        turnAbsolutePosition = cancoder.getAbsolutePosition(true);
        turnAbsolutePosition.refresh();
        turnTalon.setPosition(turnAbsolutePosition.getValue().in(Rotations));

        turnPosition = turnTalon.getPosition(true);
        turnPositionQueue =
        PhoenixOdometryThread.getInstance(constants).registerSignal(turnTalon.getPosition().clone());
        turnVelocity = turnTalon.getVelocity(false);
        turnAppliedVolts = turnTalon.getMotorVoltage(false);
        turnCurrent = turnTalon.getStatorCurrent(false);

        BaseStatusSignal.setUpdateFrequencyForAll(
                constants.ODOMETRY_UPDATE_RATE, drivePosition, turnPosition, turnAbsolutePosition);
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent,
                turnVelocity,
                turnAppliedVolts,
                turnCurrent); //TODO replace with constance

        StatusSignalsRunner.registerSignals(constants.MODULES_ID_ARRY[0].getDriveMotor() , driveVelocity, driveAppliedVolts,
                driveCurrent, turnVelocity, turnAppliedVolts, turnCurrent, drivePosition, turnPosition, turnAbsolutePosition);

    }

    private void configDevices(int index) {
        driveTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveTalonConfig.MotorOutput.Inverted = constants.DRIVES_INVERT[index];
        driveTalonConfig.Feedback.SensorToMechanismRatio = constants.DRIVE_GEAR_RATIO.gearRatio;
        driveTalonConfig.CurrentLimits.StatorCurrentLimit = constants.DRIVE__SLIP_LIMIT;
        driveTalonConfig.CurrentLimits.StatorCurrentLimitEnable = constants.DRIVE_ENABLE_CURRENT_LIMIT;

        driveTalonConfig.Slot0.kP = constants.DRIVE_TELEOP_GainConfig.Kp;
        driveTalonConfig.Slot0.kI = constants.DRIVE_TELEOP_GainConfig.Ki;
        driveTalonConfig.Slot0.kD = constants.DRIVE_TELEOP_GainConfig.Kd;
        driveTalonConfig.Slot0.kS = constants.DRIVE_TELEOP_GainConfig.Ks;
        driveTalonConfig.Slot0.kV = constants.DRIVE_TELEOP_GainConfig.Kv;
        driveTalonConfig.Slot0.kA = constants.DRIVE_TELEOP_GainConfig.Ka;

        driveTalonConfig.Slot1.kP = constants.DRIVE_AUTO_GainConfig.Kp;
        driveTalonConfig.Slot1.kI = constants.DRIVE_AUTO_GainConfig.Ki;
        driveTalonConfig.Slot1.kD = constants.DRIVE_AUTO_GainConfig.Kd;
        driveTalonConfig.Slot1.kS = constants.DRIVE_AUTO_GainConfig.Ks;
        driveTalonConfig.Slot1.kV = constants.DRIVE_AUTO_GainConfig.Kv;
        driveTalonConfig.Slot1.kA = constants.DRIVE_AUTO_GainConfig.Ka;

        driveTalon.getConfigurator().apply(driveTalonConfig);
        driveTalon.setPosition(0);

        turnTalonConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnTalonConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        turnTalonConfig.Feedback.SensorToMechanismRatio = constants.TURNING_GEAR_RATIO;
        turnTalonConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnTalonConfig.CurrentLimits.StatorCurrentLimit = constants.TURNING__CURRENT_LIMIT;
        turnTalonConfig.CurrentLimits.StatorCurrentLimitEnable = constants.TURNING_ENABLE_CURRENT_LIMIT;
        
        turnTalonConfig.Slot0.kP = constants.STEER_GainConfig.Kp;
        turnTalonConfig.Slot0.kI = constants.STEER_GainConfig.Ki;
        turnTalonConfig.Slot0.kD = constants.STEER_GainConfig.Kd;
        turnTalonConfig.Slot0.kS = constants.STEER_GainConfig.Ks;
        turnTalonConfig.Slot0.kV = constants.STEER_GainConfig.Kv;
        turnTalonConfig.Slot0.kA = constants.STEER_GainConfig.Ka;

        turnTalon.getConfigurator().apply(turnTalonConfig);
        

    }

    public void updateSwerveModuleData(SwerveModuleData data) {
        data.isDriveConnected = BaseStatusSignal.isAllGood(
                drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);
        data.drivePosition = drivePosition.getValue().in(Rotations) * constants.WHEEL_CIRCUMFERENCE;// Rotations to Meters
        data.driveVelocity = driveVelocity.getValue().in(RotationsPerSecond) * constants.WHEEL_CIRCUMFERENCE; // Rotations Per Second to Meters Per Second
        data.driveCurrent = driveCurrent.getValue().in(Amps);
        data.driveVolts = driveAppliedVolts.getValue().in(Volts);


        data.isSteerConnected = BaseStatusSignal.isAllGood(
                turnPosition, turnVelocity, turnAppliedVolts, turnCurrent);
        data.steerPosition = new Rotation2d(turnPosition.getValue().in(Radians));// Radians to Rotation2d
        data.steerVelocity = turnVelocity.getValue().in(RotationsPerSecond) * 60;// Rotations Per Second to RPM
        data.steerCurrent = turnCurrent.getValue().in(Amps);
        data.steerVolts = turnAppliedVolts.getValue().in(Volts);

        data.absoluteSteerPosition = new Rotation2d(turnAbsolutePosition.getValue().in(Radians));// Radians to Rotation2d
        data.isAbsoluteSteerConnected = BaseStatusSignal.isAllGood(turnAbsolutePosition);

        data.odometryDrivePositionsRad =
        drivePositionQueue.stream().mapToDouble(Units::rotationsToRadians).toArray();
        data.odometryTurnPositions =
        turnPositionQueue.stream().map(Rotation2d::fromRotations).toArray(Rotation2d[]::new);

        drivePositionQueue.clear();
        turnPositionQueue.clear();

    }

    public void setDriveVoltage(double volts) {
        driveTalon.setVoltage(volts);
    }

    public void setSteerVoltage(double volts) {
        turnTalon.setVoltage(volts);
    }

    public void setDriveVelocity(double metersPerSecond) {
        driveTalon.setControl(velocityVoltageRequest.withVelocity(metersPerSecond / constants.WHEEL_CIRCUMFERENCE)// Meters Per Second to Rotations Per Second
                .withSlot(SwerveSystemConstants.getControlSlot()).withEnableFOC(true));
    }

    public void setSteerPosition(Rotation2d rotation) {
        turnTalon.setControl(positionVoltageRequest.withPosition(rotation.getRotations()));// Rotation2d to Rotations
    }

    public void setDrivePID(double kP, double kI, double kD) {
        driveTalonConfig.Slot0.kP = kP;
        driveTalonConfig.Slot0.kI = kI;
        driveTalonConfig.Slot0.kD = kD;
        driveTalon.getConfigurator().apply(driveTalonConfig);
    }

    public void setSteerPID(double kP, double kI, double kD) {
        turnTalonConfig.Slot0.kP = kP;
        turnTalonConfig.Slot0.kI = kI;
        turnTalonConfig.Slot0.kD = kD;
        turnTalon.getConfigurator().apply(turnTalonConfig);
    }

    @Override
    public void setDriveNutralMode(boolean isBrake) {
        driveTalonConfig.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        driveTalon.getConfigurator().apply(driveTalonConfig);
    }

    @Override
    public void setSteerNutralMode(boolean isBrake) {
        turnTalonConfig.MotorOutput.NeutralMode = isBrake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
        turnTalon.getConfigurator().apply(turnTalonConfig);
    }

    @Override
    public void resetSteerPosition(Rotation2d rotation) {
        turnTalon.setPosition(rotation.getRotations());
    }
}
