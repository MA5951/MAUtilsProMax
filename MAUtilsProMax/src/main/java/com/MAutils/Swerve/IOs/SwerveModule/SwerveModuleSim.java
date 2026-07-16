
package com.MAutils.Swerve.IOs.SwerveModule;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Arrays;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import com.MAutils.Swerve.SwerveSystemConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;


import edu.wpi.first.math.util.Units;

public class SwerveModuleSim implements SwerveModuleIO {

    private final SwerveModuleSimulation simulation;
    private static final double DRIVE_KS = 0.03;
    private static final double DRIVE_KV_ROT = 0.91035; // Same units as TunerConstants: (volt * secs) / rotation
    private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
    private final SimulatedMotorController.GenericMotorController driveMotor;
    private final SimulatedMotorController.GenericMotorController turnMotor;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    private double driveFFVolts = 0.0;
    private final PIDController driveController;
    private final PIDController turnController;
    private boolean driveClosedLoop = false;
    private boolean turnClosedLoop = false;
    private final SwerveSystemConstants constants;

    public SwerveModuleSim(SwerveSystemConstants constants, int index, SwerveModuleSimulation simulation) {
        this.simulation = simulation;
        this.constants = constants;
        this.driveMotor = simulation
                .useGenericMotorControllerForDrive()
                .withCurrentLimit(Amps.of(constants.DRIVE__SLIP_LIMIT));
        this.turnMotor = simulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(constants.TURNING__CURRENT_LIMIT));

        this.driveController = new PIDController(0.05, 0.0, 0.0);
        this.turnController = new PIDController(8.0, 0.0, 0.0);

        // Enable wrapping for turn PID
        turnController.enableContinuousInput(-Math.PI, Math.PI);
    }

    public void updateSwerveModuleData(SwerveModuleData moduleData) {
        if (driveClosedLoop) {
            driveAppliedVolts = driveFFVolts
                    + driveController.calculate(
                        simulation.getDriveWheelFinalSpeed().in(RadiansPerSecond));
        } else {
            driveController.reset();
        }
        if (turnClosedLoop) {
            turnAppliedVolts = turnController.calculate(
                simulation.getSteerAbsoluteFacing().getRadians());
        } else {
            turnController.reset();
        }

        // Update simulation state
        driveMotor.requestVoltage(Volts.of(driveAppliedVolts));
        turnMotor.requestVoltage(Volts.of(turnAppliedVolts));

        moduleData.isDriveConnected = true;
        moduleData.drivePosition = simulation.getDriveWheelFinalPosition().in(Rotation) * constants.WHEEL_CIRCUMFERENCE;//Rotation to Meters
        moduleData.driveVelocity = simulation.getDriveWheelFinalSpeed().in(RotationsPerSecond) * constants.WHEEL_CIRCUMFERENCE;//RPM to Meters Per Second
        moduleData.driveVolts = driveAppliedVolts;
        moduleData.driveCurrent = Math.abs(simulation.getDriveMotorStatorCurrent().in(Amps));

        moduleData.isSteerConnected = true;
        moduleData.steerPosition = simulation.getSteerAbsoluteFacing();
        moduleData.steerVelocity = simulation.getSteerAbsoluteEncoderSpeed().in(DegreesPerSecond);
        moduleData.steerVolts = turnAppliedVolts;
        moduleData.steerCurrent = Math.abs(simulation.getSteerMotorStatorCurrent().in(Amps));

        moduleData.isAbsoluteSteerConnected = true;
        moduleData.absoluteSteerPosition = simulation.getSteerAbsoluteFacing();
    
        moduleData.odometryDrivePositionsRad = Arrays.stream(simulation.getCachedDriveWheelFinalPositions())
                .mapToDouble(angle -> angle.in(Radians))
                .toArray();
        moduleData.odometryTurnPositions = simulation.getCachedSteerAbsolutePositions();
    }

    public void setDriveVoltage(double volts) {
        driveClosedLoop = false;
        driveAppliedVolts = volts;
    }

    public void setSteerVoltage(double volts) {
        turnClosedLoop = false;
        turnAppliedVolts = volts;
    }

    public void setDriveVelocity(double metersPerSecond) {
        driveClosedLoop = true;
        driveFFVolts = DRIVE_KS * Math.signum((metersPerSecond/constants.WHEEL_RADIUS)) + DRIVE_KV * (metersPerSecond/constants.WHEEL_RADIUS);//MPS to Radians Per Secound
        driveController.setSetpoint((metersPerSecond/constants.WHEEL_RADIUS));//MPS to Radians Per Secound
    }

    public void setSteerPosition(Rotation2d rotation) {
        turnClosedLoop = true;
        turnController.setSetpoint(rotation.getRadians());
    }

    public void setDrivePID(double kP, double kI, double kD) {
        
    }

    public void setSteerPID(double kP, double kI, double kD) {
        
    }

    public void setDriveNutralMode(boolean isBrake) {
    }

    public void setSteerNutralMode(boolean isBrake) {
    }

    public void resetSteerPosition(Rotation2d rotation) {
    }

}
