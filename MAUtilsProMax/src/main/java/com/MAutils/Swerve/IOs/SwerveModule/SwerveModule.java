
package com.MAutils.Swerve.IOs.SwerveModule;

import com.MAutils.Logger.MALog;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.MAutils.Swerve.IOs.SwerveModule.SwerveModuleIO.SwerveModuleData;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModule {

    protected final String name;
    protected SwerveModuleData moduleData = new SwerveModuleData();
    protected SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
    protected final SwerveSystemConstants constants;
    protected final SwerveModuleIO moduleIO;

    public SwerveModule(String name, SwerveSystemConstants constants, SwerveModuleIO moduleIO) {
        this.name = name;
        this.constants = constants;
        this.moduleIO = moduleIO;
    }

    private void updateLog() {
        MALog.log("Subsystems/Swerve/Modules/" + name + "/Is Drive Connected", moduleData.isDriveConnected);
        MALog.log("Subsystems/Swerve/Modules/" + name + "/Drive Position", moduleData.drivePosition);
        MALog.log("Subsystems/Swerve/Modules/" + name + "/Drive Velocity", moduleData.driveVelocity);
        MALog.log("Subsystems/Swerve/Modules/" + name + "/Drive Current", moduleData.driveCurrent);
        MALog.log("Subsystems/Swerve/Modules/" + name + "/Drive Volts", moduleData.driveVolts);
        MALog.log("Subsystems/Swerve/Modules/" + name + "/Is Steer Connected", moduleData.isSteerConnected);
        MALog.log("Subsystems/Swerve/Modules/" + name + "/Steer Position", moduleData.steerPosition.getDegrees()); 
        MALog.log("Subsystems/Swerve/Modules/" + name + "/Steer Velocity", moduleData.steerVelocity);
        MALog.log("Subsystems/Swerve/Modules/" + name + "/Steer Current", moduleData.steerCurrent);
        MALog.log("Subsystems/Swerve/Modules/" + name + "/Steer Volts", moduleData.steerVolts);
        MALog.log("Subsystems/Swerve/Modules/" + name + "/Absolute Steer Position", moduleData.absoluteSteerPosition.getDegrees());
        MALog.log("Subsystems/Swerve/Modules/" + name + "/Is Absolute Steer Connected", moduleData.isAbsoluteSteerConnected);
        //TODO log the fast odomerty array
    }

    public void update() {
        moduleIO.updateSwerveModuleData(moduleData);
        updateLog();

        int sampleCount = moduleData.odometryDrivePositionsRad.length;
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
            double positionMeters = moduleData.odometryDrivePositionsRad[i] * constants.WHEEL_RADIUS;
            Rotation2d angle = moduleData.odometryTurnPositions[i];
            odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    public void setDriveNutralMode(boolean isBrake) {
        moduleIO.setDriveNutralMode(isBrake);
    }

    public void setVoltage(double driveVolts, double steerVolts) {
        moduleIO.setDriveVoltage(driveVolts);
        moduleIO.setSteerVoltage(steerVolts);
    }
    
    public void setDriveVoltage(double voltage) {
        moduleIO.setDriveVoltage(voltage);
    }

    public void setSteerVoltage(double voltage) {
        moduleIO.setSteerVoltage(voltage);
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(moduleData.driveVelocity, moduleData.steerPosition);//MPS , Rotation2d
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(moduleData.drivePosition, moduleData.steerPosition);//Meters , Rotation2d
    }

    public SwerveModuleData getModuleData() {
        return moduleData;
    }

    public SwerveModulePosition[] getOdometryPositions() {
        return odometryPositions;
    }

    public void setSetPoint(SwerveModuleState setpoint) {
        moduleIO.setDriveVelocity(setpoint.speedMetersPerSecond);
        moduleIO.setSteerPosition(setpoint.angle);
    }

    //TODO resetSteerPosition() need to be imploment 
    //TODO setPID?
    //TODO setDriveNutralMode,setSteerNutralMode??

}
