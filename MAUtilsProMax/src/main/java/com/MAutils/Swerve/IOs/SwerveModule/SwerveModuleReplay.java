
package com.MAutils.Swerve.IOs.SwerveModule;


import com.MAutils.Logger.MALog;
import edu.wpi.first.math.geometry.Rotation2d;



public class SwerveModuleReplay implements SwerveModuleIO {


    private String name;

    public SwerveModuleReplay(String name) {
        this.name = name;
    }

    public void updateSwerveModuleData(SwerveModuleData moduleData) {


        moduleData.isDriveConnected = true;
        moduleData.drivePosition = MALog.getReplayEntry("Subsystems/Swerve/Modules/" + name + "/Drive Position").getDouble(0);
        moduleData.driveVelocity = MALog.getReplayEntry("Subsystems/Swerve/Modules/" + name + "/Drive Velocity").getDouble(0);
        moduleData.driveVolts = MALog.getReplayEntry("Subsystems/Swerve/Modules/" + name + "/Drive Volts").getDouble(0);
        moduleData.driveCurrent = Math.abs(MALog.getReplayEntry("Subsystems/Swerve/Modules/" + name + "/Drive Current").getDouble(0));

        moduleData.isSteerConnected = true;
        moduleData.steerPosition = Rotation2d.fromDegrees(MALog.getReplayEntry("Subsystems/Swerve/Modules/" + name + "/Steer Position").getDouble(0));
        moduleData.steerVelocity = MALog.getReplayEntry("Subsystems/Swerve/Modules/" + name + "/Steer Velocity").getDouble(0);
        moduleData.steerVolts = 0;
        moduleData.steerCurrent = 0;

        moduleData.isAbsoluteSteerConnected = true;
        moduleData.absoluteSteerPosition = Rotation2d.fromDegrees(0);
    
        moduleData.odometryDrivePositionsRad = new double[] {
            MALog.getReplayEntry("Subsystems/Swerve/Modules/" + name + "/Drive Position").getDouble(0) / 0.0508
        };
        moduleData.odometryTurnPositions = new Rotation2d[] {
            Rotation2d.fromDegrees(MALog.getReplayEntry("Subsystems/Swerve/Modules/" + name + "/Absolute Angle").getDouble(0))
        };
    }

    public void setDriveVoltage(double volts) {
    }

    public void setSteerVoltage(double volts) {
    }

    public void setDriveVelocity(double metersPerSecond) {
    }

    public void setSteerPosition(Rotation2d rotation) {
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
