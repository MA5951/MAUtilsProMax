
package com.MAutils.Swerve.IOs.SwerveModule;

import com.MAutils.Logger.MALog;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class SwerveModuleReplayS2025 implements SwerveModuleIO {
    //TODO same as the gyro

    private String name;

    public SwerveModuleReplayS2025(String name) {
        this.name = name;
    }

    public void updateSwerveModuleData(SwerveModuleData moduleData) {


        moduleData.isDriveConnected = true;
        moduleData.drivePosition = NetworkTableInstance.getDefault().getEntry("/Replay//MALog//Subsystems/Swerve/Modules/" + name + "/Drive Position").getDouble(0);
        moduleData.driveVelocity = NetworkTableInstance.getDefault().getEntry("/Replay//MALog//Subsystems/Swerve/Modules/" + name + "/Drive Velocity").getDouble(0);//Drive Velocity
        moduleData.driveVolts = NetworkTableInstance.getDefault().getEntry("/Replay//MALog//Subsystems/Swerve/Modules/" + name + "/Drive Volts").getDouble(0);//Drive Volts
        moduleData.driveCurrent = NetworkTableInstance.getDefault().getEntry("/Replay//MALog//Subsystems/Swerve/Modules/" + name + "/Drive Current").getDouble(0);//Drive Current

        moduleData.isSteerConnected = true;
        moduleData.steerPosition = Rotation2d.fromRadians(NetworkTableInstance.getDefault().getEntry("/Replay//MALog//Subsystems/Swerve/Modules/" + name + "/Steer Position").getDouble(0));//Steer Position
        moduleData.steerVelocity = NetworkTableInstance.getDefault().getEntry("/Replay//MALog//Subsystems/Swerve/Modules/" + name + "/Steer Velocity").getDouble(0);//Steer Velocity
        moduleData.steerVolts = 0;
        moduleData.steerCurrent = 0;

        moduleData.isAbsoluteSteerConnected = true;
        moduleData.absoluteSteerPosition = Rotation2d.fromDegrees(0);
    
        moduleData.odometryDrivePositionsRad = new double[] {
            moduleData.drivePosition / 0.0508
        };
        moduleData.odometryTurnPositions = new Rotation2d[] {
            moduleData.steerPosition
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
