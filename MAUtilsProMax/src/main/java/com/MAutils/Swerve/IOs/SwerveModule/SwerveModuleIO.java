
package com.MAutils.Swerve.IOs.SwerveModule;

import edu.wpi.first.math.geometry.Rotation2d;

public interface SwerveModuleIO {

    public static class SwerveModuleData {
        public boolean isDriveConnected;
        public double drivePosition;//Meters
        public double driveVelocity;//Meters Per Secound
        public double driveCurrent;//Amps
        public double driveVolts;//Volts

        public boolean isSteerConnected;
        public Rotation2d steerPosition;//Rotation2d
        public double steerVelocity;//RPM
        public double steerCurrent;//Amps
        public double steerVolts;//Volts

        public Rotation2d absoluteSteerPosition;//Rotation2d
        public boolean isAbsoluteSteerConnected;

        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

        public SwerveModuleData() {
            isDriveConnected = false;
            drivePosition = 0.0;
            driveVelocity = 0.0;
            driveCurrent = 0.0;
            driveVolts = 0.0;

            isSteerConnected = false;
            steerPosition = Rotation2d.fromDegrees(0.0);
            steerVelocity = 0.0;
            steerCurrent = 0.0;
            steerVolts = 0.0;

            absoluteSteerPosition = Rotation2d.fromDegrees(0);
            isAbsoluteSteerConnected = false;
        }
    }

    void updateSwerveModuleData(SwerveModuleData data);

    void resetSteerPosition(Rotation2d rotation);

    void setDriveVoltage(double volts);

    void setSteerVoltage(double volts);

    void setDriveVelocity(double metersPerSecond);

    void setSteerPosition(Rotation2d rotation);

    void setDrivePID(double kP, double kI, double kD);

    void setSteerPID(double kP, double kI, double kD);

    void setDriveNutralMode(boolean isBrake);

    void setSteerNutralMode(boolean isBrake);


}
