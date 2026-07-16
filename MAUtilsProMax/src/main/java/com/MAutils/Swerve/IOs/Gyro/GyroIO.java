
package com.MAutils.Swerve.IOs.Gyro;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GyroIO {

    public static class GyroData { 
        public boolean isConnected = false; 
        public double yaw = 0; //Degrees
        public double yawVelocity = 0; //Degrees per second
        public double pitch = 0; //Degrees
        public double roll = 0; //Degrees
        public double accelX = 0; //G
        public double accelY = 0; //G
        public double[] odometryYawTimestamps = new double[] {};
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
    }

    void updateGyroData(GyroData data);

    void resetYaw(double yaw);

}