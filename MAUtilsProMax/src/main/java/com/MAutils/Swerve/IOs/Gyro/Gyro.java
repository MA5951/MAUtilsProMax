
package com.MAutils.Swerve.IOs.Gyro;

import com.MAutils.Logger.MALog;
import com.MAutils.Swerve.IOs.Gyro.GyroIO.GyroData;

public class Gyro {

    private final String name;
    private final GyroIO gyroIO;
    private GyroData gyroData = new GyroData();

    public Gyro(String name, GyroIO gyroIO) {
        this.gyroIO = gyroIO;
        this.name = name;
    }

    public void update() {
        gyroIO.updateGyroData(gyroData);
        logGyroData();
    }

    private void logGyroData() {
        MALog.log("Subsystems/Swerve/" + name + "/Is Connected", gyroData.isConnected);
        MALog.log("Subsystems/Swerve/" + name + "/Yaw", gyroData.yaw);
        MALog.log("Subsystems/Swerve/" + name + "/Yaw Velocity", gyroData.yawVelocity);
        MALog.log("Subsystems/Swerve/" + name + "/Pitch", gyroData.pitch);
        MALog.log("Subsystems/Swerve/" + name + "/Roll", gyroData.roll);
        MALog.log("Subsystems/Swerve/" + name + "/Accel X", gyroData.accelX);
        MALog.log("Subsystems/Swerve/" + name + "/Accel Y", gyroData.accelY);
    }

    public GyroData getGyroData() {
        return gyroData;
    }

    public void resetYaw(double yaw) {
        gyroIO.resetYaw(yaw);
    }

    

}
