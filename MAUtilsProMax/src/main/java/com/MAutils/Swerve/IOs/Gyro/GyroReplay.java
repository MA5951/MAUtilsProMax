
package com.MAutils.Swerve.IOs.Gyro;


import com.MAutils.Logger.MALog;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;



public class GyroReplay implements GyroIO {
    //TODO why keep s2025 and this?


    public GyroReplay() {
    }

    public void resetYaw(double yaw) {
    }

    public void updateGyroData(GyroData gyroData) {
        gyroData.isConnected = true;
        gyroData.yaw = MALog.getReplayEntry("Subsystems/Swerve/" + "Piegon 2" + "/Yaw").getDouble(0);
        gyroData.yawVelocity = 0;
        gyroData.pitch = MALog.getReplayEntry("Subsystems/Swerve/" + "Piegon 2" + "/Pitch").getDouble(0);
        gyroData.roll = MALog.getReplayEntry("Subsystems/Swerve/" + "Piegon 2" + "/Roll").getDouble(0);
        gyroData.accelX = MALog.getReplayEntry("Subsystems/Swerve/" + "Piegon 2" + "/Accel X").getDouble(0);
        gyroData.accelY = MALog.getReplayEntry("Subsystems/Swerve/" + "Piegon 2" + "/Accel Y").getDouble(0);

        gyroData.odometryYawTimestamps = new double[] {
            Timer.getFPGATimestamp()
        };
        gyroData.odometryYawPositions = new Rotation2d[] {
            Rotation2d.fromDegrees(MALog.getReplayEntry("Subsystems/Swerve/" + "Piegon 2" + "/Yaw").getDouble(0))
        };
    }

}
