
package com.MAutils.Swerve.IOs.Gyro;

import static edu.wpi.first.units.Units.DegreesPerSecond;
import org.ironmaple.simulation.drivesims.GyroSimulation;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.MAutils.Utils.PhoenixUtil;

import edu.wpi.first.math.geometry.Rotation2d;



public class GyroSim implements GyroIO {

    private GyroSimulation gyroSim;

    public GyroSim(SwerveSystemConstants constants) {
        gyroSim = constants.SWERVE_DRIVE_SIMULATION.getGyroSimulation();
    }

    public void resetYaw(double yaw) {
        gyroSim.setRotation(new Rotation2d(yaw));
    }

    public void updateGyroData(GyroData gyroData) {
        gyroData.isConnected = true;
        gyroData.yaw = gyroSim.getGyroReading().getDegrees();
        gyroData.yawVelocity = gyroSim.getMeasuredAngularVelocity().in(DegreesPerSecond);
        gyroData.pitch = 0;
        gyroData.roll = 0;
        gyroData.accelX = 0;
        gyroData.accelY = 0;

        gyroData.odometryYawTimestamps = PhoenixUtil.getSimulationOdometryTimeStamps();
        gyroData.odometryYawPositions = gyroSim.getCachedGyroReadings();
    }

}
