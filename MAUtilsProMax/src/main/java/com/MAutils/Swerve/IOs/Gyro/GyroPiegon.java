
package com.MAutils.Swerve.IOs.Gyro;

import java.util.Queue;

import com.MAutils.CanBus.StatusSignalsRunner;
import com.MAutils.Swerve.IOs.PhoenixOdometryThread;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.Gs;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import frc.robot.RobotContainer;

public class GyroPiegon implements GyroIO {

    private final Pigeon2 piegon;
    private final StatusSignal<Angle> yawAngle;
    private final StatusSignal<Angle> pitchAngle;
    private final StatusSignal<Angle> rollAngle;
    private final StatusSignal<AngularVelocity> yawRate;
    private final StatusSignal<LinearAcceleration> accelX;
    private final StatusSignal<LinearAcceleration> accelY;
    private Queue<Double> yawPositionQueue;
    private Queue<Double> yawTimestampQueue;

    public GyroPiegon(SwerveSystemConstants constants) {
        piegon = new Pigeon2(constants.PIEGEON_CAN_ID.id, constants.PIEGEON_CAN_ID.bus);

        piegon.getConfigurator().setYaw(0);

        yawAngle = piegon.getYaw(true);
        pitchAngle = piegon.getPitch(false);
        rollAngle = piegon.getRoll(false);
        yawRate = piegon.getAngularVelocityZDevice(false);
        accelX = piegon.getAccelerationX(false);
        accelY = piegon.getAccelerationY(false);
        //TODO add accelZ

        yawTimestampQueue = PhoenixOdometryThread.getInstance(constants).makeTimestampQueue(); 
        yawPositionQueue = PhoenixOdometryThread.getInstance(constants).registerSignal(yawAngle.clone());

        BaseStatusSignal.setUpdateFrequencyForAll(
                constants.ODOMETRY_UPDATE_RATE, yawAngle);

        StatusSignalsRunner.registerSignals(constants.PIEGEON_CAN_ID, pitchAngle, rollAngle, yawRate, accelX, accelY, yawAngle);

    }

    public void resetYaw(double yaw) {
        piegon.setYaw(yaw);
    }

    public void updateGyroData(GyroData gyroData) {
        gyroData.isConnected = BaseStatusSignal.isAllGood(yawAngle, pitchAngle, rollAngle, yawRate);
        gyroData.yaw = yawAngle.getValue().in(Degrees) + RobotContainer.autoSelector.getSelectedAuto().getYawOffset();
        gyroData.yawVelocity = yawRate.getValue().in(DegreesPerSecond);
        gyroData.pitch = pitchAngle.getValue().in(Degrees);
        gyroData.roll = rollAngle.getValue().in(Degrees);
        gyroData.accelX = accelX.getValue().in(Gs);
        gyroData.accelY = accelY.getValue().in(Gs);
        //TODO need to log the yawTimestampQueue and yawPositionQueue

        gyroData.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        gyroData.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);

        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

}
