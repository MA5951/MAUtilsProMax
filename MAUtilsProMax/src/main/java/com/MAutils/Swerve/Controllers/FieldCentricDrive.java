
package com.MAutils.Swerve.Controllers;

import java.util.function.Supplier;

import com.MAutils.Controllers.MAController;
import com.MAutils.Logger.MALog;
import com.MAutils.PoseEstimation.PoseEstimationMA;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.Swerve.IOs.Gyro.GyroIO.GyroData;
import com.MAutils.Swerve.SwerveSystem;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.MAutils.Swerve.Utils.SwerveController;
import com.MAutils.Utils.ChassisSpeedsUtil;
import com.MAutils.Utils.DriverStationUtil;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotConstants;
import frc.robot.RobotContainer;

public class FieldCentricDrive extends SwerveController {

    private MAController controller;
    private final SwerveSystemConstants constants;

    private double xyScaler = 1;
    private double omegaScaler = 1;

    private Supplier<GyroData> gyroDataSupplier;
    private double angleOffset = 0;
    private SlewRateLimiter tunrLimite = new SlewRateLimiter(12);

    public FieldCentricDrive(MAController controller, SwerveSystemConstants constants,
            Supplier<GyroData> gyroDataSupplier) {
        super("Field Centric Drive");
        this.constants = constants;
        this.controller = controller;
        this.gyroDataSupplier = gyroDataSupplier;
    }

    public FieldCentricDrive withSclers(double xyScaler, double omegaScaler) {
        this.omegaScaler = omegaScaler;
        this.xyScaler = xyScaler;
        return this;
    }

    public void updateOffset() {
        angleOffset = gyroDataSupplier.get().yaw;
    }

    public void setOffset(double angleOffset) {
        this.angleOffset = angleOffset;
    }

    public double getOffset() {
        return angleOffset;
    }

    public void updateSpeeds() {
        speeds.vxMetersPerSecond = -controller.getLeftY(true, xyScaler) * constants.MAX_VELOCITY;
        speeds.vyMetersPerSecond = -controller.getLeftX(true, xyScaler) * constants.MAX_VELOCITY;
        speeds.omegaRadiansPerSecond = -controller.getRightX(true, omegaScaler) * constants.MAX_ANGULAR_VELOCITY;// (Math.pow(-controller.getRightX(true,
                                                                                                      // omegaScaler),2)*(Math.abs(-controller.getRightX(true,
                                                                                                      // omegaScaler) )
                                                                                                      // *
                                                                                                      // (1/-controller.getRightX(true,
                                                                                                      // omegaScaler))))
                                                                                                      // *
                                                                                                      // constants.MAX_ANGULAR_VELOCITY;

        if (RobotContainer.getRobotState() == RobotConstants.FEEDING_IN_MOTION) {
            if ((PoseEstimator.getCurrentPose().getRotation().getDegrees() < 0
                    && PoseEstimator.getCurrentPose().getRotation().getDegrees() < -160
                    && RobotContainer.getDriverController().getRightX(true, 1) > 0
                    || RobotContainer.getDriverController().getRightX(true, 1) == 0) && DriverStationUtil.getAlliance() == Alliance.Blue) {// ||
                                                                                       // (PoseEstimator.getCurrentPose().getRotation().getDegrees()
                                                                                       // < 0 &&
                                                                                       // PoseEstimator.getCurrentPose().getRotation().getDegrees()
                                                                                       // > -180 &&
                                                                                       // RobotContainer.getDriverController().getRightX(true,
                                                                                       // 1) < 0)
                speeds.omegaRadiansPerSecond = 0;
            }
            
            
        }

        if (RobotContainer.getRobotState() == RobotConstants.SHOOTING_UNLOCKED && speeds.vxMetersPerSecond < -0.25 ) {
            speeds.vxMetersPerSecond /= 2;
        }

        speeds = ChassisSpeedsUtil.FromFieldToRobot(speeds,
                Rotation2d.fromDegrees(gyroDataSupplier.get().yaw - angleOffset));

    }

    @Override
    public void logController() {
        super.logController();
        MALog.log("Subsystems/Swerve/Controllers/Field Centric Drive/Angle Offset", angleOffset);
    }

}
