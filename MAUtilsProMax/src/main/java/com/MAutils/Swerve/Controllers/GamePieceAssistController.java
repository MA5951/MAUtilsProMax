
package com.MAutils.Swerve.Controllers;

import java.util.function.Supplier;

import com.MAutils.Controllers.MAController;
import com.MAutils.Logger.MALog;
import com.MAutils.Swerve.IOs.Gyro.GyroIO.GyroData;
import com.MAutils.Swerve.SwerveSystem;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.MAutils.Swerve.Utils.SwerveController;
import com.MAutils.Utils.ChassisSpeedsUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GamePieceAssistController extends SwerveController {

    private MAController controller;
    private final SwerveSystemConstants constants;
    private Supplier<GyroData> gyroDataSupplier;
    private Supplier<Pose2d> targetLocationSupplier, robotLocationSupplier;
    private double angleOffset = 180, Kp = 0, kMaxCorrection = 0, xyScaler = 1, omegaScaler = 1, forwardDot = 0,
            lateralError = 0, assistSpeed = 0;
    private Translation2d driverVelo, driverDirection, robotToTarget, perpVelo, finalVelo;

    public GamePieceAssistController(SwerveSystemConstants constants, MAController controller,
            Supplier<Pose2d> targetLocationSupplier, Supplier<Pose2d> robotLocationSupplier, double Kp,
            double kMaxCorrection) {
        super("Field Centric Drive");
        this.constants = constants;
        this.controller = controller;
        this.Kp = Kp;
        this.kMaxCorrection = kMaxCorrection;
        this.targetLocationSupplier = targetLocationSupplier;
        this.robotLocationSupplier = robotLocationSupplier;
        //this.gyroDataSupplier = () -> SwerveSystem.getInstance(constants).getGyroData();//TODO GALDO
    }

    public GamePieceAssistController withSclers(double xyScaler, double omegaScaler) {
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
        speeds.vxMetersPerSecond = -controller.getLeftY(true, xyScaler) *
                constants.MAX_VELOCITY;
        speeds.vyMetersPerSecond = -controller.getLeftX(true, xyScaler) *
                constants.MAX_VELOCITY;
        speeds.omegaRadiansPerSecond = -controller.getRightX(true, omegaScaler) *
                constants.MAX_ANGULAR_VELOCITY;



        driverVelo = new Translation2d(-speeds.vyMetersPerSecond, speeds.vxMetersPerSecond);
        MALog.log("Subsystems/Swerve/Controllers/Assist/Driver Velo", driverVelo);

        
        driverDirection = driverVelo.div(driverVelo.getNorm());
        MALog.log("Subsystems/Swerve/Controllers/Assist/Driver Direction", driverDirection);


        if (driverVelo.getNorm() > 0.05) {

            robotToTarget = targetLocationSupplier.get().getTranslation()
                    .minus(robotLocationSupplier.get().getTranslation());
            MALog.log("Subsystems/Swerve/Controllers/Assist/Robot To Target", robotToTarget);

            // forwardDot is ALSO the forward distance in meters because driverDirection is
            // unit length
            forwardDot = (robotToTarget.getX() * driverDirection.getX()
                    + robotToTarget.getY() * driverDirection.getY());
            MALog.log("Subsystems/Swerve/Controllers/Assist/Forward Dot", forwardDot);

            if (!(forwardDot <= 0.0)) {
                perpVelo = new Translation2d(-driverDirection.getY(), driverDirection.getX());
                MALog.log("Subsystems/Swerve/Controllers/Assist/Prep Velo", perpVelo);

                lateralError = robotToTarget.getX() * perpVelo.getX()
                        + robotToTarget.getY() * perpVelo.getY();

                // -----------------------------
                // NEW: dynamic aggressiveness
                // -----------------------------

                // 1) More assist when DRIVING FASTER
                double speed = driverVelo.getNorm(); // m/s
                double nominalSpeed = 4.9; // m/s – tune!
                double speedFactor = speed / nominalSpeed; // 1.0 at nominal
                // Clamp so it doesn't explode at crazy speeds
                speedFactor = Math.max(0.5, Math.min(2.5, speedFactor));

                // 2) More assist when CLOSER to the target (forward distance small)
                // forwardDot is projection along driverDirection, i.e., distance in front of us
                double forwardDistance = forwardDot; // meters
                double farDistance = 4.0; // distance where factor ~= 1.0 (tune)
                double eps = 0.2; // to avoid division by zero very close
                double distanceFactor = farDistance / (forwardDistance + eps);
                // Clamp to keep it sane
                distanceFactor = Math.max(1.0, Math.min(3.0, distanceFactor));

                // Effective Kp
                double effectiveKp = Kp * speedFactor * distanceFactor;
                MALog.log("Subsystems/Swerve/Controllers/Assist/Speed Factor", speedFactor);
                MALog.log("Subsystems/Swerve/Controllers/Assist/Distance Factor", distanceFactor);
                MALog.log("Subsystems/Swerve/Controllers/Assist/Effective Kp", effectiveKp);

                // Final assist speed
                assistSpeed = effectiveKp * lateralError;

                assistSpeed = Math.max(-kMaxCorrection, Math.min(kMaxCorrection, assistSpeed));
                MALog.log("Subsystems/Swerve/Controllers/Assist/Assist Velo", perpVelo.times(assistSpeed));

                finalVelo = driverVelo.plus(perpVelo.times(assistSpeed));

                speeds.vxMetersPerSecond = finalVelo.getX();
                speeds.vyMetersPerSecond = finalVelo.getY();
            }
        }

        speeds = ChassisSpeedsUtil.FromFieldToRobot(speeds,
                Rotation2d.fromDegrees(gyroDataSupplier.get().yaw - angleOffset));

                // speeds.vxMetersPerSecond = 0;
                // speeds.vyMetersPerSecond = 1;

    }

    @Override
    public void logController() {
        super.logController();
        MALog.log("Subsystems/Swerve/Controllers/Assist/Angle Offset", angleOffset);
    }

}
