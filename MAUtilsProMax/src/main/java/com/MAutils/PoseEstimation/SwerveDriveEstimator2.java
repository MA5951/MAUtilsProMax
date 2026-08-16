package com.MAutils.PoseEstimation;

import com.MAutils.Logger.TelemetryLogger;
import com.MAutils.Swerve.SwerveSystem;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.MAutils.Swerve.Utils.CollisionDetector;

import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;


/*
 * Estimates the robot's pose using swerve drive odometry.
 */
public class SwerveDriveEstimator2 {
    private final double MAX_UPDATE_ANGLE = 5;
    private final double SKIP_ODOMETRY_Gs = 3;
    private final double MAX_ANGULAR_VELOCITY = (150);
    private final double MAX_LINEAR_VELOCITY = 3; // Maximum linear velocity in meters per second   


    private final SwerveSystem swerveSystem;
    private final CollisionDetector collisionDetector;
    private double[] sampleTimestamps;
    private final PoseEstimatorSource odometrySource;
    private Twist2d loopTwistSum = new Twist2d(), odometryTwist;
   


    public SwerveDriveEstimator2(SwerveSystemConstants swerveConstants, SwerveSystem swerveSystem) {
        this.swerveSystem = swerveSystem;

        this.collisionDetector = new CollisionDetector(swerveSystem::getGyroData);

        this.odometrySource = new PoseEstimatorSource("Swerve Odometry",
                () -> loopTwistSum, () -> getTranslationFOM(), () -> getRotationFOM(), () -> Timer.getFPGATimestamp());

        PoseEstimator.addSource(odometrySource);

    }

    private double getTranslationFOM() {
        return 1;
    }

    private double getRotationFOM() {
        return 1;
    }

    private boolean isCollisionDetected() {
        collisionDetector.calculateCollision();
        return collisionDetector.getForceVector() >= SKIP_ODOMETRY_Gs;
    }

    private boolean isTilted() {
        return swerveSystem.getTiltAngle() >= MAX_UPDATE_ANGLE;
    }

    private boolean isAngularVelocityTooHigh() {
        return Math.abs(swerveSystem.getGyroData().yawVelocity) > MAX_ANGULAR_VELOCITY;
    }

    private boolean isLinearVelocityTooHigh() {
        return Math.hypot(swerveSystem.getChassisSpeeds().vxMetersPerSecond, swerveSystem.getChassisSpeeds().vyMetersPerSecond) > MAX_LINEAR_VELOCITY;
    }

    public void updateOdometry() {
        if(!isCollisionDetected() && !isTilted() && !isAngularVelocityTooHigh() && !isLinearVelocityTooHigh()) {
            loopTwistSum.dx = 0;
            loopTwistSum.dy = 0;
            loopTwistSum.dtheta = 0;

            sampleTimestamps = swerveSystem.getGyroData().odometryYawTimestamps;
            SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
            for (int i = 0; i < sampleTimestamps.length; i++) {
                for (int j = 0; j < wheelPositions.length; j++) { 
                    wheelPositions[j] = swerveSystem.getSwerveModules()[j].getOdometryPositions()[i];
                }

                odometryTwist = swerveSystem.getTranslationDelta(wheelPositions);
                odometryTwist.dtheta = swerveSystem.getGyroDelta( swerveSystem.getGyroData().odometryYawPositions[i]);

                loopTwistSum.dx += odometryTwist.dx;
                loopTwistSum.dy += odometryTwist.dy;
                loopTwistSum.dtheta += odometryTwist.dtheta;
            }
        } else {
            if(isCollisionDetected()) {
                TelemetryLogger.logSwerve("Ignoring odometry data, collision detected");
            }
            if(isTilted()) {
                TelemetryLogger.logSwerve("Ignoring odometry data, tilt detected");
            }
            if(isAngularVelocityTooHigh()) {
                TelemetryLogger.logSwerve("Ignoring odometry data, angular velocity too high");
            }
            if(isLinearVelocityTooHigh()) {
                TelemetryLogger.logSwerve("Ignoring odometry data, linear velocity too high");
            }

            loopTwistSum.dx = 0;
            loopTwistSum.dy = 0;
            loopTwistSum.dtheta = 0;
        }

        odometrySource.capture();
    }

}