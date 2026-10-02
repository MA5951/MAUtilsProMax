package com.MAutils.PoseEstimation;

import com.MAutils.Logger.MALog;
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
public class SwerveDriveEstimator {
    private final double MAX_UPDATE_ANGLE = 5;
    private final double SKIP_ODOMETRY_Gs = 3;


    private final SwerveSystem swerveSystem;
    private final CollisionDetector collisionDetector;
    private double[] sampleTimestamps;
    private final PoseEstimatorSource odometrySource;
    private Twist2d loopTwistSum = new Twist2d(), odometryTwist;
   


    public SwerveDriveEstimator(SwerveSystemConstants swerveConstants, SwerveSystem swerveSystem) {
        this.swerveSystem = swerveSystem;

        this.collisionDetector = new CollisionDetector(swerveSystem::getGyroData);

        this.odometrySource = new PoseEstimatorSource("Swerve Odometry", getTranslationFOM(), getRotationFOM());

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

    public void updateOdometry() {
        MALog.log("PE/ swerve/isCollisionDetected", isCollisionDetected());
        MALog.log("PE/ swerve/isTilted", isTilted());


        if(!isCollisionDetected() ) {
            loopTwistSum.dx = 0;
            loopTwistSum.dy = 0;
            loopTwistSum.dtheta = 0;

            sampleTimestamps = swerveSystem.getGyroData().odometryYawTimestamps;
            SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
            for (int i = 0; i < sampleTimestamps.length; i++) {
                for (int j = 0; j < wheelPositions.length; j++) { 
                    wheelPositions[j] = swerveSystem.getSwerveModules()[j].getOdometryPositions()[i];
                    MALog.log("PE/MODULE POSE/" + j, wheelPositions[j].angle.getDegrees());
                }                    
                    //MALog.log("PE/Swerve/wheelPositions", wheelPositions[j].distanceMeters);

                    odometryTwist = swerveSystem.getTwist2d(wheelPositions);

                    loopTwistSum.dx += odometryTwist.dx;
                    loopTwistSum.dy += odometryTwist.dy;

                    //odometryTwist.dtheta = swerveSystem.getGyroDelta();// TODO CHNAGE TO GYRO HIGH ODOMETRY TRED

                 MALog.log("Pose Estimation/ swerve/small twisted", loopTwistSum.dy);
            }
        } else {
            if(isCollisionDetected()) {
                TelemetryLogger.logSwerve("Ignoring odometry data, collision detected");
            }
            if(isTilted()) {
                TelemetryLogger.logSwerve("Ignoring odometry data, tilt detected");
            }
            

            loopTwistSum.dx = 0;
            loopTwistSum.dy = 0;
            loopTwistSum.dtheta = 0;
        }

        odometryTwist.dtheta = swerveSystem.getGyroDelta();
        loopTwistSum.dtheta = odometryTwist.dtheta;


        odometrySource.capture(loopTwistSum, Timer.getFPGATimestamp());
    }

}