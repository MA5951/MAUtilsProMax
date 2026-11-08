// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.MAutils.PoseEstimation;

import com.MAutils.Logger.MALog;
import com.MAutils.Logger.TelemetryLogger;
import com.MAutils.Swerve.SwerveSystem;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.MAutils.Swerve.Utils.CollisionDetector;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class SwerveDriveEstimator {

     private final double MAX_UPDATE_ANGLE = 5;
    private final double SKIP_ODOMETRY_Gs = 3;

    private SwerveDriveOdometry od;

    private Pose2d lastPose2d = new Pose2d(),currPose2d = new Pose2d();

    private Twist2d loopTwistSum = new Twist2d();

    private final SwerveSystem swerveSystem;
    private final CollisionDetector collisionDetector;
    private final PoseEstimatorSource odometrySource;

    private boolean isCollision;
    private boolean isTilted;



    private SwerveSystemConstants constants;

    public SwerveDriveEstimator(SwerveSystemConstants swerveConstants, SwerveSystem swerveSystem) {
        this.swerveSystem = swerveSystem;

        this.collisionDetector = new CollisionDetector(swerveSystem::getGyroData);

        this.odometrySource = new PoseEstimatorSource("Swerve Odometry", getTranslationFOM(), getRotationFOM());

        this.constants = swerveConstants;

        od = new SwerveDriveOdometry(constants.kinematics, swerveSystem.getRobotRotation2d(), swerveSystem.getCurrentPositions());

        PoseEstimator.addSource(odometrySource);

        lastPose2d= new Pose2d();

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

    public void reasetPose(Pose2d newP) {
        od.resetPose(newP);
    }

    public void update() {
        isCollision = isCollisionDetected();
        isTilted = isTilted();
        MALog.log("PE/swerve/isCollisionDetected", isCollision);
        MALog.log("PE/swerve/isTilted", isTilted);

        od.update(swerveSystem.getRobotRotation2d(),swerveSystem.getCurrentPositions());

        if(!isCollision && ! isTilted) {
            od.update(swerveSystem.getRobotRotation2d(),swerveSystem.getCurrentPositions());
            currPose2d = od.getPoseMeters();
            MALog.log("PE/swerve/pose swerve", currPose2d);
            loopTwistSum.dx = currPose2d.getX()-lastPose2d.getX();
            loopTwistSum.dy = currPose2d.getY()-lastPose2d.getY();
            loopTwistSum.dtheta = swerveSystem.getGyroDelta();
            lastPose2d = od.getPoseMeters();

        } else {
            loopTwistSum.dx = 0;
            loopTwistSum.dy = 0;
            loopTwistSum.dtheta = 0;
            if(isCollision) {
                TelemetryLogger.logSwerve("Ignoring odometry data, collision detected");
            }
            if(isTilted) {
                TelemetryLogger.logSwerve("Ignoring odometry data, tilt detected");
            }
        }

        odometrySource.capture(loopTwistSum, Timer.getFPGATimestamp(), getRotationFOM(), getTranslationFOM(), od.getPoseMeters());
    }
}
