package com.MAutils.PoseEstimation;

import com.MAutils.Logger.MALog;
import com.MAutils.Logger.TelemetryLogger;
import com.MAutils.Swerve.SwerveSystem;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.MAutils.Swerve.Utils.CollisionDetector;
import com.MAutils.Swerve.Utils.SkidDetector;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;


/*
 * Estimates the robot's pose using swerve drive odometry.
 */
public class SwerveDriveEstimato1 {

    private final double MAX_UPDATE_ANGLE = 5;
    private final double SKIP_ODOMETRY_Gs = 3;
    private double totalYdelta = 0;


    private final SwerveSystem swerveSystem;
    private final CollisionDetector collisionDetector;
    private double[] sampleTimestamps;
    private final PoseEstimatorSource odometrySource;
    private Twist2d loopTwistSum = new Twist2d(), odometryTwist =  new Twist2d();


    private Rotation2d lastGyroRotation, prevAngle, currAngle;
    private double gyroDelta, deltaDistance, deltaTheta;
    private Translation2d totalDelta = new Translation2d(), arcDelta;
    private int numOfSkiddingModules = 0;

    private SwerveModulePosition[] lastPositions = new SwerveModulePosition[] {
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d())
    };
   


    public SwerveDriveEstimato1(SwerveSystemConstants swerveConstants, SwerveSystem swerveSystem) {
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


        if(!isCollisionDetected() && ! isTilted()) {
        

            loopTwistSum.dx = 0;
            loopTwistSum.dy = 0;
            loopTwistSum.dtheta = 0;

            loopTwistSum = swerveSystem.getTwist2d(swerveSystem.getCurrentPositions());
            totalYdelta += loopTwistSum.dy;


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


        odometrySource.capture(loopTwistSum, Timer.getFPGATimestamp(), getRotationFOM(), getTranslationFOM());
    }

}