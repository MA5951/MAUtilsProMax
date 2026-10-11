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
public class SwerveDriveEstimator {
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

    // private Twist2d getTranslationDelta(SwerveModulePosition[] currentPositions) {
    //     totalDelta = Translation2d.kZero; 
    //     numOfSkiddingModules = 0;

        

    //     // if (skidDetector.getNumOfSkiddingModules() >= 2) {
    //     //     totalDelta = totalDelta.plus(calculateModuleDisplysment(lastPositions[skidDetector.getLowestIndex()], currentPositions[skidDetector.getLowestIndex()]));
    //     //     totalDelta = totalDelta.plus(calculateModuleDisplysment(lastPositions[skidDetector.getSecoundLowestIndex()], currentPositions[skidDetector.getSecoundLowestIndex()]));
    //     //     numOfSkiddingModules = 2;
    //     // } else {
    //         for (int i = 0; i < currentPositions.length; i++) {
    //             deltaDistance = currentPositions[i].distanceMeters - lastPositions[i].distanceMeters;
    //             prevAngle = lastPositions[i].angle;
    //             currAngle = currentPositions[i].angle;
    //             deltaTheta = currAngle.minus(prevAngle).getRadians();

    //             if (Math.abs(deltaTheta) < 1e-5) { //TODO you duplicate code her just call calculateModuleDisplysment()
    //                 arcDelta = new Translation2d(deltaDistance, currAngle); 
    //             } else {
    //                 Translation2d v1 = new Translation2d(deltaDistance / deltaTheta,
    //                         prevAngle.minus(Rotation2d.fromRadians(Math.PI / 2)));
    //                 Translation2d v2 = v1.rotateBy(Rotation2d.fromRadians(deltaTheta));

    //                 arcDelta = v2.minus(v1);
    //             }
    //             totalDelta = totalDelta.plus(arcDelta);

    //             // if (!skidDetector.getIsSkidding()[i] && numOfSkiddingModules < 2) { // TODO you dont need to check numOfSkiddingModules < 2 its in the else
    //             //     totalDelta = totalDelta.plus(arcDelta);
    //             //     numOfSkiddingModules++; //TODO why you add one? why you dont just use getNumOfSkiddingModules()
    //             // }

    //         }
    //    // }

    //     lastPositions = currentPositions;

        

    //     return new Twist2d(
    //             totalDelta.getX() / (4 - numOfSkiddingModules), //TODO change the 4 to currentPositions.length
    //             totalDelta.getY() / (4 - numOfSkiddingModules),
    //             0); //TODO why you dont just edite the odometryTwist her
    // }

    public void updateOdometry() {
        MALog.log("PE/ swerve/isCollisionDetected", isCollisionDetected());
        MALog.log("PE/ swerve/isTilted", isTilted());


        if(!isCollisionDetected() ) {
        

            loopTwistSum.dx = 0;
            loopTwistSum.dy = 0;
            loopTwistSum.dtheta = 0;

            // sampleTimestamps = swerveSystem.getGyroData().odometryYawTimestamps;
            // for (int i = 0; i < sampleTimestamps.length; i++) {
            //     SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4]; // TODO its better to do the new outsid of the loop
            //     for (int j = 0; j < 4; j++) { //TODO change to wheelPositions.length
            //         wheelPositions[j] = swerveSystem.getSwerveModules()[j].getOdometryPositions()[i];

            //     }


            //     odometryTwist = swerveSystem.getTwist2d(wheelPositions);

            //     loopTwistSum.dx += odometryTwist.dx;
            //     loopTwistSum.dy += odometryTwist.dy;
            //     loopTwistSum.dtheta += odometryTwist.dtheta;
            //     totalYdelta += loopTwistSum.dy;


            // }

            loopTwistSum = swerveSystem.getTwist2d(swerveSystem.getCurrentPositions());
            totalYdelta += loopTwistSum.dy;



            // sampleTimestamps = swerveSystem.getGyroData().odometryYawTimestamps;
            // SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
            // for (int i = 0; i < sampleTimestamps.length; i++) {
            //     for (int j = 0; j < wheelPositions.length; j++) { 
                    //wheelPositions[j] = swerveSystem.getSwerveModules()[j].getOdometryPositions()[i];
            //         MALog.log("PE/MODULE POSE/" + j, wheelPositions[j].angle.getDegrees());
            //     }                    
            // }         //MALog.log("PE/Swerve/wheelPositions", wheelPositions[j].distanceMeters);

                    // odometryTwist = swerveSystem.getTwist2d(wheelPositions);

            //         loopTwistSum.dx += odometryTwist.dx;
            //         loopTwistSum.dy += odometryTwist.dy;

            //         //odometryTwist.dtheta = swerveSystem.getGyroDelta();// TODO CHNAGE TO GYRO HIGH ODOMETRY TRED

            //      MALog.log("Pose Estimation/ swerve/small twisted", loopTwistSum.dy);
            // }
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