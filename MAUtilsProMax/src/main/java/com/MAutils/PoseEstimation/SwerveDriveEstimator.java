package com.MAutils.PoseEstimation;

import com.MAutils.Logger.TelemetryLogger;
import com.MAutils.Swerve.SwerveSystem;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.MAutils.Swerve.Utils.CollisionDetector;
import com.MAutils.Swerve.Utils.SkidDetector;
import com.MAutils.Utils.Constants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Subsystems.Swerve.Swerve;

/*
 * Estimates the robot's pose using swerve drive odometry.
 */
public class SwerveDriveEstimator {
    private final double MAX_UPDATE_ANGLE = 5;
    private final double SKIP_ODOMETRY_Gs = 3;

    private SwerveModulePosition[] lastPositions = new SwerveModulePosition[] {
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d()),
            new SwerveModulePosition(0, new Rotation2d())
    };

    private Rotation2d lastGyroRotation, prevAngle, currAngle;
    private final SwerveSystem swerveSystem;
    private double gyroDelta, deltaDistance, deltaTheta;
    private final SkidDetector skidDetector;
    private final CollisionDetector collisionDetector;
    private double[] sampleTimestamps;
    private final PoseEstimatorSource odometrySource;
    private Twist2d loopTwistSum = new Twist2d(), odometryTwist;
    private Translation2d totalDelta = new Translation2d(), arcDelta;
    private int numOfSkiddingModules = 0;


    public SwerveDriveEstimator(SwerveSystemConstants swerveConstants, SwerveSystem swerveSystem) {
        this.swerveSystem = swerveSystem;
        this.lastGyroRotation = Rotation2d.fromDegrees(swerveSystem.getGyroData().yaw);

        this.skidDetector = new SkidDetector(swerveConstants, swerveSystem::getCurrentStates);
        this.collisionDetector = new CollisionDetector(swerveSystem::getGyroData);

        this.odometrySource = new PoseEstimatorSource("Swerve Odometry",
                () -> loopTwistSum, () -> getTranslationFOM(), () -> getRotationFOM(), () -> Timer.getFPGATimestamp());

        PoseEstimator.addSource(odometrySource);

    }

    // Deltas
    private Twist2d getTranslationDelta(SwerveModulePosition[] currentPositions) {
        totalDelta = new Translation2d(); //TODO is better to 0 the values then creat a new one totalDelta = vector zero;
        numOfSkiddingModules = 0;

        // if (skidDetector.getNumOfSkiddingModules() >= 2) {
        //     totalDelta = totalDelta.plus(calculateModuleDisplysment(lastPositions[skidDetector.getLowestIndex()], currentPositions[skidDetector.getLowestIndex()]));
        //     totalDelta = totalDelta.plus(calculateModuleDisplysment(lastPositions[skidDetector.getSecoundLowestIndex()], currentPositions[skidDetector.getSecoundLowestIndex()]));
        //     numOfSkiddingModules = 2;
        // } else {
            for (int i = 0; i < currentPositions.length; i++) {
                deltaDistance = currentPositions[i].distanceMeters - lastPositions[i].distanceMeters;
                prevAngle = lastPositions[i].angle;
                currAngle = currentPositions[i].angle;
                deltaTheta = currAngle.minus(prevAngle).getRadians();

                if (Math.abs(deltaTheta) < 1e-5) { //TODO you duplicate code her just call calculateModuleDisplysment()
                    arcDelta = new Translation2d(deltaDistance, currAngle); 
                } else {
                    Translation2d v1 = new Translation2d(deltaDistance / deltaTheta,
                            prevAngle.minus(Rotation2d.fromRadians(Math.PI / 2)));
                    Translation2d v2 = v1.rotateBy(Rotation2d.fromRadians(deltaTheta));

                    arcDelta = v2.minus(v1);
                }
                totalDelta = totalDelta.plus(arcDelta);

                // if (!skidDetector.getIsSkidding()[i] && numOfSkiddingModules < 2) { // TODO you dont need to check numOfSkiddingModules < 2 its in the else
                //     totalDelta = totalDelta.plus(arcDelta);
                //     numOfSkiddingModules++; //TODO why you add one? why you dont just use getNumOfSkiddingModules()
                // }

            }
       // }

        lastPositions = currentPositions;

        

        return new Twist2d(
                totalDelta.getX() / (4 - numOfSkiddingModules), //TODO change the 4 to currentPositions.length
                totalDelta.getY() / (4 - numOfSkiddingModules),
                0); //TODO why you dont just edite the odometryTwist her
    }

    private double getGyroDelta(Rotation2d currentGyro) {
        gyroDelta = currentGyro.minus(lastGyroRotation).getRadians();
        lastGyroRotation = currentGyro;
        return gyroDelta;
    }

    private void updateOdometryTwist(SwerveModulePosition[] currentPositions, Rotation2d currentGyro) {
        odometryTwist = getTranslationDelta(currentPositions); //TODO this is just a midelman func you can delte it
        odometryTwist.dtheta = getGyroDelta(currentGyro);
    }
    //TODO add func that return the odometryTwist


    private double getTranslationFOM() {
        return 1;
    }

    private double getRotationFOM() {
        return 1;
    }

    // Update
    public void updateOdometry() {
        //skidDetector.calculateSkid();
        collisionDetector.calculateCollision();

        if (collisionDetector.getForceVector() < SKIP_ODOMETRY_Gs 
                && swerveSystem.getTiltAngle() < MAX_UPDATE_ANGLE) {

            loopTwistSum.dx = 0;
            loopTwistSum.dy = 0;
            loopTwistSum.dtheta = 0;

            sampleTimestamps = swerveSystem.getGyroData().odometryYawTimestamps;
            for (int i = 0; i < sampleTimestamps.length; i++) {
                SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4]; // TODO its better to do the new outsid of the loop
                for (int j = 0; j < 4; j++) { //TODO change to wheelPositions.length
                    wheelPositions[j] = swerveSystem.getSwerveModules()[j].getOdometryPositions()[i];

                }

                updateOdometryTwist(wheelPositions, swerveSystem.getGyroData().odometryYawPositions[i]);

                loopTwistSum.dx += odometryTwist.dx;
                loopTwistSum.dy += odometryTwist.dy;
                loopTwistSum.dtheta += odometryTwist.dtheta;

            }

            odometrySource.capture();

            TelemetryLogger.logSwerve("Odometry Captured");

        } else {
            loopTwistSum.dx = 0;
            loopTwistSum.dy = 0;
            loopTwistSum.dtheta = 0;
            odometrySource.capture();
            TelemetryLogger.logSwerve("Ignoring odometry data, collision or tilt detected");
        }
    }

    private Translation2d calculateModuleDisplysment(SwerveModulePosition lastPosition,
            SwerveModulePosition currentPosition) {
        deltaDistance = currentPosition.distanceMeters - lastPosition.distanceMeters;
        prevAngle = lastPosition.angle;
        currAngle = currentPosition.angle;
        deltaTheta = currAngle.minus(prevAngle).getRadians();

        if (Math.abs(deltaTheta) < Constants.EPSILON) { 
            arcDelta = new Translation2d(deltaDistance, currAngle); //TODO its better to zero the vector instade of creant a new one
        } else {
            Translation2d v1 = new Translation2d(deltaDistance / deltaTheta,
                    prevAngle.minus(Rotation2d.fromRadians(Math.PI / 2))); //TODO its better to zero the vector instade of creant a new one
            Translation2d v2 = v1.rotateBy(Rotation2d.fromRadians(deltaTheta));

            arcDelta = v2.minus(v1);
        }

        return arcDelta;
    }

}