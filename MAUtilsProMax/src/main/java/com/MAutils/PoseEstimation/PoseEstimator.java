package com.MAutils.PoseEstimation;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;
import java.util.NavigableMap;
import java.util.TreeMap;
import java.util.Map;


import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.MAutils.Logger.MALog;
import com.MAutils.Logger.TelemetryLogger;
import com.MAutils.PoseEstimation.Helpers.HistoryEntry;
import com.MAutils.PoseEstimation.Helpers.Measurement;
import com.MAutils.Utils.Constants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Util.Field;

public class PoseEstimator {

    private static final double MAX_TRANSLATION_VEL_MPS = 8.0; 
    private static final double MAX_ANGULAR_VEL_RADPS = 14.0; 

    private static final List<PoseEstimatorSource> sources = new ArrayList<>();

    private static SwerveDriveSimulation swerveSim = null;
    private static boolean outOfFieldBool = false;

    private static Pose2d poseBeforeHistory = new Pose2d();

    private static final NavigableMap<Double, HistoryEntry> history = new TreeMap<>();

    //private static final Deque<HistoryEntry> history = new ArrayDeque<>();


    private static Pose2d currentPose = new Pose2d();
    private static double lastUpdateTime;

    public static void setSwerveSim(SwerveDriveSimulation sim) {
        swerveSim = sim;
    }

    public static void resetOutOfFieldFlag() {
        outOfFieldBool = false;
    }

    public static boolean isOutOfFieldFlag() {
        return outOfFieldBool;
    }

    public static void resetPose(Pose2d newPose) {
        double now = Timer.getFPGATimestamp();
        history.clear();
        poseBeforeHistory = newPose;
        currentPose = newPose;
        lastUpdateTime = now;

        TelemetryLogger.logPoseEstimator(
            "Pose reset - X:" + newPose.getX()
            + " Y:" + newPose.getY()
            + " O:" + newPose.getRotation().getDegrees()
        );

        if (!Robot.isReal() && swerveSim != null) {
            swerveSim.setSimulationWorldPose(newPose);
        }
    }

    public static void addSource(PoseEstimatorSource src) {
        TelemetryLogger.logPoseEstimator("Added source: " + src.name);
        sources.add(src);
    }

    public static void update() {
        applyAtTime(Timer.getFPGATimestamp());
    }

    public static Pose2d getCurrentPose() {
        if (currentPose == null) {
            return new Pose2d(-1, -1, new Rotation2d());
        }
        return currentPose;
    }

    public static Pose2d getPoseLookAhead(double time, ChassisSpeeds speedsRobotRelative) {
        return currentPose.exp(
            new Twist2d(
                speedsRobotRelative.vxMetersPerSecond * time,
                speedsRobotRelative.vyMetersPerSecond * time,
                speedsRobotRelative.omegaRadiansPerSecond * time
            )
        );
    }

    public static Pose2d getPoseAt(double queryTime) {
        Pose2d pose = poseBeforeHistory;

        MALog.log("PE/poseBeforeHistory", pose);

        for (HistoryEntry entry :
            history.headMap(queryTime, true).values()) {

            pose = pose.exp(entry.twist);
        }

        return pose;

        // Pose2d pose = poseBeforeHistory;
        // for (HistoryEntry e : history) {
        //     if (e.time > queryTime) break;
        //     pose = pose.exp(e.twist);
        // }
        // return pose;
    }




    private static void applyAtTime(double timestamp) {
        //final double dt = Math.max(0.0, timestamp - lastUpdateTime);

        Twist2d fused = calculateTwist2d(timestamp);
        //fused = clampTwistByDt(fused, dt);

        Pose2d candidate = currentPose;
        Twist2d t = calculateTwist2d(timestamp);
        t.dtheta = Units.degreesToRadians(t.dtheta);
        candidate = candidate.exp(t);
        MALog.log("Pose Estimator/Candidate", candidate);

        MALog.log("PE/ isInHub", Field.HUB_BLUE.contains(candidate.getTranslation()) &&
            Field.HUB_RED.contains(candidate.getTranslation()));

        MALog.log("PE/ isInField", Field.ALLOWED_FIELD.contains(candidate.getTranslation()));


        if (Field.ALLOWED_FIELD.contains(candidate.getTranslation()) && 
            !Field.HUB_BLUE.contains(candidate.getTranslation()) &&
            !Field.HUB_RED.contains(candidate.getTranslation())) {
            
            currentPose = candidate; 
            //history.addLast(new HistoryEntry(timestamp, fused));
            history.put(timestamp, new HistoryEntry(timestamp, calculateTwist2d(timestamp)));

            lastUpdateTime = timestamp;

            cleanupHistory(Timer.getFPGATimestamp());
            MALog.log("Pose Estimator/Current Pose", currentPose);

        } else {
            outOfFieldBool = true;
            TelemetryLogger.logPoseEstimator("Update rejected: Pose is outside the field boundaries");
        }
    }

    private static Twist2d calculateTwist2d(double timestamp) {
        double sumFomXY = 0.0;
        double sumFomTheta = 0.0;

        double dx = 0.0;
        double dy = 0.0;
        double dTheta = 0.0;

        for (PoseEstimatorSource src : sources) {
            Measurement meas = src.getMeasurement(timestamp);
            if (meas != null) {

                double fXY = meas.fomXY;
                double fTh = meas.fomTheta;

                dx += meas.twist.dx * fXY;
                dy += meas.twist.dy * fXY;
                dTheta += meas.twist.dtheta * fTh;

                sumFomXY += fXY;
                sumFomTheta += fTh;
            }
        }

        final boolean hasXY = sumFomXY > Constants.MIN_FOM_VALUE;
        final boolean hasTh = sumFomTheta > Constants.MIN_FOM_VALUE;

        MALog.log("Pose Estimator/sumofFOM/XY", sumFomXY);
        MALog.log("Pose Estimator/sumofFOM/Theta", sumFomTheta);

        if (!hasXY || !hasTh) {
            return new Twist2d();
        }

        double outDx = (dx / sumFomXY) ;
        double outDy = (dy / sumFomXY);
        double outDTh =(dTheta / sumFomTheta);

        MALog.log("Pose Estimator/Total Twist/X", outDx);
        MALog.log("Pose Estimator/Total Twist/Y", outDy);
        MALog.log("Pose Estimator/Total Twist/Theta", outDTh);

        return new Twist2d(outDx, outDy, outDTh);
    }

    private static Twist2d clampTwistByDt(Twist2d t, double dt) {
        if (dt <= 0.0) return t;

        double maxDx = MAX_TRANSLATION_VEL_MPS * dt;
        double maxDy = MAX_TRANSLATION_VEL_MPS * dt;
        double maxDTheta = MAX_ANGULAR_VEL_RADPS * dt;

        double dx = Math.max(-maxDx, Math.min(maxDx, t.dx));
        double dy = Math.max(-maxDy, Math.min(maxDy, t.dy));
        double dtheta = Math.max(-maxDTheta, Math.min(maxDTheta, t.dtheta));

        return new Twist2d(dx, dy, dtheta);
    }

    private static void cleanupHistory(double currentTime) {
        double oldestAllowedTime =
        currentTime - PoseEstimatorSource.BUFFER_DURATION;

        if (history.isEmpty()) return;

        Double firstValidTime = history.ceilingKey(oldestAllowedTime);

        if (firstValidTime == null) {
            return;
        }

       Map.Entry<Double, HistoryEntry> lastRemoved = history.lowerEntry(firstValidTime);

        if (lastRemoved != null) poseBeforeHistory = getPoseAt(lastRemoved.getKey());


        history.headMap(firstValidTime, false).clear();
    }

    // private static void trimHistory() {
    //     double cutoff = lastUpdateTime - PoseEstimatorSource.BUFFER_DURATION;

    //     while (!history.isEmpty() && history.peekFirst().time < cutoff) {
    //         HistoryEntry e = history.removeFirst(); // explicit type (more readable here)
    //         poseBeforeHistory = poseBeforeHistory.exp(e.twist);
    //         //historyStartTime = e.time;
    //     }
    // }

}