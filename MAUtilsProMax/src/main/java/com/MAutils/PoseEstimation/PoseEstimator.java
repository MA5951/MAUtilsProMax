package com.MAutils.PoseEstimation;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.List;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

import com.MAutils.Logger.MALog;
import com.MAutils.Logger.TelemetryLogger;
import com.MAutils.Vision.Filters.FiltersConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.Util.Field;

/**
 * PoseEstimator holds any number of PoseEstimatorSource,
 * fuses them each loop by XY/θ FOM-weighted averages, and corrects for latency
 * by replaying a short history when late measurements arrive.
 */
public class PoseEstimator {
    private static final double HISTORY_WINDOW_SEC = 0.5; // must exceed max latency

    // Numerical guard for FOM sums
    private static final double FOM_EPS = 1e-9;

    /**
     * Motion sanity limits (used to clamp per-step twist when integrating).
     * Consider wiring these to drivetrain constants.
     */
    private static final double MAX_TRANSLATION_VEL_MPS = 8.0;  // typical fast swerve ballpark
    private static final double MAX_ANGULAR_VEL_RADPS = 14.0;    // ~800 deg/s ballpark

    private static final List<PoseEstimatorSource> sources = new ArrayList<>();

    private static SwerveDriveSimulation swerveSim = null;
    private static boolean outOfFieldBool = false;

    // Pose and time just before our replay buffer begins
    private static Pose2d poseBeforeHistory = new Pose2d();
    private static double historyStartTime;

    // Recent applied twists, for replay
    private static final Deque<HistoryEntry> history = new ArrayDeque<>();

    private static Pose2d currentPose = new Pose2d();
    private static double lastUpdateTime;

    /**
     * Optional hook: if a source supports pruning, we can ask it to drop
     * measurements older than our history window.
     */
    public interface PrunablePoseEstimatorSource {
        void pruneBefore(double timestamp);
    }

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
        historyStartTime = now;
        currentPose = newPose;
        lastUpdateTime = now;

        TelemetryLogger.logPoseEstimator(
                "Pose reset - X:" + newPose.getX()
                        + " Y:" + newPose.getY()
                        + " R:" + newPose.getRotation().getDegrees());

        if (!Robot.isReal() && swerveSim != null) {
            swerveSim.setSimulationWorldPose(newPose);
        }
    }

    /** Add odometry / limelight / any other source */
    public static void addSource(PoseEstimatorSource src) {
        TelemetryLogger.logPoseEstimator("Added source: " + src.name);
        sources.add(src);
    }

    /** Call once per robot loop. */
    public static void update() {
        final double now = Timer.getFPGATimestamp();

        // Ask sources to prune anything older than our history window (optional).
        pruneSources(now - HISTORY_WINDOW_SEC - 0.05); // small cushion

        // If any source has a measurement stamped before our last update, we need to replay.
        // NOTE: This assumes sources only report "unconsumed" late packets via hasBefore().
        // If a source keeps old packets forever and hasBefore() doesn't account for consumption,
        // it should be fixed inside that source (or implement PrunablePoseEstimatorSource).
        // if (hasLatePackets()) {
        //     replayHistory();
        // }

        // Apply one step at 'now'
        applyAtTime(now, history, true);

        // Trim old history
        trimHistory();
    }

    public static Pose2d getCurrentPose() {
        if (currentPose == null) {
            return new Pose2d(-1, -1, new Rotation2d());
        }
        return currentPose;
    }

    /** Backward-compatible overall FOM (mean of XY and θ across sources). */
    public static double getRobotFOM() {
        return sources.stream()
                .mapToDouble(s -> s.getFomAt(lastUpdateTime))
                .average()
                .orElse(0.0);
    }

    /** New: average XY FOM across sources at the last update. */
    public static double getRobotFOMXY() {
        return sources.stream()
                .mapToDouble(s -> s.getFomXYAt(lastUpdateTime))
                .average()
                .orElse(0.0);
    }

    public static Pose2d getPoseLookAhead(double time, ChassisSpeeds speedsRobotRelativ) {
        return PoseEstimator.getCurrentPose().exp(
                new Twist2d(
                        speedsRobotRelativ.vxMetersPerSecond * time,
                        speedsRobotRelativ.vyMetersPerSecond * time,
                        speedsRobotRelativ.omegaRadiansPerSecond * time));
    }

    // —— internal —— //

    /** Pose reconstructed by applying history up to a query time. */
    public static Pose2d getPoseAt(double queryTime) {
        Pose2d pose = poseBeforeHistory;
        for (HistoryEntry e : history) {
            if (e.time > queryTime) break;
            pose = pose.exp(e.twist);
        }
        return pose;
    }

    /** True if any source claims it has a packet older than lastUpdateTime (late/out-of-order). */
    private static boolean hasLatePackets() {
        return sources.stream().anyMatch(s -> s.hasBefore(lastUpdateTime));
    }

    private static void pruneSources(double pruneBeforeTime) {
        for (PoseEstimatorSource src : sources) {
            if (src instanceof PrunablePoseEstimatorSource) {
                ((PrunablePoseEstimatorSource) src).pruneBefore(pruneBeforeTime);
            }
        }
    }

    /**
     * Replay from poseBeforeHistory through all history entries, slotting in any late packets.
     */
    private static void replayHistory() {
        // We assume 'history' timestamps are monotonic-increasing. If that invariant is ever broken,
        // replay should sort (but we avoid allocations/sorts unless needed).
        Pose2d pose = poseBeforeHistory;
        Deque<HistoryEntry> newHist = new ArrayDeque<>();

        // Rebuild by re-computing fused twists at each stored history timestamp
        for (HistoryEntry old : history) {
            // Temporarily set currentPose so applyAtTime() uses correct base pose while rebuilding
            currentPose = pose;
            applyAtTime(old.time, newHist, false);
            pose = currentPose;
        }

        MALog.log("Pose Estimator/Replayed Pose", pose);

        // Validate reconstructed pose
        if (Field.ALLOWED_FIELD.contains(pose.getTranslation()) && 
        !Field.HUB_BLUE.contains(pose.getTranslation()) &&
        !Field.HUB_RED.contains(pose.getTranslation())) {
            history.clear();
            history.addAll(newHist);
            currentPose = pose;

            HistoryEntry last = history.peekLast();
            lastUpdateTime = (last == null) ? historyStartTime : last.time;
            

            // Keep window sane after replay
            trimHistory();
        } else {
            outOfFieldBool = true;
            TelemetryLogger.logPoseEstimator(
                    "Update rejected in replay because pose is outside the field");
        }
    }

    /**
     * Applies the fused twist at timestamp t onto currentPose.
     *
     * @param timestamp  integration time
     * @param targetHist where to write the history entry (can be a temp deque during replay)
     * @param logNowPose whether to log the final pose
     */
    private static void applyAtTime(double timestamp, Deque<HistoryEntry> targetHist, boolean logNowPose) {
        final double dt = Math.max(0.0, timestamp - lastUpdateTime);

        Twist2d fused = computeFusedTwist(timestamp);

        // Sanity clamp per-step motion to something physically plausible
        fused = clampTwistByDt(fused, dt);

        Pose2d candidate = currentPose.exp(fused);
        MALog.log("Pose Estimator/Canidate", candidate);

        if (Field.ALLOWED_FIELD.contains(candidate.getTranslation()) && 
        !Field.HUB_BLUE.contains(candidate.getTranslation()) &&
        !Field.HUB_RED.contains(candidate.getTranslation())) {
            currentPose = candidate;
            targetHist.addLast(new HistoryEntry(timestamp, fused));
            lastUpdateTime = timestamp;

            if (logNowPose) {
                MALog.log("Pose Estimator/Current Pose", currentPose);
            }
        } else {
            outOfFieldBool = true;
            TelemetryLogger.logPoseEstimator("Update rejected at apply at time because pose is outside the field");
        }
    }

    private static Twist2d clampTwistByDt(Twist2d t, double dt) {
        if (dt <= 0.0) return t;

        double maxDx = MAX_TRANSLATION_VEL_MPS * dt;
        double maxDy = MAX_TRANSLATION_VEL_MPS * dt;
        double maxDTheta = MAX_ANGULAR_VEL_RADPS * dt;

        double dx = clamp(t.dx, -maxDx, maxDx);
        double dy = clamp(t.dy, -maxDy, maxDy);
        double dtheta = clamp(t.dtheta, -maxDTheta, maxDTheta);

        return new Twist2d(dx, dy, dtheta);
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    /**
     * Weighted average of each source’s twist at exactly timestamp T
     * (translation weighted by XY FOM; rotation weighted by θ FOM).
     */
    private static Twist2d computeFusedTwist(double timestamp) {
        double sumFomXY = 0.0;
        double sumFomTheta = 0.0;

        double dxAcc = 0.0;
        double dyAcc = 0.0;
        double dThetaAcc = 0.0;

        for (PoseEstimatorSource src : sources) {
            Twist2d tt = src.getTwistAt(timestamp);
            if (tt == null) continue;

            double fXY = src.getFomXYAt(timestamp);
            double fTh = src.getFomThetaAt(timestamp);

            // Defensive: ignore negative/NaN FOMs
            if (!(fXY > 0.0)) fXY = 0.0;
            if (!(fTh > 0.0)) fTh = 0.0;

            dxAcc += tt.dx * fXY;
            dyAcc += tt.dy * fXY;
            dThetaAcc += tt.dtheta * fTh;

            sumFomXY += fXY;
            sumFomTheta += fTh;
        }

        // Avoid division by ~0; allow translation-only or rotation-only fusion
        final boolean hasXY = sumFomXY > FOM_EPS;
        final boolean hasTh = sumFomTheta > FOM_EPS;

        if (!hasXY && !hasTh) {
            return new Twist2d();
        }

       
        MALog.log("Pose Estimator/sumofFOM/XY", sumFomXY);
        MALog.log("Pose Estimator/sumofFOM/Theta", sumFomTheta);

        double outDx = hasXY ? (dxAcc / sumFomXY) : 0.0;
        double outDy = hasXY ? (dyAcc / sumFomXY) : 0.0;
        double outDTh = hasTh ? (dThetaAcc / sumFomTheta) : 0.0;

        MALog.log("Pose Estimator/Total Twist/X", outDx);
        MALog.log("Pose Estimator/Total Twist/Y", outDy);
        MALog.log("Pose Estimator/Total Twist/Theta", outDTh);

        return new Twist2d(outDx, outDy, outDTh);
    }

    /** Drop anything older than our window and roll poseBeforeHistory forward. */
    private static void trimHistory() {
        double cutoff = lastUpdateTime - HISTORY_WINDOW_SEC;

        while (!history.isEmpty() && history.peekFirst().time < cutoff) {
            HistoryEntry e = history.removeFirst(); // explicit type (more readable here)
            poseBeforeHistory = poseBeforeHistory.exp(e.twist);
            historyStartTime = e.time;
        }
    }

    private static class HistoryEntry {
        final double time;
        final Twist2d twist;

        HistoryEntry(double t, Twist2d tw) {
            time = t;
            twist = tw;
        }
    }
}
