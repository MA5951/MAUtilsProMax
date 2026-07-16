package com.MAutils.Vision.Filters;

import java.util.function.Supplier;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.Vision.IOs.VisionCameraIO;
import com.MAutils.Vision.Util.LimelightHelpers.PoseEstimate;
import com.MAutils.Vision.Util.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class AprilTagFilters {

    private FiltersConfig config;
    private final VisionCameraIO visionCameraIO;
    private final Supplier<ChassisSpeeds> chassisSpeeds;
    private final Supplier<Double> imuYawRadSupplier;
    private final Supplier<Double> imuYawVelocitySupplier;

    // Cache latest data to ensure XY and Omega use the same frame
    private PoseEstimate lastEstimate;
    private Pose2d lastPose = new Pose2d();
    private RawFiducial lastTag;
    private double lastCaptureTime, fomCof;

    public AprilTagFilters(FiltersConfig config, 
                           VisionCameraIO visionCameraIO, 
                           Supplier<ChassisSpeeds> chassisSpeedsSupplier, 
                           Supplier<Double> imuYawRadSupplier,
                           Supplier<Double> imuYawVelocitySupplier, double fomCOF) {
        this.config = config;
        this.visionCameraIO = visionCameraIO;
        this.chassisSpeeds = chassisSpeedsSupplier;
        this.imuYawRadSupplier = imuYawRadSupplier;
        this.imuYawVelocitySupplier = imuYawVelocitySupplier;
        this.fomCof = fomCOF;
    }

    /** Updates the internal state with the latest camera data. Call this once per loop. */
    public void update() {
        this.lastEstimate = visionCameraIO.getPoseEstimate(config.poseEstimateType);
        this.lastTag = visionCameraIO.getTag();
        if (lastEstimate != null) {
            this.lastCaptureTime = Timer.getFPGATimestamp() - (lastEstimate.latency / 1000.0);
        }
    }

    /** Calculates FOM for XY Translation (0..1) */
    public double getXyFOM() {
        if (!isBasicValid()) return 0.0;

        if (lastPose.getX() == 0) {
            return 0.6;
        }

        // 1. Motion Gate (Innovation)
        //double motionTrust = calculateMotionTrust();
        // if (motionTrust <= 0) return 0.0;

        // 2. Geometric / Range Trust
        double geomTrust = calculateGeometricTrust();

        // 3. Final Combined XY FOM
        return clamp01(( geomTrust) / 2 ) * fomCof;//m
    }

    /** Calculates FOM for Omega Rotation (0..1) */
    public double getOFOM() {
        

        

        // Since you are certain in your angle, we compare Vision Yaw vs IMU Yaw
        double visionYaw = lastEstimate.pose.getRotation().getRadians();
        double imuYaw = imuYawRadSupplier.get();
        double yawDiffDeg = Math.toDegrees(Math.abs(MathUtil.angleModulus(visionYaw - imuYaw)));

        if (yawDiffDeg >= config.hardYawGateDeg) return 0.0;

        // Smooth drop-off for orientation trust
        double trustTheta = 1.0;
        if (yawDiffDeg > config.softYawFullTrustDeg) {
            double norm = (yawDiffDeg - config.softYawFullTrustDeg) / 
                          (config.hardYawGateDeg - config.softYawFullTrustDeg);
            trustTheta = 0.5 * (1.0 + Math.cos(Math.PI * norm));
        }

        lastPose = lastEstimate.pose;

        return clamp01(trustTheta * calculateGeometricTrust()) * fomCof;
    }

    // ===================== Modular Filter Blocks =====================

    private boolean isBasicValid() {
        if (lastEstimate == null || lastEstimate.pose == null || lastTag == null) return false;
        if (lastEstimate.tagCount < config.minTagsSeen) return false;
        if (lastTag.ambiguity > config.maxAmbiguity) return false;
        if (!FiltersConfig.fieldRactangle.contains(lastEstimate.pose.getTranslation())) return false;
        if (Math.abs(imuYawVelocitySupplier.get()) > 150) return false;
        
        // Zero-check (prevent Limelight boot-up glitches)
        if (Math.abs(lastEstimate.pose.getX()) < 1e-3) return false;
        
        return true;
    }

    private boolean isBasicNone0() {
        if (lastEstimate == null || lastEstimate.pose == null || lastTag == null) return false;
        if (lastEstimate.tagCount < config.minTagsSeen) return false;
        if (lastTag.ambiguity > config.maxAmbiguity) return false;
        if (!FiltersConfig.fieldRactangle.contains(lastEstimate.pose.getTranslation())) return false;
        if (Math.abs(imuYawVelocitySupplier.get()) > 150) return false;
        
        // Zero-check (prevent Limelight boot-up glitches)
        if (Math.abs(lastEstimate.pose.getX()) < 1e-3) return false;
        
        return true;
    }

    /** Logic for the "Motion Cone" / Teleportation check */
    private double calculateMotionTrust() {
        Pose2d predicted = PoseEstimator.getPoseAt(lastCaptureTime);
        double dist = lastEstimate.pose.getTranslation().getDistance(predicted.getTranslation());

        ChassisSpeeds speeds = chassisSpeeds.get();
        double v = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        double dt = lastEstimate.latency / 1000.0;

        // v*dt + 0.5*a*dt^2 + margin
        double maxAllowedJump = Math.max(config.idleBubbleMeters, 
                                (v * dt) + (0.5 * config.aMax * dt * dt)) 
                                + config.motionMarginMeters;

        if (dist > maxAllowedJump) return 0.0;

        // Returns a trust value that drops as dist approaches maxAllowedJump
        return 1.0 - sq(dist / maxAllowedJump);
    }

    /** Logic for Distance and FOV penalties */
    private double calculateGeometricTrust() {
        // Distance Penalty: 1.0 up to 1m, linear drop to 0.1 at 5m
        double dist = lastEstimate.avgTagDist;
        double trustRange = 1.0;
        
        if (dist > 1.0) {
            // Linear interpolation from 1.0 at 1m to 0.1 at maxDistanceMeters
            trustRange = MathUtil.interpolate(1.0, 0.1, 
                (dist - 1.0) / (config.maxDistanceMeters - 1.0));
        }

        // Multi-tag bonus
        if (lastEstimate.tagCount > 1) trustRange *= 1.2;

        return clamp01(trustRange);
    }

    // ===================== Helpers =====================
    private static double sq(double x) { return x * x; }
    private static double clamp01(double v) { return MathUtil.clamp(v, 0, 1); }
}