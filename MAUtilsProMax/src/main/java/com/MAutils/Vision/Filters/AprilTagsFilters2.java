
package com.MAutils.Vision.Filters;

import java.util.function.Supplier;

import com.MAutils.Utils.Circle2d;
import com.MAutils.Vision.IOs.VisionCameraIO;
import com.MAutils.Vision.Util.LimelightHelpers.PoseEstimate;
import com.MAutils.Vision.Util.LimelightHelpers.RawFiducial;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class AprilTagsFilters2 {

    private FiltersConfig config;
    private final VisionCameraIO visionCameraIO;
    private final Supplier<ChassisSpeeds> chassisSpeeds;
    private final Supplier<Double> imuYawRadSupplier;
    private final Supplier<Double> imuYawVelocitySupplier;

    private PoseEstimate lastEstimate;
    private Translation2d lastPose = new Translation2d();
    private RawFiducial lastTag;
    private double lastCaptureTime;
    private double lastMeasurementTime;
    private double lastLinearVelocity;

    private double visionYaw = 0;
    private double yawDiffDeg = 0;
    private double acceleration = 0;
    private double linearVelocity = 0;
    private double dt = 0;
    private double radius = 0 ;

    private  boolean isInside = false;

    private Circle2d c;

    public AprilTagsFilters2(FiltersConfig config, 
                           VisionCameraIO visionCameraIO, 
                           Supplier<ChassisSpeeds> chassisSpeedsSupplier, 
                           Supplier<Double> imuYawRadSupplier,
                           Supplier<Double> imuYawVelocitySupplier) {

        this.config = config;
        this.visionCameraIO = visionCameraIO;
        this.chassisSpeeds = chassisSpeedsSupplier;
        this.imuYawRadSupplier = imuYawRadSupplier;
        this.imuYawVelocitySupplier = imuYawVelocitySupplier;
        lastMeasurementTime = Timer.getFPGATimestamp();
        lastLinearVelocity = 0.0;
        lastPose = null;
    }

    public void update() {
        this.lastEstimate = visionCameraIO.getPoseEstimate(config.poseEstimateType);
        this.lastTag = visionCameraIO.getTag();
        if (lastEstimate != null) {
            this.lastCaptureTime = Timer.getFPGATimestamp() - (lastEstimate.latency / 1000.0);
        }

    }

    private double currentAcceleration() {
        dt = Timer.getFPGATimestamp() - lastMeasurementTime;
        lastMeasurementTime = Timer.getFPGATimestamp();

        if (dt <= 0) {
            return 0.0;
        }

        linearVelocity = Math.hypot(chassisSpeeds.get().vxMetersPerSecond, chassisSpeeds.get().vyMetersPerSecond);

        acceleration = (linearVelocity - lastLinearVelocity) / dt;
        lastLinearVelocity = linearVelocity;

        return acceleration;
    }

    public boolean isPossiblePose() {
        if (lastEstimate == null || lastEstimate.pose == null) return false;

        if (lastPose == null) {
            lastPose = lastEstimate.pose.getTranslation();
            return true; 
        }

        acceleration = currentAcceleration();

        radius = (lastLinearVelocity * dt) + (0.5 * acceleration * dt * dt) + config.motionMarginMeters;

        
        c = new Circle2d(lastPose, radius);

        isInside = c.contains(lastEstimate.pose.getTranslation());

        if (isInside) {
            lastPose = lastEstimate.pose.getTranslation(); 
        }
        return isInside;
    }

    public boolean isLinearVelocityTooHigh() {
        return Math.hypot(chassisSpeeds.get().vxMetersPerSecond, chassisSpeeds.get().vyMetersPerSecond) > config.maxLinearVelocityMS;
    }

    public boolean isAngularVelocityTooHigh() {
        return Math.abs(imuYawVelocitySupplier.get()) > config.maxAngularVelocityRS;
    }

    public boolean isOutOfField() {
        if (lastEstimate == null)  return true;
        return !FiltersConfig.fieldRactangle.contains(lastEstimate.pose.getTranslation());
    }

    public boolean isTagAmbiguousTooBig() {
        if (lastTag == null) return true;
        return lastTag.ambiguity > config.maxAmbiguity;
    }

    public boolean isHardYawDrift() {
        if (lastEstimate == null) return false;
        visionYaw = lastEstimate.pose.getRotation().getRadians(); //TODO check if imuYawRadSupplier is in radians or degrees
        yawDiffDeg = Math.toDegrees(Math.abs(MathUtil.angleModulus(visionYaw - imuYawRadSupplier.get())));
        return (yawDiffDeg >= config.hardYawGateDeg);
    }

    public boolean hasEnoughTags() {
    if (lastEstimate == null) return false;
    return !(lastEstimate.tagCount < config.minTagsSeen);
    }

    public boolean isBasicValid() {
        return !(lastEstimate == null || lastEstimate.pose == null || lastTag == null);
    }

    public boolean isTagSizeTooSmall() {
        if (lastTag == null) return true;
        return lastTag.ta < config.smallestTagSize;
    }

    public boolean isValid() {
        if (!isBasicValid()) return false;
        if (!hasEnoughTags()) return false;
        if (isOutOfField()) return false;
        if (isTagAmbiguousTooBig()) return false;
        if (isHardYawDrift()) return false;
        if (isLinearVelocityTooHigh() || isAngularVelocityTooHigh()) return false;
        if (isTagSizeTooSmall()) return false;
        if (!isPossiblePose()) return false;

        if (Math.abs(lastEstimate.pose.getX()) < 1e-3) return false;
        if (Math.abs(lastEstimate.pose.getY()) < 1e-3) return false;

        return true;
    }
}