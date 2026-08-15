
package com.MAutils.Vision.Filters;

import java.util.function.Supplier;

import org.dyn4j.geometry.Ellipse;

import com.MAutils.Vision.IOs.VisionCameraIO;
import com.MAutils.Vision.Util.LimelightHelpers.PoseEstimate;
import com.MAutils.Vision.Util.LimelightHelpers.RawFiducial;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Ellipse2d;
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


    private PoseEstimate currentEstimate;
    private Translation2d currentPose = new Translation2d();
    private RawFiducial currentTag;
    private double currentMeasurementTime;

    private double linearVelocity = 0;
    private double dt = 0;
    private double radius = 0 ;

    private  boolean isInside = false;

    private Ellipse2d c;

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
        currentMeasurementTime = Timer.getFPGATimestamp();
        currentPose = null;
    }

    public void update() {
        this.currentEstimate = visionCameraIO.getPoseEstimate(config.poseEstimateType);
        this.currentTag = visionCameraIO.getTag();
    }

    public boolean isPossiblePose() {
        if (currentEstimate == null || currentEstimate.pose == null) return false;

        if (currentPose == null) {
            currentPose = currentEstimate.pose.getTranslation();
            return true; 
        }

        linearVelocity = Math.hypot(chassisSpeeds.get().vxMetersPerSecond, chassisSpeeds.get().vyMetersPerSecond);
        dt = Timer.getFPGATimestamp() - currentMeasurementTime;

        radius = (linearVelocity * dt) + config.motionMarginMeters;//TODO: CHANGE TO 30 CM

        
        c = new Ellipse2d(currentPose, radius);

        isInside = c.contains(currentEstimate.pose.getTranslation());


        currentPose = currentEstimate.pose.getTranslation(); 

        return isInside;
    }

    public boolean isLinearVelocityTooHigh() {
        return Math.hypot(chassisSpeeds.get().vxMetersPerSecond, chassisSpeeds.get().vyMetersPerSecond) > config.maxLinearVelocityMS;
    }

    public boolean isAngularVelocityTooHigh() {
        return Math.abs(imuYawVelocitySupplier.get()) > config.maxAngularVelocityRS;
    }

    public boolean isOutOfField() {
        if (currentEstimate == null)  return true;
        return !FiltersConfig.fieldRactangle.contains(currentEstimate.pose.getTranslation());
    }

    public boolean isTagAmbiguousTooBig() {
        if (currentTag == null) return true;
        return currentTag.ambiguity > config.maxAmbiguity;
    }

    public boolean hasEnoughTags() {
    if (currentEstimate == null) return false;
        return !(currentEstimate.tagCount < config.minTagsSeen);
    }

    public boolean isTagSizeTooSmall() {
        if (currentTag == null) return true;
        return currentTag.ta < config.smallestTagSize;
    }

    public boolean isValid() {
        if (!hasEnoughTags()) return false;
        if (isOutOfField()) return false;
        if (isTagAmbiguousTooBig()) return false;
        if (isLinearVelocityTooHigh() || isAngularVelocityTooHigh()) return false;
        if (isTagSizeTooSmall()) return false;
        if (!isPossiblePose()) return false;

        if (Math.abs(currentEstimate.pose.getX()) < 1e-3) return false;
        if (Math.abs(currentEstimate.pose.getY()) < 1e-3) return false;

        return true;
    }
}