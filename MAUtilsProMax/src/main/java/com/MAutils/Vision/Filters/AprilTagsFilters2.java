
package com.MAutils.Vision.Filters;

import java.util.function.Supplier;

import com.MAutils.Vision.IOs.VisionCameraIO;
import com.MAutils.Vision.Util.LimelightHelpers.PoseEstimate;
import com.MAutils.Vision.Util.LimelightHelpers.RawFiducial;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class AprilTagsFilters2 {

    private FiltersConfig config;
    private final VisionCameraIO visionCameraIO;
    private final Supplier<ChassisSpeeds> chassisSpeeds;
    private final Supplier<Double> imuYawRadSupplier;
    private final Supplier<Double> imuYawVelocitySupplier;

    private PoseEstimate lastEstimate;
    private Pose2d lastPose = new Pose2d();
    private RawFiducial lastTag;
    private double lastCaptureTime;

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
    }

    public boolean isLinearVelocity() {
        return chassisSpeeds.get().vxMetersPerSecond > config.maxXvelocityMS ||
         chassisSpeeds.get().vyMetersPerSecond > config.maxYvelocityMS;
    }

    public boolean angularVelocity() {
        return chassisSpeeds.get().omegaRadiansPerSecond > config.maxOvelocityRS;
    }
}


