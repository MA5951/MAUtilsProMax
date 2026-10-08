
package com.MAutils.Vision.Filters;

import java.util.function.Supplier;

import com.MAutils.Logger.MALog;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.Utils.Constants;
import com.MAutils.Vision.IOs.VisionCameraIO;
import com.MAutils.Vision.Util.LimelightHelpers.PoseEstimate;
import com.MAutils.Vision.Util.LimelightHelpers.RawFiducial;

import edu.wpi.first.math.geometry.Ellipse2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class AprilTagsFilters {

    private FiltersConfig config;
    private final VisionCameraIO visionCameraIO;
    private final Supplier<ChassisSpeeds> chassisSpeeds;
    private final Supplier<Double> imuYawVelocitySupplier; 


    private Translation2d currentPose = new Translation2d();
    private Translation2d lastPose = new Translation2d();
    private RawFiducial currentTag;
    private double currentMeasurementTime;
    private double amountOftagsSeen = 0;

    private double linearVelocity = 0;
    private double dt = Constants.LOOP_TIME;
    private double radius = 0 ;

    private  boolean isInside = false;

    private Ellipse2d c;

    public AprilTagsFilters(FiltersConfig config, 
                           VisionCameraIO visionCameraIO, 
                           Supplier<ChassisSpeeds> chassisSpeedsSupplier, 
                           Supplier<Double> imuYawVelocitySupplier) {

        this.config = config;
        this.visionCameraIO = visionCameraIO;
        this.chassisSpeeds = chassisSpeedsSupplier;
        this.imuYawVelocitySupplier = imuYawVelocitySupplier;
        currentMeasurementTime = Timer.getFPGATimestamp();
        currentPose = null;
    }

    public void update() {
        lastPose = currentPose;
        this.currentPose = visionCameraIO.getPoseEstimate(config.poseEstimateType).pose.getTranslation();
        this.currentTag = visionCameraIO.getTag();
        amountOftagsSeen = visionCameraIO.getPoseEstimate(config.poseEstimateType).tagCount;

        MALog.log("PE/ Limelight/isImpossiblePose", isImpossiblePose());
        MALog.log("PE/ Limelight/isLinearVelocityOutTolerance", isLinearVelocityOutTolerance());
        MALog.log("PE/ Limelight/isAngularVelocityOutTolerance", isAngularVelocityOutTolerance());
        MALog.log("PE/ Limelight/isOutOfField", isOutOfField());
        MALog.log("PE/ Limelight/isTagAmbiguousTooBig", isTagAmbiguousTooBig());
        MALog.log("PE/ Limelight/isTagSizeTooSmall", isTagSizeTooSmall());

        
        MALog.log("PE/ Limelight/pose",currentPose);

    }

    public boolean isImpossiblePose() {

        if (currentPose == null || lastPose == null) {
            return true; 
        }

        if(Math.abs(lastPose.getX()) < 1e-3 || Math.abs(lastPose.getY()) < 1e-3) {
            return true;
        }
        linearVelocity = Math.hypot(chassisSpeeds.get().vxMetersPerSecond, chassisSpeeds.get().vyMetersPerSecond);

        radius = (linearVelocity * dt) + config.motionMarginMeters;

        
        c = new Ellipse2d(PoseEstimator.getCurrentPose().getTranslation(), radius);//cAHNGE CURRENT POSE TO POSE ESTIMATE

        isInside = c.contains(currentPose);


        currentMeasurementTime = Timer.getFPGATimestamp();

        return !isInside;
    }

    public boolean isLinearVelocityOutTolerance() {
        return Math.hypot(chassisSpeeds.get().vxMetersPerSecond, chassisSpeeds.get().vyMetersPerSecond) > config.maxLinearVelocityMS;
    }

    public boolean isAngularVelocityOutTolerance() {
        return Math.abs(imuYawVelocitySupplier.get()) > config.maxAngularVelocityRS;
    }

    public boolean isOutOfField() {
        if (currentPose == null)  return true;
        return !config.fieldRactangle.contains(currentPose);
    }

    public boolean isTagAmbiguousTooBig() {
        if (currentTag == null) return true;
        return currentTag.ambiguity > config.maxAmbiguity;
    }


    public boolean isTagSizeTooSmall() {
        if (currentTag == null) return true;
        return currentTag.ta < config.smallestTagSize;
    }

    public double getXYFOM() {
        return 1;
    }

    public double getOFOM() {
        return 1;
    }

    public boolean isValid() {
        if (currentPose == null || currentTag == null || amountOftagsSeen < config.minTagsSeen) return false;   

        if (isOutOfField()) return false;
        if (isTagAmbiguousTooBig()) return false;
        if (isAngularVelocityOutTolerance() || isLinearVelocityOutTolerance()) return false;
        if (isTagSizeTooSmall()) return false; 
        if (isImpossiblePose()) return false;

        if (Math.abs(currentPose.getX()) < 1e-3) return false;
        if (Math.abs(currentPose.getY()) < 1e-3) return false;

        return true;
    }
}