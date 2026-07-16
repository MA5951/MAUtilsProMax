package com.MAutils.Vision.IOs;

import java.util.function.Supplier;

import com.MAutils.Logger.MALog;
import com.MAutils.PoseEstimation.PoseEstimationMA;
import com.MAutils.PoseEstimation.PoseEstimationMA.VisionObservation;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.PoseEstimation.PoseEstimatorSource;
import com.MAutils.Vision.Filters.AprilTagFilters;
import com.MAutils.Vision.Filters.FiltersConfig;
import com.MAutils.Vision.IOs.VisionCameraIO.PoseEstimateType;
import com.MAutils.Vision.Util.LimelightHelpers.PoseEstimate;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class AprilTagCamera extends Camera {

    private final PoseEstimatorSource poseEstimatorSource;
    public final Supplier<Double> robotAngleSupplier; // degrees
    public final Supplier<Double> robotAngleVelocitySupplier; // degrees/sec

    private FiltersConfig teleopConfig, autoConfig;
    private AprilTagFilters aprilTagFilters;
    private Transform2d delta;
    private PoseEstimate poseEstimate;
    private Pose2d visionPose, prior;
    private Rotation2d heading;
    private Twist2d visionTwsit = new Twist2d();
    private boolean updatePoseEstiamte = true;
    private double xyFom, oFom, visionTs, fieldDx, fieldDy, fieldDtheta, robotDx, robotDy, fomCOF;

    // ===== ADDED: supplier for chassis speeds so filters can use real v =====
    private final Supplier<ChassisSpeeds> chassisSpeedsSupplier; // ADDED

    // ===== CHANGED: original ctor now delegates and uses zero speeds by default
    // =====

    // TODO need to add filter of more then one camrea that check if the dis bettwen
    // the src are in tolorecn

    public AprilTagCamera(VisionCameraIO cameraIO,
            FiltersConfig teleopConfig,
            FiltersConfig autoConfig,
            Supplier<Double> robotAngleSupplier) {
        this(cameraIO, teleopConfig, autoConfig, robotAngleSupplier, () -> new ChassisSpeeds(), () -> 0.0, 1.0); // CHANGED
    }

    public AprilTagCamera(VisionCameraIO cameraIO,
            FiltersConfig teleopConfig,
            Supplier<Double> robotAngleSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            Supplier<Double> robotAngleVelocitySupplier, double fomCOF) { // ADDED
        this(cameraIO, teleopConfig, teleopConfig, robotAngleSupplier, chassisSpeedsSupplier,
                robotAngleVelocitySupplier, fomCOF); // CHANGED
    }

    // ===== ADDED: new ctor that accepts a ChassisSpeeds supplier =====
    public AprilTagCamera(VisionCameraIO cameraIO,
            FiltersConfig teleopConfig,
            FiltersConfig autoConfig,
            Supplier<Double> robotAngleSupplier,
            Supplier<ChassisSpeeds> chassisSpeedsSupplier,
            Supplier<Double> robotAngleVelocitySupplier, double fomCOF) { // ADDED
        super(cameraIO);

        this.fomCOF = fomCOF;
        this.teleopConfig = teleopConfig;
        this.autoConfig = autoConfig;

        this.robotAngleVelocitySupplier = robotAngleVelocitySupplier;
        this.robotAngleSupplier = robotAngleSupplier;
        this.chassisSpeedsSupplier = chassisSpeedsSupplier != null
                ? chassisSpeedsSupplier
                : () -> new ChassisSpeeds(); // ADDED

        // ===== CHANGED: pass IMU yaw (radians) + chassis speeds to filters
        this.aprilTagFilters = new AprilTagFilters(getFiltersConfig(),
                cameraIO,
                this.chassisSpeedsSupplier,
                () -> Math.toRadians(this.robotAngleSupplier.get()),
                robotAngleVelocitySupplier, fomCOF); // ADDED

        poseEstimatorSource = new PoseEstimatorSource(cameraIO.getName(),
                () -> getRobotRelaticTwist(poseEstimate, visionTs),
                () -> xyFom,
                () -> oFom,
                () -> visionTs);

        PoseEstimator.addSource(poseEstimatorSource);
    }

    public void setUpdatePoseEstimate(boolean updatePoseEstiamte) {
        this.updatePoseEstiamte = updatePoseEstiamte;
    }

    public FiltersConfig getFiltersConfig() {
        return DriverStation.isTeleop() ? teleopConfig : autoConfig;
    }

    // TODO need to also consider isValidForHeadingReset
    @Override
    public void update() {
        aprilTagFilters.update();
        cameraIO.update();
        poseEstimate = cameraIO.getPoseEstimate(getFiltersConfig().poseEstimateType);
        logIO();

        if (updatePoseEstiamte) {
            xyFom = aprilTagFilters.getXyFOM(); // CHANGED: now computed by yaw/motion gates
            oFom = aprilTagFilters.getOFOM(); // CHANGED

            MALog.log("Subsystems/Vision/Cameras/" + name + "/XY FOM", xyFom);
            MALog.log("Subsystems/Vision/Cameras/" + name + "/Omega FOM", oFom);

            visionTs = getVisionTimetemp();

            if (cameraIO.isTag() && (cameraIO.getPoseEstimate(FiltersConfig.poseEstimateType).pose.getX() > 0.01)
                    && (cameraIO.getPoseEstimate(FiltersConfig.poseEstimateType).pose.getY() > 0.01)) {
                getRobotRelaticTwist(poseEstimate, visionTs);
                poseEstimatorSource.capture();
                PoseEstimationMA.getInstance().addVisionObservation(
                        new VisionObservation(Timer.getFPGATimestamp() - (poseEstimate.latency / 1000.0),
                                new Pose3d(poseEstimate.pose), VecBuilder.fill(0.07, 0.07, 10)),
                        cameraIO.getName());
                MALog.log("Subsystems/Vision/Cameras/" + name + "/Odometry", "Odometry captured");
            } else {
                oFom = 0;
                xyFom = 0;
                poseEstimatorSource.capture();
                MALog.log("Subsystems/Vision/Cameras/" + name + "/Odometry", "Odometry didn't captured");

            }

        }

    }

    @Override
    protected void logIO() {
        super.logIO();

        MALog.log("Subsystems/Vision/Cameras/" + name + "/Target/Ambiguit", tag.ambiguity);
        MALog.log("Subsystems/Vision/Cameras/" + name + "/Target/Id", tag.id);

        MALog.log("Subsystems/Vision/Cameras/" + name + "/Pose Estimate/Pose", poseEstimate.pose);
        MALog.log("Subsystems/Vision/Cameras/" + name + "/Pose Estimate/Avg Distance", poseEstimate.avgTagDist);
        MALog.log("Subsystems/Vision/Cameras/" + name + "/Pose Estimate/Tag Count", poseEstimate.tagCount);
        MALog.log("Subsystems/Vision/Cameras/" + name + "/Pose Estimate/Latency", poseEstimate.latency);
        MALog.log("Subsystems/Vision/Cameras/" + name + "/Pose Estimate/Timestamp", poseEstimate.timestampSeconds);
        MALog.log("Subsystems/Vision/Cameras/" + name + "/Heading Reset", isValidForHeadingReset());

    }

    private Double getVisionTimetemp() {
        return Timer.getFPGATimestamp() - (poseEstimate.latency / 1000.0);
    }

    // TODO add get raw pose value,without fom or filters just pose3d

    private Twist2d getRobotRelaticTwist(PoseEstimate poseEstimator, double timestemp) {
        visionPose = poseEstimate.pose;
        prior = PoseEstimator.getPoseAt(timestemp);
        MALog.log("/OdometryDebug/PriorPose", prior);

        delta = new Transform2d(prior, visionPose);
        // MALog.log("/OdometryDebug/Delta", new Pose2d(new Translation2d(de),new
        // Rotation2d(0)));

        fieldDx = delta.getTranslation().getX();
        fieldDy = delta.getTranslation().getY();
        fieldDtheta = delta.getRotation().getRadians();

        visionTwsit.dx = fieldDx;
        visionTwsit.dy = fieldDy;
        visionTwsit.dtheta = fieldDtheta;

        return visionTwsit;

    }

    public boolean isValidForHeadingReset() {
        return cameraIO.isTag() && cameraIO.getFiducials().length > 1 && cameraIO.getTag().distToCamera < 2
                && cameraIO.getPoseEstimate(VisionCameraIO.PoseEstimateType.MT1).pose.getX() != -1;
    }
}
