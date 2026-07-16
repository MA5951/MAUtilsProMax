package com.MAutils.Vision.IOs;

import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.MAutils.Components.CameraTypes.Cameras;
import com.MAutils.Simulation.Simulatables.VisionWorldSimulation;
import com.MAutils.Simulation.SimulationManager;
import com.MAutils.Vision.Util.LimelightHelpers.IMUData;
import com.MAutils.Vision.Util.LimelightHelpers.PoseEstimate;
import com.MAutils.Vision.Util.LimelightHelpers.RawFiducial;
import com.MAutils.Vision.Util.VisionTarget;
import com.MAutils.Vision.Util.VisionUtil;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;

public class CameraSimIO implements VisionCameraIO {

    // --- Constants / tuning ---
    private static final double STALE_SEC = 0.20; // consider frame stale after 200 ms

    // --- Blank fallbacks ---
    private final RawFiducial[] blankFiducial =
        new RawFiducial[]{ new RawFiducial(0, 0, 0, 0, 0, 0, 0) };

    private final PoseEstimate blankPoseEstimate =
        new PoseEstimate(new Pose2d(-1, -1, new Rotation2d()), 0, 0, 0, 0, 0, 0,
                         blankFiducial, false);

    // --- State ---
    private RawFiducial[] fiducials;
    private PoseEstimate poseEstimate;
    private PoseEstimate lastReturn = blankPoseEstimate ;

    private final Transform3d robotToCamera;
    private AprilTagFieldLayout tagLayout;
    private final PhotonPoseEstimator poseEstimator;
    private EstimatedRobotPose photonPoseEstimate;

    public final Supplier<Pose2d> robotPose;

    public static VisionSystemSim visionSystemSim;
    private static boolean isRegisterd = false;
    private static boolean isVisitionRegisters= false;

    private final PhotonCamera camera;
    private final PhotonCameraSim cameraSim;

    // Cached most-recent result (do NOT read queue elsewhere)
    private PhotonPipelineResult lastResult;
    private double lastResultTs = -1.0; // seconds (FPGA clock), for staleness

    public CameraSimIO(String name,
                       Cameras cameraProps,
                       Transform3d robotToCamera,
                       Supplier<Pose2d> robotPose) {
        this.robotToCamera = robotToCamera;
        this.robotPose = robotPose;

        try {
            tagLayout = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile
            );
        } catch (Exception e) {
            // Fail fast so you actually see the root cause
            throw new RuntimeException("Failed to load AprilTag field layout", e);
        }

        if (!isVisitionRegisters) {
            visionSystemSim = new VisionSystemSim("Main Vision Sim");

            try {
                tagLayout = AprilTagFieldLayout.loadFromResource(
                    AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile
                );
                visionSystemSim.addAprilTags(tagLayout);
            } catch (Exception e) {
                // Fail fast so you actually see the root cause
                throw new RuntimeException("Failed to load AprilTag field layout", e);
            }

            isVisitionRegisters = true;
        }


        camera = new PhotonCamera(name);
        cameraSim = new PhotonCameraSim(camera, cameraProps.getSimulationProp());

        poseEstimator = new PhotonPoseEstimator(
            tagLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            robotToCamera
        );

        visionSystemSim.addCamera(cameraSim, robotToCamera);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        if (!isRegisterd) {
            SimulationManager.registerSimulatable(new VisionWorldSimulation(robotPose));
            isRegisterd = true;
        }
    }

    public void setCameraPosition(Transform3d positionRelaticToRobot) {
        visionSystemSim.adjustCamera(cameraSim, positionRelaticToRobot);
    }

    public void setPipline(int pipeline) {}
    public void setCrop(double cropXMin, double cropXMax, double cropYMin, double cropYMax) {}
    public void allowTags(int[] tags) {}
    public void takeSnapshot() {}
    public int getPipline() { return 0; }

    @Override
    public String getName() {
        return camera.getName();
    }

    /** Call once per robot loop. */
    public void update() {
        // 1) Step the vision sim with the current robot pose
        visionSystemSim.update(robotPose.get());

        // 2) Consume unread queue EXACTLY ONCE per loop
        var unread = camera.getAllUnreadResults(); // this clears the FIFO
        if (unread != null && !unread.isEmpty()) {
            lastResult = unread.get(unread.size() - 1); // newest
            // prefer the result's own timestamp; fall back to FPGA time if not exposed
            try {
                lastResultTs = lastResult.getTimestampSeconds();
            } catch (Throwable t) {
                lastResultTs = Timer.getFPGATimestamp();
            }
        } // else keep prior lastResult (don’t null it)

        // 3) Optional debug
        double age = (lastResultTs > 0) ? (Timer.getFPGATimestamp() - lastResultTs) : Double.POSITIVE_INFINITY;
    }

    /** Whether we currently have a (possibly cached) frame with targets and it's not stale. */
    public boolean isTag() {
        if (lastResult == null || !lastResult.hasTargets()) return false;
        double age = (lastResultTs > 0) ? (Timer.getFPGATimestamp() - lastResultTs) : Double.POSITIVE_INFINITY;
        return age <= STALE_SEC;
    }

    public RawFiducial getTag() {
        var fids = getFiducials();
        return fids.length > 0 ? fids[0] : blankFiducial[0];
    }

    public RawFiducial[] getFiducials() {
        if (lastResult == null || !lastResult.hasTargets()) return blankFiducial;

        var targets = lastResult.getTargets();
        if (targets == null || targets.isEmpty()) return blankFiducial;

        fiducials = new RawFiducial[targets.size()];
        for (int i = 0; i < targets.size(); i++) {
            PhotonTrackedTarget t = targets.get(i);

            double distCam = 0.0, distRobot = 0.0;
            var tagPoseOpt = tagLayout.getTagPose(t.getFiducialId());
            if (tagPoseOpt.isPresent()) {
                double tagZ = tagPoseOpt.get().getZ();
                distCam = VisionUtil.getDistance(robotToCamera, tagZ, t.getPitch());
                distRobot = distCam; 
            }

            // Construct the element (avoid NPEs)
            fiducials[i] = new RawFiducial(
                t.getFiducialId(),
                t.getYaw(),
                t.getPitch(),
                t.getArea(),
                t.getPoseAmbiguity(),
                distCam,
                distRobot
            );
        }
        return fiducials.length > 0 ? fiducials : blankFiducial;
    }

    public PoseEstimate getPoseEstimate(PoseEstimateType type) {
        // Use cached frame; don’t touch the FIFO here
        if (lastResult == null) return blankPoseEstimate;

        if (Math.abs(lastReturn.timestampSeconds - lastResult.getTimestampSeconds()) < 1e-6) {
            return lastReturn; // already computed for this frame
        }
        

        var opt = poseEstimator.update(lastResult); // Optional<EstimatedRobotPose>
        //if (opt.isEmpty()) return blankPoseEstimate;

        

        if (opt.isEmpty()) {
            return blankPoseEstimate;
        }

        photonPoseEstimate = opt.get();

        if (poseEstimate == null) {
            poseEstimate = new PoseEstimate(new Pose2d(), 0, 0, 0, 0, 0, 0, blankFiducial, false);
        }

        
        poseEstimate.pose = photonPoseEstimate.estimatedPose.toPose2d();
        poseEstimate.timestampSeconds = photonPoseEstimate.timestampSeconds;
        poseEstimate.isMegaTag2 = type == PoseEstimateType.MT2;
        poseEstimate.rawFiducials = getFiducials();
        poseEstimate.tagCount = poseEstimate.rawFiducials.length;
        lastReturn = poseEstimate;

        return poseEstimate;
    }

    @Override
    public VisionTarget getTarget() {
        throw new UnsupportedOperationException("Unimplemented method 'getTarget'");
    }

    @Override
    public IMUData getIMU() {
        return new IMUData();
    }
}
