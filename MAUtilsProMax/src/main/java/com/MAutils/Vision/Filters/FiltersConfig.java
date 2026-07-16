package com.MAutils.Vision.Filters;

import com.MAutils.Vision.IOs.VisionCameraIO.PoseEstimateType;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;

public class FiltersConfig {

    public double maxAmbiguity = 0.4; // Reject if ambiguity > this
    public double maxDistanceMeters = 6.0; // Reject if pose is too far

    public double maxPoseJumpMeters = 0.7; // Reject if new pose is too far from current

    public double maxDeltaAngleDegrees = 25.0; // Reject if rotation jump is too big
    public double maxLatencyMillis = 100; // Reject if data is too old //TODO need to be match to the history size in the poseestimation, but its a good filter to add
    public double minTagsSeen = 1; // Reject if fewer than this many tags seen

    public double fieldOfViewLimitXDeg = 25.0; // Reject if tx near horizontal edge //TODO also a good filter to add 
    public double fieldOfViewLimitYDeg = 20.0; // Reject if ty near vertical edge

    public boolean requireTagWhitelist = false; // Enable tag whitelist //TODO onther filter that need to be imploment

    public static final Translation2d FieldZeroCorner = new Translation2d(0, 0);
    public static final Translation2d FieldFarCorner = new Translation2d(17.548, 8.052);    
    public static final Translation2d FieldMiddlePoint = new Translation2d(17.548 / 2, 8.052 / 2);
    public static final Rectangle2d fieldRactangle = new Rectangle2d(FieldZeroCorner, FieldFarCorner);

    //TODO need to add hear a obstical list that the filter will go over it, you can jump into a abostical + move to constants the sizees

    public static PoseEstimateType poseEstimateType = PoseEstimateType.MT2; // Use MegaTag2 for pose estimates

    public FiltersConfig() {}

    // ===== ADDED: Simple trust scorer tuning =====
    /** Degrees: >= hardYawGateDeg → reject yaw trust (and typically the whole frame). */
    public double hardYawGateDeg = 30.0;              // ADDED
    /** Degrees: <= softYawFullTrustDeg → yaw trust ≈ 1, then smooth drop to 0 at hard gate. */
    public double softYawFullTrustDeg = 5.0;          // ADDED

    /** Motion-cone model: v*dt + 0.5*aMax*dt^2 + margin (meters). */
    public double aMax = 6.0;                         // ADDED
    public double motionMarginMeters = 1.5;          // ADDED
    public double idleBubbleMeters = 0.07;            // ADDED

    /** Minimum trust to consider a measurement useful. */
    public double minAcceptTrust = 0.05;              // ADDED
}
