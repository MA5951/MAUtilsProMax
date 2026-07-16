
package com.MAutils.Vision.IOs;

import com.MAutils.Vision.Util.VisionTarget;
import com.MAutils.Vision.Util.LimelightHelpers.IMUData;
import com.MAutils.Vision.Util.LimelightHelpers.PoseEstimate;
import com.MAutils.Vision.Util.LimelightHelpers.RawFiducial;

import edu.wpi.first.math.geometry.Transform3d;

public interface VisionCameraIO {

    public enum PoseEstimateType {
        MT2,
        MT1
    }

    String getName();

    boolean isTag();

    VisionTarget getTarget();

    void setCameraPosition(Transform3d positionRelaticToRobot);

    void setPipline(int pipeline);

    void setCrop(double cropXMin, double cropXMax, double cropYMin, double cropYMax);

    void allowTags(int[] tags);

    void takeSnapshot();

    int getPipline();

    RawFiducial getTag();

    RawFiducial[] getFiducials();

    IMUData getIMU();

    PoseEstimate getPoseEstimate(PoseEstimateType type);

    void update();

}
