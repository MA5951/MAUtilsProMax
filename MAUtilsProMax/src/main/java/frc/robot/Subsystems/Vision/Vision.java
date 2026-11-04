package frc.robot.Subsystems.Vision;

import com.MAutils.Logger.MALog;
import com.MAutils.Vision.VisionSystem;
import com.MAutils.Vision.Util.LimelightHelpers;

import edu.wpi.first.units.measure.Distance;
import frc.robot.Util.Field;

public class Vision {
    double rawDistance;
    private static Vision vision;
    private int[] tag = Field.MAIN_TAGS;
     
    private Vision() {
        VisionSystem.getInstance().setCameras(VisionConstants.LL);

        VisionConstants.LL.getCameraIO().allowTags(new int[]{26});
    }

    public int getTagID() {
        return VisionConstants.LL.getCameraIO().getTag().id;
    }

    public double getRawTrigDistance() {
        MALog.log("Subsystems/Vision/Pitch", -VisionConstants.LL.getCameraIO().getIMU().Pitch);
        
        
        rawDistance = (Field.HUB_TAG_HIGHT - VisionConstants.FRONT_LL_HIGHT)
                / Math.tan(Math.toRadians((-VisionConstants.LL.getCameraIO().getIMU().Pitch)
                        + LimelightHelpers.getTY(VisionConstants.LL.getCameraIO().getName())));

        if(getTagID() == 0) rawDistance =0;
        MALog.log("Subsystems/Vision/Distance", rawDistance);

        return (Field.HUB_TAG_HIGHT - VisionConstants.FRONT_LL_HIGHT)
                / Math.tan(Math.toRadians((-VisionConstants.LL.getCameraIO().getIMU().Pitch)
                        + LimelightHelpers.getTY(VisionConstants.LL.getCameraIO().getName())));

    }

    public static Vision getInstance() {
        if(vision == null) {
            vision = new Vision();
        }
        return vision;
    }
} 