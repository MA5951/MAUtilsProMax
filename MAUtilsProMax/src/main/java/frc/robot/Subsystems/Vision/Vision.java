
package frc.robot.Subsystems.Vision;

import com.MAutils.Vision.VisionSystem;

import frc.robot.Util.Field;

public class Vision {
    private static Vision vision;
     
    private Vision() {
        VisionSystem.getInstance().setCameras(VisionConstants.LL);

        VisionConstants.LL.getCameraIO().allowTags(Field.ALL_TAGS);
    }

    public int getTagID() {
        return VisionConstants.LL.getCameraIO().getTag().id;
    }

    public static Vision getInstance() {
        if(vision == null) {
            vision = new Vision();
        }
        return vision;
    }
} 
