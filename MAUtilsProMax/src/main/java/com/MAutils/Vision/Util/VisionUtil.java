
package com.MAutils.Vision.Util;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionUtil {

    //TODO this passed a axis tasting? , need to check that robotToCamera.getRotation().getY() + tagPitch < 80 deg, otherwise the func will retunr nan/inf
    public static Double getDistance(Transform3d robotToCamera, double tagHeight, double tagPitch) {
        return (tagHeight - robotToCamera.getZ()) / Math.tan(Math.toRadians(robotToCamera.getRotation().getY() + tagPitch));
    }

    //TODO add one for TX 
    

}
