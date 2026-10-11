package com.MAutils.Vision.IOs;

import com.MAutils.Logger.MALog;
import com.MAutils.Vision.Util.LimelightHelpers;
import com.MAutils.Vision.Util.LimelightHelpers.RawFiducial;

public class Camera {

    protected VisionCameraIO cameraIO;
    protected String name;

    protected RawFiducial tag = new RawFiducial(0, 0, 0, 0, 0, 0, 0);

    public Camera(VisionCameraIO cameraIO) {
        this.cameraIO = cameraIO;
        this.name = cameraIO.getName();
    }

    public VisionCameraIO getCameraIO() {
        return cameraIO;
    }

    public void update() {
        cameraIO.update();
        logIO();
    }

    protected void logIO() {

        MALog.log("Subsystems/Vision/Cameras/" + name +"/Target/Pipline", cameraIO.getPipline());
        LimelightHelpers.RawFiducial[] fiducials = LimelightHelpers.getRawFiducials(name);
        
        if(LimelightHelpers.getTV(name) && fiducials != null && fiducials.length >= 1) {
            tag = fiducials[0];

            MALog.log("Subsystems/Vision/Cameras/" + name +"/Target/Tx", LimelightHelpers.getTX(name));
            MALog.log("Subsystems/Vision/Cameras/" + name +"/Target/Ty", LimelightHelpers.getTY(name));
            MALog.log("Subsystems/Vision/Cameras/" + name +"/Target/Ta", LimelightHelpers.getTA(name));
        } else {
            MALog.log("Subsystems/Vision/Cameras/" + name +"/Target/Tx", 0);
            MALog.log("Subsystems/Vision/Cameras/" + name +"/Target/Ty", 0);
            MALog.log("Subsystems/Vision/Cameras/" + name +"/Target/Ta", 0);
            tag = new RawFiducial(0, 0, 0, 0, 0, 0, 0);

        }
        

    }

}
