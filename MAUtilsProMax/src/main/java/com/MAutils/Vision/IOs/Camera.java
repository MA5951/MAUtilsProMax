package com.MAutils.Vision.IOs;

import com.MAutils.Logger.MALog;
import com.MAutils.Vision.Util.LimelightHelpers.RawFiducial;

public class Camera {

    protected VisionCameraIO cameraIO;
    protected String name;

    protected RawFiducial tag;

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
        tag = cameraIO.getTag();
        MALog.log("Subsystems/Vision/Cameras/" + name +"/Target/Pipline", cameraIO.getPipline());
        MALog.log("Subsystems/Vision/Cameras/" + name +"/Target/Tx", tag.txnc);
        MALog.log("Subsystems/Vision/Cameras/" + name +"/Target/Ty", tag.tync);
        MALog.log("Subsystems/Vision/Cameras/" + name +"/Target/Ta", tag.ta);

    }

}
