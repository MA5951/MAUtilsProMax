
package com.MAutils.Vision;

import com.MAutils.Vision.IOs.Camera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSystem extends SubsystemBase {
    private static VisionSystem instance;

    private Camera[] cameras;

    private VisionSystem(Camera... cameras) {
        this.cameras = cameras;
    }

    public void setCameras(Camera... cameras) {
        this.cameras = cameras;
    }

    @Override
    public void periodic() {
        for (Camera camera : cameras) {
            camera.update();
        }
    }

    public static VisionSystem getInstance() {
        if (instance == null) {
            instance = new VisionSystem();
        }
        return instance;
    }

}
