
package com.MAutils.Simulation.Simulatables;

import java.util.function.Supplier;

import com.MAutils.Simulation.Utils.Simulatable;
import com.MAutils.Vision.IOs.CameraSimIO;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionWorldSimulation implements Simulatable {

    private Supplier<Pose2d> robotPose;

    public VisionWorldSimulation(Supplier<Pose2d> robotPose) {
        this.robotPose = robotPose; //TODO its better to pass the swerves sim pose stirght 
    }

    @Override
    public void updateSimulation() {
        CameraSimIO.visionSystemSim.update(robotPose.get());
    }


}
