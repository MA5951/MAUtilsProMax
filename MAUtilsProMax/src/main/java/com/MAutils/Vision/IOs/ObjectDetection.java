package com.MAutils.Vision.IOs;

import java.util.function.Supplier;

import com.MAutils.Logger.MALog;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.PoseEstimation.PoseEstimatorSource;
import com.MAutils.Vision.Filters.AprilTagFilters;
import com.MAutils.Vision.Filters.FiltersConfig;
import com.MAutils.Vision.IOs.VisionCameraIO.PoseEstimateType;
import com.MAutils.Vision.Util.LimelightHelpers;
import com.MAutils.Vision.Util.VisionTarget;
import com.MAutils.Vision.Util.LimelightHelpers.PoseEstimate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class ObjectDetection extends Camera {

//TODO what the point of this class? 
    public ObjectDetection(VisionCameraIO cameraIO) { 
        super(cameraIO);

    }

    

    

    @Override
    public void update() {
        cameraIO.update();
        logIO();


        
    }

    @Override
    protected void logIO() {
        super.logIO();
        

        MALog.log("Subsystems/Vision/Cameras/" + name + "/Target/Tx", cameraIO.getTarget().tx);
        MALog.log("Subsystems/Vision/Cameras/" + name + "/Target/Ty", cameraIO.getTarget().ty);
        MALog.log("Subsystems/Vision/Cameras/" + name + "/Target/Ta", cameraIO.getTarget().ta);


    }

}
