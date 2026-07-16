
package com.MAutils.RobotControl;

import java.util.function.Supplier;

import com.MAutils.PoseEstimation.PoseEstimationMA;
import com.MAutils.PoseEstimation.PoseEstimator;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rectangle2d;


/*
 * Default superstructure class that provides common functionality for robot superstructures.
 */
public abstract class DeafultSuperStructure {

    public static final double MOVING_MPS = 0.03;


    protected Supplier<Pose2d> robotPoseSupplier = () -> PoseEstimator.getCurrentPose();
    protected Supplier<Double> robotVelocitySupplier;

    protected DeafultSuperStructure(Supplier<Double> robotVelocitySupplier) {
        this.robotVelocitySupplier = robotVelocitySupplier;
    }

    public boolean isRobotIn(Rectangle2d area) {
        return area.contains(robotPoseSupplier.get().getTranslation());
    }

    public boolean isAtPose(Pose2d target, double toleranceMeters) {
        return robotPoseSupplier.get().getTranslation().getDistance(target.getTranslation()) < toleranceMeters;
    }

    public boolean isMoving() {
        return robotVelocitySupplier.get() > MOVING_MPS; 
    }

    

    
}
