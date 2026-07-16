
package com.MAutils.Swerve.Utils;

import java.util.function.Supplier;

import com.MAutils.Logger.MALog;
import com.MAutils.Swerve.IOs.Gyro.GyroIO.GyroData;

import edu.wpi.first.math.geometry.Translation2d;

public class CollisionDetector {
    private final double G_THRESHLOD = 1.5; //TODO move to constance

    private final Supplier<GyroData> gyroDataSupplier;
    private double collisionVectorSize = 0;
    private boolean isColliding = false;

    public CollisionDetector(Supplier<GyroData> gyroDataSupplierr) {
        this.gyroDataSupplier = gyroDataSupplierr;
    }

    public void updateForceVectorSize() {
        collisionVectorSize = Math.sqrt(Math.pow(getCollisionVector().getX(), 2) +
        Math.pow(getCollisionVector().getY(), 2)); 
        MALog.log("Pose Estimator/Collision/Collision Vector", collisionVectorSize);
    }

    private Translation2d getCollisionVector() {
        return new Translation2d(gyroDataSupplier.get().accelX, gyroDataSupplier.get().accelY);
    }
//TODO its better to just calculat the vector insted of creat every loop new Translation2d like you did at updateForceVectorSize()
    public void calculateCollision() {
        updateForceVectorSize();
        isColliding = collisionVectorSize> G_THRESHLOD;
        MALog.log("Pose Estimator/Collision/Is Colliding", isColliding);
        //TODO log the vectorsize also 
    }

    public boolean getIsColliding() {
        return isColliding;
    }

    public double getForceVector() {
        return collisionVectorSize;
    }

}
