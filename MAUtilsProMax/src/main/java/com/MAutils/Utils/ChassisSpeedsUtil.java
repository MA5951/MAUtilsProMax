
package com.MAutils.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class ChassisSpeedsUtil {
  
    public static ChassisSpeeds FromFieldToRobot(ChassisSpeeds speeds, Rotation2d robotAngle) {
        //TODO  this need to be at the swerve subsystem in my oppinain
        var rotated = new Translation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) //TODO why var?
                .rotateBy(robotAngle.unaryMinus());

        speeds.vxMetersPerSecond = rotated.getX(); 
        //TODO its bad to change the smae speeds object you change the passing object in all of the code 
        //use temp var
        speeds.vyMetersPerSecond = rotated.getY();
        return speeds;
    }

    public static double getSpeedMagnitude(ChassisSpeeds speeds) {
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }
    //TODO remaind my again why we dont use the wpi one? 
}
