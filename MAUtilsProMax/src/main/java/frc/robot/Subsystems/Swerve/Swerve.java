

package frc.robot.Subsystems.Swerve;

import com.MAutils.Logger.MALog;
import com.MAutils.Swerve.SwerveSystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class Swerve extends SwerveSystem{
    private static Swerve instance;

    private Swerve() {
        super(SwerveConstants.SWERVE_CONSTANTS); 
    }


    public double getVelocityVector() {
        return Math.sqrt(Math.pow(getChassisSpeeds().vxMetersPerSecond,2) + Math.pow(getChassisSpeeds().vyMetersPerSecond,2));
    }

   

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }

    @Override
    public void periodic() {
        super.periodic();
        MALog.log("Swerve/closest 90", SwerveConstants.clossest90());
    }



}