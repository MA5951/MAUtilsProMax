

package frc.robot.Subsystems.Swerve;

import com.MAutils.Logger.MALog;
import com.MAutils.Swerve.SwerveSystem;

import edu.wpi.first.math.kinematics.ChassisSpeeds;


public class Swerve extends SwerveSystem{
    private static Swerve instance;
    private double lastSpeed;
    private double deltaSpeed;
    private double deltaCurrent;


    private boolean isRampFlag = false;

    private Swerve() {
        super(SwerveConstants.SWERVE_CONSTANTS);
        
    }


    public boolean atPointForShooting() {
        return true;
    }

    public boolean atPointForFeeding() {
        return true;
    }

    public boolean atPointForFeedingInMotion() {
        return true;
    }

    public double getVelocityVector() {
        return Math.sqrt(Math.pow(getChassisSpeeds().vxMetersPerSecond,2) + Math.pow(getChassisSpeeds().vyMetersPerSecond,2));
    }

   


    @Override
    public void periodic() {
        super.periodic();

        if (getTiltAngle() > 5) {
            isRampFlag = true;
        }

        MALog.log("/Subsystems/Swerve/tiltAngle", getTiltAngle());
    }

    public void resetRampFlag() {
        isRampFlag = false;
    }


    public boolean isRampFlag() {
        return isRampFlag;
    }

    public boolean isModuleStuck() {
        deltaSpeed = Math.abs(getCurrentStates()[1].speedMetersPerSecond) - lastSpeed;
        lastSpeed = Math.abs(lastSpeed);
        return (deltaSpeed < 0.03) && (Math.abs(getSwerveModuleData()[1].driveCurrent)> 15);
    }


   

    public static Swerve getInstance() {
        if (instance == null) {
            instance = new Swerve();
        }
        return instance;
    }



}