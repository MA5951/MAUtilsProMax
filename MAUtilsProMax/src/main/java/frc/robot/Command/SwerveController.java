
package frc.robot.Command;

import com.MAutils.Swerve.SwerveSystemController;
import com.MAutils.Swerve.Utils.SwerveState;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConstants;

public class SwerveController extends SwerveSystemController {

    public SwerveController() {
        super(Swerve.getInstance(), SwerveConstants.SWERVE_CONSTANTS, RobotContainer.getDriverController());
    }

    public void ConfigControllers() {
    }

    public void SetSwerveState() {
        if (RobotContainer.getDriverController().getL2()) {
            setState(SwerveConstants.FIELD_CENTRIC_40);
                
        // } else if(RobotContainer.getDriverController().getR2()) {
        //     setState(SwerveConstants.ABS_CENTERING);
        // } else if(RobotContainer.getDriverController().getR1()  ) {
           
        //     setState(SwerveConstants.REL_CENTRING);
            
        // } else if(RobotContainer.getDriverController().getL1()) {]\[]
        //     setState(SwerveConstants.REL_UNLOCKED);
        } else {
            setState(SwerveConstants.FIELD_CENTRIC);
                
        } 
    }

}