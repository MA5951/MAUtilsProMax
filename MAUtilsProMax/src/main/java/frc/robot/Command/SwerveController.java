
package frc.robot.Command;

import com.MAutils.Swerve.SwerveSystemController;

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
                
        } else {
            setState(SwerveConstants.FIELD_CENTRIC);
                
        } 
    }

}