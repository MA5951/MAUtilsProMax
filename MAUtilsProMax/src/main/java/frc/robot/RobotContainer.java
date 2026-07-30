
package frc.robot;

import com.MAutils.RobotControl.DeafultRobotContainer;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Swerve.SwerveConstants;


public class RobotContainer extends DeafultRobotContainer{
  public RobotContainer() {
    super();
    
  }

  @Override
  public void configAuto() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'configAuto'");
  }

  @Override
  public void configBinding() {
    new Trigger(() -> getDriverController().getActionsUp()).onTrue(
                                new InstantCommand(() -> SwerveConstants.FIELD_CENTRIC_DRIVE.updateOffset()));
  }
}
