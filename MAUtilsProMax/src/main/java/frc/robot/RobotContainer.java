
package frc.robot;

import com.MAutils.AutoChooser.AutoSelector;
import com.MAutils.Logger.MALog;
import com.MAutils.PoseEstimation.PoseEstimationMA;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.RobotControl.DeafultRobotContainer;
import com.MAutils.Swerve.SwerveAutoFollower;
import com.MAutils.Vision.IOs.VisionCameraIO.PoseEstimateType;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConstants;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionConstants;

public class RobotContainer extends DeafultRobotContainer {
  public static final AutoSelector autoSelector = new AutoSelector();

  public RobotContainer() {
    super();

    Vision.getInstance();

    new SwerveAutoFollower();

      //NamedCommands.registerCommand("Intake",new InstantCommand(() -> RobotConstants.INTAKE_DEPLOY.setState()));
    //NamedCommands.registerCommand("coast",new InstantCommand(() -> MALog.log("FAHHHHHHHHHH", "SHIT")));//Swerve.getInstance().isBrake(false)));
    autoSelector.setAutoOptions(RobotConstants.TEST);

  }

  @Override
  public void configAuto() {
      
  }

  @Override
  public void configBinding() {
    new Trigger(() -> getDriverController().getActionsUp()).onTrue(
        new InstantCommand(() -> SwerveConstants.FIELD_CENTRIC_DRIVE.updateOffset()));

    new Trigger(() -> getDriverController().getDpadDown()).onTrue(
        new InstantCommand(() -> PoseEstimator.resetPose(VisionConstants.LL.getCameraIO()
            .getPoseEstimate(PoseEstimateType.MT1).pose)));

    // new Trigger(() -> getDriverController().getDpadDown()).onTrue(
    //     new InstantCommand(() -> PoseEstimationMA.getInstance()
    //         .resetPose(VisionConstants.LL.getCameraIO()
    //             .getPoseEstimate(PoseEstimateType.MT1).pose)));
  }
}