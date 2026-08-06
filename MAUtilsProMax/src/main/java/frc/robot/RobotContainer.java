
package frc.robot;

import com.MAutils.PoseEstimation.PoseEstimationMA;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.RobotControl.DeafultRobotContainer;
import com.MAutils.Vision.IOs.VisionCameraIO.PoseEstimateType;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Swerve.SwerveConstants;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.VisionConstants;

public class RobotContainer extends DeafultRobotContainer {
  public RobotContainer() {
    super();

    Vision.getInstance();

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

    new Trigger(() -> getDriverController().getDpadDown()).onTrue(
        new InstantCommand(() -> PoseEstimationMA.getInstance()
            .resetPose(VisionConstants.LL.getCameraIO()
                .getPoseEstimate(PoseEstimateType.MT1).pose)));
  }
}
