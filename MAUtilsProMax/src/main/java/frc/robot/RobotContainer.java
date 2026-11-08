
package frc.robot;

import com.MAutils.AutoChooser.AutoSelector;
import com.MAutils.Logger.MALog;
import com.MAutils.PoseEstimation.PoseEstimationMA;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.PoseEstimation.PoseEstimatorSource;
import com.MAutils.RobotControl.DeafultRobotContainer;
import com.MAutils.Swerve.SwerveAutoFollower;
import com.MAutils.Swerve.SwerveSystem;
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


    //NamedCommands.registerCommand("Intake",new InstantCommand(() -> RobotConstants.INTAKE_DEPLOY.setState()));

    autoSelector.setAutoOptions(RobotConstants.TEST);

  }

  @Override
  public void configAuto() {
    new SwerveAutoFollower();

    NamedCommands.registerCommand("coast",new InstantCommand(() ->Swerve.getInstance().isBrake(false)));
  }

  @Override
  public void configBinding() {
    new Trigger(() -> getDriverController().getActionsUp()).onTrue(
        new InstantCommand(() -> SwerveConstants.FIELD_CENTRIC_DRIVE.updateOffset()));

    new Trigger(() -> getDriverController().getDpadDown()).onTrue(
        new InstantCommand(() -> PoseEstimator.resetPose(VisionConstants.LL.getCameraIO()
            .getPoseEstimate(PoseEstimateType.MT1).pose)));

    new Trigger(() -> getDriverController().getDpadDown()).onTrue(
        new InstantCommand(() -> Swerve.getInstance().getSwerveDriveEstimator().reasetPose(VisionConstants.LL.getCameraIO()
            .getPoseEstimate(PoseEstimateType.MT1).pose)));

    new Trigger(() -> PoseEstimatorSource.isReaset()).onTrue(
        new InstantCommand(() -> Swerve.getInstance().getSwerveDriveEstimator().reasetPose(VisionConstants.LL.getCameraIO()
            .getPoseEstimate(PoseEstimateType.MT1).pose)));

    // new Trigger(() -> getDriverController().getDpadDown()).onTrue(
    //     new InstantCommand(() -> PoseEstimationMA.getInstance()
    //         .resetPose(VisionConstants.LL.getCameraIO()
    //             .getPoseEstimate(PoseEstimateType.MT1).pose)));
  }
}