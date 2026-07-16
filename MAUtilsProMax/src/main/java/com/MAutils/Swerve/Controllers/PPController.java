
package com.MAutils.Swerve.Controllers;

import com.MAutils.PoseEstimation.PoseEstimationMA;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.Swerve.SwerveSystem;
import com.MAutils.Swerve.SwerveSystemConstants;
import com.MAutils.Swerve.Utils.PPHolonomicDriveController;
import com.MAutils.Swerve.Utils.SwerveController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PPController extends SwerveController {

    public PPController(SwerveSystemConstants constants, SwerveSystem swerveSystem,
            PPHolonomicDriveController holonomicDriveController) {
        super("PPController");

        AutoBuilder.configure(
                () -> PoseEstimator.getCurrentPose(),
                (pose2d) -> PoseEstimator.resetPose(pose2d),
                () -> swerveSystem.getChassisSpeeds(),
                (speeds, feedforwards) -> setChassisSpeeds(speeds),
                holonomicDriveController,
                constants.getRobotConfig(),
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, new SubsystemBase("Virtual PPController Subssystem") {

                });

    }

    public void startPath(String pathName) {
        try {
            AutoBuilder.followPath(PathPlannerPath.fromPathFile(pathName)).schedule();
        } catch (Exception e) {
            DriverStation.reportError("Path " + pathName + " Not Found", false);
        }
    }

    private void setChassisSpeeds(ChassisSpeeds speeds) {
        this.speeds = speeds;
    }



    public void updateSpeeds() {
    }

}
