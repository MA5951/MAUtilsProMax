
package com.MAutils.Swerve;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.MAutils.PoseEstimation.PoseEstimationMA;
import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.Swerve.Utils.PPHolonomicDriveController;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.SwerveConstants;

public class SwerveAutoFollower {

    private static RobotConfig config;

    public SwerveAutoFollower() {

        Swerve swerve = Swerve.getInstance();

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                () -> PoseEstimator.getCurrentPose(),
                pose -> PoseEstimator.resetPose(pose),
                () -> swerve.getChassisSpeeds(),
                (speeds, feedforwards) -> swerve.drive(speeds),
                new PPHolonomicDriveController(new PIDConstants(1.3), new PIDConstants(1)),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                }, swerve);


    }

    public static Command buildAuto(String autoName) {
        return new PathPlannerAuto(autoName);
    }

    public static Command followPath(String path) {
        try {
            return AutoBuilder.followPath(PathPlannerPath.fromPathFile(path));
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
        }
        return new InstantCommand();
    }

    public static Command findfollowPath(String path) {
        try {
            return AutoBuilder.pathfindThenFollowPath(PathPlannerPath.fromPathFile(path), SwerveConstants.constraints);
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
        }
        return new InstantCommand();
    }

    public static Command pathFindToPose(Pose2d targetPose, PathConstraints constraints) {
        return AutoBuilder.pathfindToPose(targetPose, constraints);
    }

    public static Pose2d getStartPose(String path) {
        try {
            return PathPlannerPath.fromPathFile(path).getStartingHolonomicPose().get();
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
        }
        return new Pose2d();
    }
}