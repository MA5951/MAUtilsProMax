
package frc.robot;

import com.MAutils.AutoChooser.Autonomus;
import com.MAutils.Swerve.SwerveAutoFollower;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Command.Auto.Testing;

public class RobotConstants {

    public static final Autonomus TEST = new Autonomus("Test", new Testing(), SwerveAutoFollower.getStartPose("P1"), 0);

}