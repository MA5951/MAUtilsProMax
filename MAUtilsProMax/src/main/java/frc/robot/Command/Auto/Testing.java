
package frc.robot.Command.Auto;

import com.MAutils.Swerve.SwerveAutoFollower;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class Testing extends SequentialCommandGroup {
  public Testing() {
    addCommands(
      SwerveAutoFollower.findfollowPath("P1")
    );
  }
}
