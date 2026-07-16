
package com.MAutils.AutoChooser;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Util.Field;

public class AutoSelector {
    private final SendableChooser<Autonomus> autoChooser = new SendableChooser<>();

    public AutoSelector() {
        SmartDashboard.putData(autoChooser);
    }

    public void setAutoOptions(Autonomus... options) {
        autoChooser.setDefaultOption(options[0].getName(), options[0]);
        for (int i = 1; i < options.length; i++) {
            autoChooser.addOption(options[i].getName(), options[i]);
        }
    }

    public Autonomus getSelectedAuto() {
        if ( autoChooser.getSelected() == null) {
            return new Autonomus("NONE", new InstantCommand(), new Pose2d(3.586, Field.WIDTH / 2, new Rotation2d()), 0);
        }

        return autoChooser.getSelected();
    }
}
