
package com.MAutils.Swerve;

import com.MAutils.Controllers.MAController;
import com.MAutils.Logger.MALog;
import com.MAutils.Swerve.Utils.SwerveState;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

public abstract class SwerveSystemController extends Command {
  protected SwerveSystem swerveSystem;
  protected SwerveSystemConstants constants;
  protected MAController drivController;
  private ChassisSpeeds currentSpeeds;

  public SwerveSystemController(SwerveSystem swerveSystem, SwerveSystemConstants constants ,MAController drivController) {
    super();
    this.constants = constants;
    this.drivController = drivController;
    this.swerveSystem = swerveSystem;
    addRequirements(swerveSystem);
    setName("SwerveSystemController");
    setState(new SwerveState("NONE"));

    
  }

  protected void setState(SwerveState state) {
    if (swerveSystem.getState() != state) {
      state.getOnStateEnter().run();
    }
    swerveSystem.setState(state);
  }

  public abstract void ConfigControllers();

  public abstract void SetSwerveState();

  public void initialize() {
    ConfigControllers();
  }

  public void execute() {
    SetSwerveState();

    swerveSystem.getState().getOnStateRuning().run();

    currentSpeeds = swerveSystem.getState().getSpeeds();
    logCurrentState();


    swerveSystem.drive(currentSpeeds);
  }

  public void end(boolean interrupted) {
    swerveSystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  public boolean isFinished() {
    return false;
  }

  private void logCurrentState() {
    MALog.log("Subsystems/Swerve/Controllers/SystemController/Current State Speeds", currentSpeeds);
  }
}
