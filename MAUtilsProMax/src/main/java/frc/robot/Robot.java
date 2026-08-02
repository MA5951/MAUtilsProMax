// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.MAutils.PoseEstimation.PoseEstimator;
import com.MAutils.RobotControl.DeafultRobot;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Util.Field;

public class Robot extends DeafultRobot {
  private Command m_autonomousCommand;
  private final RobotContainer m_robotContainer;
  private TalonFX motor;



  public Robot() {
    super();
    m_robotContainer = new RobotContainer();
    PoseEstimator.resetPose(Field.flipByAlliance(new Pose2d(3.586,3.596, Rotation2d.fromDegrees(-90))));

    motor = new TalonFX(3);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    CommandScheduler.getInstance().run();
  }

  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  
  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  
  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

 
}
